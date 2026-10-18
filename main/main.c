// Bring-up test for the v7 BNO085 carrier on a XIAO ESP32-S3.
//
//   1. Pin check       : does each XIAO pin see what the netlist says is on
//                        it (R1/R2/R3 pull-ups, H_INTN driven by U1)?
//   2. Reset + H_INTN  : pulse NRST and see whether the BNO085 boots and
//                        announces itself on H_INTN (no I2C involved).
//   3. I2C scan        : who ACKs on the bus? Expect 0x4A (SA0 on GND).
//   4. Stream          : hand over to the sh2 driver and print sensor data
//                        over the USB serial port (imu_stream.c).
//
// If step 3 finds nothing, the whole sequence repeats every few seconds so
// the result is visible whenever a serial monitor gets attached.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "board.h"
#include "ble_link.h"
#include "afe.h"

bool imu_stream_run(uint8_t addr);   // imu_stream.c; returns only on failure
bool uart_sniff_run(void);           // uart_sniff.c; true if SDA carried UART data

static const char *TAG = "imu_v7";

// The datasheet gives ~90 ms from reset release to the first H_INTN. A fresh
// 32.768 kHz crystal can take far longer than that to start, so watch for
// a generous window before declaring the part dead.
// The D3 test clock sits on the header pin next to SDA and couples spikes
// onto it; with SCL high those look like START/STOP conditions and can wedge
// the BNO085's I2C slave. Only turn it on while fly-wiring it to XIN.
#define ENABLE_TEST_CLOCK 0

#define INT_WATCH_MS   2000
// A real announcement holds H_INTN low until the host reads the packet, so
// anything that lets go sooner than this was a glitch.
#define INT_HOLD_MS    300
#define RETRY_DELAY_MS 5000

// ---------------------------------------------------------------------------
// 32.768 kHz square wave on D3 for crystal troubleshooting. Fed into XIN32
// through ~10k it overrides X1, so if the BNO085 boots with it wired up, the
// chip is fine and the fault is in the X1/C5/C6 circuit. Harmless when the
// pin is left unconnected, so it always runs.
//
// With 2-bit resolution the LEDC divider is 80 MHz / (4 * 32768) =
// 610.3515625 = 610 + 90/256, which the S3's 10.8 fixed-point divider holds
// exactly: the average frequency is exactly 32768 Hz. (1-bit would need a
// divider of 1220.7, past the 10-bit integer part.)
static void start_test_clock(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_2_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 32768,
        .clk_cfg = LEDC_USE_APB_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    ledc_channel_config_t ch = {
        .gpio_num = PIN_TEST_CLK,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 2,                       // 2 of 4 = 50 %
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    ESP_LOGI(TAG, "test clock: 32.768 kHz on D3/GPIO4 (J1.4)");
}

// ---------------------------------------------------------------------------
typedef enum { PIN_HIGH, PIN_LOW, PIN_FLOAT } pin_state_t;
static const char *const PIN_STATE_STR[] = { "HIGH", "LOW", "FLOATING" };

typedef struct {
    pin_state_t sda, scl, nrst, intn;
    bool sda_drivable, scl_drivable;   // can the ESP32 pull the line low?
    bool sda_scl_bridged;              // does pulling one drag the other?
} pin_report_t;

// Read a pin once with the internal pull-down and once with the internal
// pull-up (~45k each). A board 10k pull-up or a chip driving the line wins
// against either; a pin with nothing on it simply follows the pull. This
// tells "the XIAO is not connected to what the netlist says" apart from
// "it is connected, but U1 is not doing anything".
static pin_state_t probe_pin(gpio_num_t pin)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    esp_rom_delay_us(500);
    int with_pd = gpio_get_level(pin);

    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en   = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&io));
    esp_rom_delay_us(500);
    int with_pu = gpio_get_level(pin);

    if (with_pd && with_pu)   return PIN_HIGH;
    if (!with_pd && !with_pu) return PIN_LOW;
    return PIN_FLOAT;
}

// Pull SDA and SCL low one at a time through open-drain outputs and read
// both back. The idle check above cannot see a solder bridge under U1 that
// ties a bus line to a 3V3 pin (18 sits right next to SCL on 19) or ties
// SDA to SCL (19/20): both still idle high, but no transfer can work.
// Pulling SDA low with SCL high is a START, releasing it a STOP; the BNO085
// just sees an empty transaction.
static void check_bus_drive(pin_report_t *r)
{
    gpio_set_level(PIN_SDA, 1);
    gpio_set_level(PIN_SCL, 1);
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_SDA) | (1ULL << PIN_SCL),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    esp_rom_delay_us(50);
    // A line that is already low cannot "follow" the other one; without this
    // a stuck-low SCL reads as a bridge.
    bool sda_idle_high = gpio_get_level(PIN_SDA) == 1;
    bool scl_idle_high = gpio_get_level(PIN_SCL) == 1;

    gpio_set_level(PIN_SDA, 0);
    esp_rom_delay_us(50);
    r->sda_drivable = gpio_get_level(PIN_SDA) == 0;
    bool scl_followed = scl_idle_high && gpio_get_level(PIN_SCL) == 0;
    gpio_set_level(PIN_SDA, 1);
    esp_rom_delay_us(50);

    gpio_set_level(PIN_SCL, 0);
    esp_rom_delay_us(50);
    r->scl_drivable = gpio_get_level(PIN_SCL) == 0;
    bool sda_followed = sda_idle_high && gpio_get_level(PIN_SDA) == 0;
    gpio_set_level(PIN_SCL, 1);
    esp_rom_delay_us(50);

    r->sda_scl_bridged = scl_followed || sda_followed;
}

// ---------------------------------------------------------------------------
// Actual line voltages, through ADC1 (GPIO1-10 are ADC1 channels on the S3).
// The digital reads above only say "below the input threshold"; a voltage
// tells a hard short from a leak, and with the 10k pull-up it gives the
// leak's resistance. NRST rests at 3V3 through R3 and doubles as the
// reference while U1 runs.
static int read_mv(adc_oneshot_unit_handle_t unit, adc_cali_handle_t cali, gpio_num_t pin)
{
    adc_unit_t u;
    adc_channel_t ch;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(pin, &u, &ch));
    adc_oneshot_chan_cfg_t cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(unit, ch, &cfg));
    int sum = 0;
    for (int i = 0; i < 8; i++) {
        int raw, mv;
        ESP_ERROR_CHECK(adc_oneshot_read(unit, ch, &raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali, raw, &mv));
        sum += mv;
    }
    return sum / 8;
}

// Resistance to GND implied by v_mv on a line pulled to 3V3 through 10k.
// Worked out against the nominal 3.3 V because the ADC saturates near 3.1 V.
static int leak_ohm(int v_mv)
{
    return v_mv >= 3300 ? 0 : 10000 * v_mv / (3300 - v_mv);
}

// ref_mv only decides "close enough to the rail to call it healthy".
static void report_leak(const char *name, int v_mv, int ref_mv)
{
    if (v_mv >= ref_mv - 200) return;
    if (v_mv < 50) {
        ESP_LOGE(TAG, "    -> %s is at GND: hard short", name);
    } else {
        ESP_LOGE(TAG, "    -> %s leaks ~%d ohm to GND (should be open)", name, leak_ohm(v_mv));
    }
}

static void adc_open(adc_oneshot_unit_handle_t *unit, adc_cali_handle_t *cali)
{
    adc_oneshot_unit_init_cfg_t ucfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, unit));
    adc_cali_curve_fitting_config_t ccfg = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&ccfg, cali));
}

static void adc_close(adc_oneshot_unit_handle_t unit, adc_cali_handle_t cali)
{
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(cali));
    ESP_ERROR_CHECK(adc_oneshot_del_unit(unit));
}

// Live readout for locating a leak by heating: a damaged junction leaks far
// more when warm, so warming the guilty chip (iron held near it, not
// touching) visibly drags the line down while warming the other does not.
static void live_voltmeter(int seconds)
{
    adc_oneshot_unit_handle_t unit;
    adc_cali_handle_t cali;
    adc_open(&unit, &cali);
    ESP_LOGW(TAG, "[1c] live SCL voltmeter for %d s: warm U1, then the XIAO, and watch", seconds);
    for (int i = 0; i < seconds * 4; i++) {
        int scl = read_mv(unit, cali, PIN_SCL);
        int sda = read_mv(unit, cali, PIN_SDA);
        printf("    t=%5.2f s  SCL %4d mV (~%5d ohm to GND)   SDA %4d mV\n",
               i / 4.0f, scl, leak_ohm(scl), sda);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    adc_close(unit, cali);
}

// Returns the lowest bus-line voltage seen with U1 held in reset.
static int measure_lines(void)
{
    adc_oneshot_unit_handle_t unit;
    adc_cali_handle_t cali;
    adc_open(&unit, &cali);

    int ref = read_mv(unit, cali, PIN_IMU_NRST);
    int sda_run = read_mv(unit, cali, PIN_SDA);
    int scl_run = read_mv(unit, cali, PIN_SCL);

    // U1's pins go high-Z in reset, so whatever still pulls a line down then
    // is on the board, not in U1.
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << PIN_IMU_NRST,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PIN_IMU_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    int sda_rst = read_mv(unit, cali, PIN_SDA);
    int scl_rst = read_mv(unit, cali, PIN_SCL);
    gpio_set_level(PIN_IMU_NRST, 1);
    adc_close(unit, cali);

    ESP_LOGI(TAG, "[1b] line voltages (3V3 ref via NRST = %d mV, ADC tops out ~3100):", ref);
    ESP_LOGI(TAG, "    SDA: U1 running %4d mV, U1 in reset %4d mV", sda_run, sda_rst);
    ESP_LOGI(TAG, "    SCL: U1 running %4d mV, U1 in reset %4d mV", scl_run, scl_rst);
    report_leak("SDA (U1 in reset)", sda_rst, ref);
    report_leak("SCL (U1 in reset)", scl_rst, ref);
    if (scl_rst >= ref - 200 && scl_run < ref - 200)
        ESP_LOGW(TAG, "    -> SCL recovers when U1 is in reset: U1 itself is holding it");
    if (sda_rst >= ref - 200 && sda_run < ref - 200)
        ESP_LOGW(TAG, "    -> SDA recovers when U1 is in reset: U1 itself is holding it");
    return sda_rst < scl_rst ? sda_rst : scl_rst;
}

static pin_report_t check_pins(void)
{
    pin_report_t r = {
        .sda  = probe_pin(PIN_SDA),
        .scl  = probe_pin(PIN_SCL),
        .nrst = probe_pin(PIN_IMU_NRST),     // probed before we start driving it
        .intn = probe_pin(PIN_IMU_INT),
    };
    ESP_LOGI(TAG, "[1] pin check (internal pull-down vs pull-up):");
    ESP_LOGI(TAG, "    SDA  D4/GPIO5 : %-8s expect HIGH (R1 10k)", PIN_STATE_STR[r.sda]);
    ESP_LOGI(TAG, "    SCL  D5/GPIO6 : %-8s expect HIGH (R2 10k)", PIN_STATE_STR[r.scl]);
    ESP_LOGI(TAG, "    NRST D1/GPIO2 : %-8s expect HIGH (R3 10k)", PIN_STATE_STR[r.nrst]);
    ESP_LOGI(TAG, "    INT  D0/GPIO1 : %-8s expect HIGH, or LOW if U1 has data waiting",
             PIN_STATE_STR[r.intn]);

    check_bus_drive(&r);
    ESP_LOGI(TAG, "    pull SDA low  : %-8s expect OK", r.sda_drivable ? "OK" : "STUCK HI");
    ESP_LOGI(TAG, "    pull SCL low  : %-8s expect OK", r.scl_drivable ? "OK" : "STUCK HI");
    ESP_LOGI(TAG, "    SDA vs SCL    : %-8s expect SEPARATE",
             r.sda_scl_bridged ? "BRIDGED" : "SEPARATE");

    if (measure_lines() < 2500) live_voltmeter(60);
    return r;
}

// ---------------------------------------------------------------------------
// Pulse NRST and time how long the BNO085 takes to pull H_INTN low. A part
// that boots sends an unsolicited SHTP advertisement and asserts H_INTN to
// announce it -- with no I2C traffic at all -- which separates "the chip is
// not running" from "the chip runs but I cannot reach it over I2C".
// Returns ms after reset release, or -1 if H_INTN never asserted.
static int reset_and_watch_int(void)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << PIN_IMU_NRST,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));

    // H_INTN is an output of the BNO085 -- never drive it. The pull-up only
    // gives a defined level if the chip is absent.
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << PIN_IMU_INT,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    gpio_set_level(PIN_IMU_NRST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_IMU_NRST, 1);

    int64_t t0 = esp_timer_get_time();
    int low_at_ms = -1;
    while ((esp_timer_get_time() - t0) < INT_WATCH_MS * 1000LL) {
        if (gpio_get_level(PIN_IMU_INT) == 0) {
            low_at_ms = (int)((esp_timer_get_time() - t0) / 1000);
            break;
        }
        esp_rom_delay_us(200);
    }
    if (low_at_ms >= 0) {
        int64_t t_low = esp_timer_get_time();
        while (gpio_get_level(PIN_IMU_INT) == 0 &&
               (esp_timer_get_time() - t_low) < INT_HOLD_MS * 1000LL) {
            esp_rom_delay_us(20);
        }
        int held_us = (int)(esp_timer_get_time() - t_low);
        if (held_us >= INT_HOLD_MS * 1000) {
            ESP_LOGI(TAG, "[2] H_INTN went LOW %d ms after reset and stayed low"
                     " -> BNO085 firmware is running", low_at_ms);
        } else {
            ESP_LOGW(TAG, "[2] H_INTN dipped LOW %d ms after reset but let go after %d us"
                     " -> a glitch, not the BNO085 announcing itself", low_at_ms, held_us);
            low_at_ms = -1;
        }
    } else {
        ESP_LOGE(TAG, "[2] H_INTN stayed HIGH for %d ms after reset -> BNO085 did not announce itself",
                 INT_WATCH_MS);
    }
    return low_at_ms;
}

// ---------------------------------------------------------------------------
// Full 7-bit scan. i2c_master_probe always runs at 100 kHz in IDF v6, so
// there is no point scanning at other speeds; 400 kHz gets exercised for
// real by the sh2 driver afterwards. Returns the BNO085 address, or 0.
static uint8_t scan_bus(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,   // R1/R2 on the board do this
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));

    ESP_LOGI(TAG, "[3] scanning I2C 0x08-0x77 ...");
    uint8_t bno = 0;
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 50) != ESP_OK) continue;
        const char *who = "  (unknown device)";
        if (addr == BNO_ADDR_SA0_LOW)  { who = "  <- BNO085, SA0=GND (expected)"; bno = addr; }
        if (addr == BNO_ADDR_SA0_HIGH) { who = "  <- BNO085, SA0=high (SA0 not grounded?)"; bno = addr; }
        if (addr == BNO_ADDR_BOOT_LOW || addr == BNO_ADDR_BOOT_HIGH)
            who = "  <- BNO085 BOOTLOADER (BOOTN read low at reset: check R4)";
        ESP_LOGI(TAG, "    found 0x%02X%s", addr, who);
        found++;
    }
    if (!found) {
        // NOT_FOUND is a clean NACK; TIMEOUT means the bus itself misbehaved
        // (a line held low, or SCL stretched and never released).
        esp_err_t e = i2c_master_probe(bus, BNO_ADDR_SA0_LOW, 100);
        ESP_LOGE(TAG, "    no device ACKed any address (0x%02X -> %s), H_INTN now %s",
                 BNO_ADDR_SA0_LOW, esp_err_to_name(e),
                 gpio_get_level(PIN_IMU_INT) ? "high/floating" : "LOW");
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
    return bno;
}

// ---------------------------------------------------------------------------
// Second half of the board: if the ADS1115 answers, sweep the mux once.
static void probe_front_end(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
    if (i2c_master_probe(bus, ADS1115_ADDR, 50) != ESP_OK) {
        ESP_LOGW(TAG, "[5] no ADS1115 at 0x%02X: resistance front end not fitted", ADS1115_ADDR);
    } else if (afe_init(bus)) {
        afe_probe();
        afe_deinit();
    }
    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}

// ---------------------------------------------------------------------------
static void explain_failure(const pin_report_t *p, int int_ms)
{
    ESP_LOGE(TAG, "=== BNO085 NOT FOUND on I2C ===");
    if (p->sda == PIN_FLOAT && p->scl == PIN_FLOAT && p->nrst == PIN_FLOAT) {
        ESP_LOGE(TAG, "None of the three 10k pull-ups is visible from the XIAO. Either 3V3");
        ESP_LOGE(TAG, "never reaches the carrier (J2.5) or the XIAO is not on the pins the");
        ESP_LOGE(TAG, "netlist assumes: check it is not mounted mirrored / rotated (USB end");
        ESP_LOGE(TAG, "vs silkscreen) and that J1/J2 are actually soldered.");
    } else if (p->sda != PIN_HIGH || p->scl != PIN_HIGH || p->nrst != PIN_HIGH) {
        ESP_LOGE(TAG, "Some pins do not see their pull-up (table [1]). A LOW line is a short");
        ESP_LOGE(TAG, "or a part holding it; a FLOATING line is an open between the XIAO pin");
        ESP_LOGE(TAG, "and the resistor. Fix those first: measure J1.2/J1.5/J1.6 to 3V3/GND.");
    } else if (!p->sda_drivable || !p->scl_drivable || p->sda_scl_bridged) {
        ESP_LOGE(TAG, "SDA/SCL cannot be driven independently (table [1]). That is a solder");
        ESP_LOGE(TAG, "bridge under U1: STUCK HI = tied to a 3V3 pin (SCL 19 sits next to");
        ESP_LOGE(TAG, "H_CSN 18, which is on 3V3); BRIDGED = pins 19 and 20 joined.");
    } else if (int_ms >= 0) {
        ESP_LOGE(TAG, "The part booted (H_INTN asserted) yet nothing ACKs at 0x4A/0x4B.");
        ESP_LOGE(TAG, "So it is alive but not on this I2C bus: suspect PS0/PS1 (U1.6/U1.5)");
        ESP_LOGE(TAG, "not reading GND (wrong interface mode), or U1.19/U1.20 open.");
    } else {
        ESP_LOGE(TAG, "XIAO-to-carrier wiring and 3V3 are fine (all three pull-ups seen),");
        ESP_LOGE(TAG, "so the fault is at U1 itself. It never announced itself%s.",
                 p->intn == PIN_FLOAT ? " and does not even drive H_INTN" : "");
        ESP_LOGE(TAG, "  - U1 orientation: pin-1 mark vs silkscreen (if machine assembled,");
        ESP_LOGE(TAG, "    check the assembler's placement preview for a rotated LGA)");
        ESP_LOGE(TAG, "  - multimeter: 3V3 on C2/C3; CAP pin (C4 to GND) ~1 V if U1 is alive,");
        ESP_LOGE(TAG, "    ~0 V if its core is not powered");
        ESP_LOGE(TAG, "  - LGA joints under U1: VDD 3, VDDIO 28, GND 2/25, NRST 11");
        ESP_LOGE(TAG, "  - X1 32.768 kHz not oscillating (scope on U1.27 with a 10x probe)");
    }
}

void app_main(void)
{
    // Give a freshly opened USB serial monitor a moment to attach.
    // Advertise from the start, so the host can connect while bring-up runs.
    ble_link_start();
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (ENABLE_TEST_CLOCK) start_test_clock();

    for (int attempt = 1;; attempt++) {
        ESP_LOGI(TAG, "======== BNO085 v7 board bring-up, attempt %d ========", attempt);

        pin_report_t pins = check_pins();
        int int_ms = reset_and_watch_int();
        uint8_t addr = scan_bus();

        if (addr) {
            probe_front_end();
            ESP_LOGI(TAG, "=== I2C OK: BNO085 at 0x%02X. Starting sensor stream ===", addr);
            imu_stream_run(addr);                  // does not return on success
            ESP_LOGE(TAG, "BNO085 ACKs its address but the sh2 driver could not talk to it");
            ble_link_log("E (0) BNO085 answers on I2C but sh2 failed - see serial log");
        } else {
            ble_link_log("E (0) BNO085 not found on I2C - check wiring, details on serial log");
            if (!uart_sniff_run()) explain_failure(&pins, int_ms);
        }

        ESP_LOGI(TAG, "retrying in %d s ...\n", RETRY_DELAY_MS / 1000);
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
    }
}
