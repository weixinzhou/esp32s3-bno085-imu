// Resistance front end.
//
// MEAS_NODE is pulled to 3V3 through R_REF; the mux connects the selected
// FPC channel's sensor from MEAS_NODE to GND, so the node sits on a divider:
//     V = Vsupply * R / (R_REF + R)   ->   R = R_REF * V / (Vsupply - V)
// Vsupply is not taken on trust: an unrouted mux input leaves MEAS_NODE on
// the pull-up alone, which reads the supply through the same path.
//
// The mux's on-resistance (~70 ohm for an HC4067 at 3.3 V) sits in series
// with every sensor and is included in R.

#include "afe.h"

#include <math.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "board.h"
#include "ble_link.h"

static const char *TAG = "afe";

// ADS1115 config: start single shot, AIN0 vs GND, +/-4.096 V full scale,
// single-shot mode, 860 SPS, comparator off.
#define ADS_REG_CONV    0x00
#define ADS_REG_CONFIG  0x01
#define ADS_CFG_AIN0    0xC3E3
#define ADS_MV_PER_LSB  0.125f        // 4.096 V / 32768
#define ADS_CONV_US     1300          // 860 SPS -> 1.16 ms, plus margin
#define MUX_SETTLE_US   100

static i2c_master_dev_handle_t s_ads;

static const gpio_num_t SELECT[4] = { PIN_MUX_S0, PIN_MUX_S1, PIN_MUX_S2, PIN_MUX_S3 };

bool afe_init(i2c_master_bus_handle_t bus)
{
    // Weakest drive: GPIO44 is UART0 RX, which on a dev board is wired to
    // the USB-UART bridge's TX. On a XIAO it is free, and the HC4067's CMOS
    // select inputs need no current anyway.
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_MUX_S0) | (1ULL << PIN_MUX_S1) |
                        (1ULL << PIN_MUX_S2) | (1ULL << PIN_MUX_S3),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    for (int i = 0; i < 4; i++) gpio_set_drive_capability(SELECT[i], GPIO_DRIVE_CAP_0);

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ADS1115_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(bus, &dev, &s_ads) != ESP_OK) {
        ESP_LOGE(TAG, "could not add ADS1115 at 0x%02X", ADS1115_ADDR);
        return false;
    }
    return true;
}

void afe_deinit(void)
{
    if (s_ads) i2c_master_bus_rm_device(s_ads);
    s_ads = NULL;
}

static void select_channel(int ch)
{
    for (int i = 0; i < 4; i++) gpio_set_level(SELECT[i], (ch >> i) & 1);
}

bool afe_read_mv(int ch, float *mv)
{
    select_channel(ch);
    esp_rom_delay_us(MUX_SETTLE_US);

    uint8_t cfg[3] = { ADS_REG_CONFIG, ADS_CFG_AIN0 >> 8, ADS_CFG_AIN0 & 0xFF };
    if (i2c_master_transmit(s_ads, cfg, sizeof(cfg), 20) != ESP_OK) return false;
    esp_rom_delay_us(ADS_CONV_US);

    uint8_t reg = ADS_REG_CONV, raw[2];
    if (i2c_master_transmit_receive(s_ads, &reg, 1, raw, 2, 20) != ESP_OK) return false;
    *mv = (int16_t)((raw[0] << 8) | raw[1]) * ADS_MV_PER_LSB;
    return true;
}

void afe_probe(void)
{
    float v[16];
    bool ok[16];
    for (int ch = 0; ch < 16; ch++) ok[ch] = afe_read_mv(ch, &v[ch]);

    if (!ok[AFE_OPEN_CH]) {
        ESP_LOGE(TAG, "[5] ADS1115 not answering conversions");
        ble_link_log("E (0) ADS1115 at 0x%02X does not answer conversions", ADS1115_ADDR);
        return;
    }
    float vs = v[AFE_OPEN_CH];
    ESP_LOGI(TAG, "[5] resistance front end: supply via open input I%d = %.1f mV, R_ref %.0f ohm (assumed)",
             AFE_OPEN_CH, vs, AFE_R_REF_OHM);
    for (int ch = 0; ch < 16; ch++) {
        const char *name = ch < AFE_CHANNELS ? "RCH" : "    ";
        if (!ok[ch]) {
            ESP_LOGE(TAG, "    I%-2d %s%-2s  read failed", ch, name, "");
            continue;
        }
        char tag[8] = "";
        if (ch < AFE_CHANNELS) snprintf(tag, sizeof(tag), "%d", ch);
        float d = vs - v[ch];
        if (d < vs * 0.01f) {
            ESP_LOGI(TAG, "    I%-2d %s%-2s %7.1f mV  open (nothing connected)", ch, name, tag, v[ch]);
        } else if (v[ch] < 2.0f) {
            ESP_LOGI(TAG, "    I%-2d %s%-2s %7.1f mV  short to GND", ch, name, tag, v[ch]);
        } else {
            float r = AFE_R_REF_OHM * v[ch] / d;
            ESP_LOGI(TAG, "    I%-2d %s%-2s %7.1f mV  %8.0f ohm", ch, name, tag, v[ch], r);
        }
    }
}

// ---------------------------------------------------------------------------
static int s_slot;
static bool s_busy;
static int64_t s_started;
static float s_mv[AFE_SLOTS];

static int slot_channel(int slot) { return slot < AFE_CHANNELS ? slot : AFE_OPEN_CH; }

static void start_slot(void)
{
    select_channel(slot_channel(s_slot));
    esp_rom_delay_us(MUX_SETTLE_US);
    uint8_t cfg[3] = { ADS_REG_CONFIG, ADS_CFG_AIN0 >> 8, ADS_CFG_AIN0 & 0xFF };
    if (i2c_master_transmit(s_ads, cfg, sizeof(cfg), 20) != ESP_OK) return;
    s_started = esp_timer_get_time();
    s_busy = true;
}

void afe_poll(void)
{
    if (!s_ads) return;
    if (!s_busy) {
        start_slot();
        return;
    }
    if (esp_timer_get_time() - s_started < ADS_CONV_US) return;
    uint8_t reg = ADS_REG_CONV, raw[2];
    if (i2c_master_transmit_receive(s_ads, &reg, 1, raw, 2, 20) == ESP_OK)
        s_mv[s_slot] = (int16_t)((raw[0] << 8) | raw[1]) * ADS_MV_PER_LSB;
    s_busy = false;
    s_slot = (s_slot + 1) % AFE_SLOTS;
    // Start the next channel right away rather than on the next loop pass,
    // which roughly doubles the sweep rate.
    start_slot();
}

bool afe_latest(float mv[AFE_SLOTS])
{
    for (int i = 0; i < AFE_SLOTS; i++) mv[i] = s_ads ? s_mv[i] : 0.0f;
    return s_ads != NULL;
}
