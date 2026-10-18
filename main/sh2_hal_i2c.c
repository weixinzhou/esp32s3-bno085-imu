// sh2_Hal_t implementation for the BNO085 on ESP-IDF, I2C + H_INTN.
//
// SHTP over I2C works like this: when the BNO085 has a packet for the host it
// pulls H_INTN low. The host then reads a 4-byte SHTP header to learn the
// cargo length, and reads the whole packet -- header included, because the
// part re-sends the cargo from the start on the next read -- in a second
// transaction. Bit 15 of the length field is the continuation flag, not part
// of the length.
//
// The part also stretches SCL rather than NAKing when it is not ready, so
// scl_wait_us has to be generous or a stretching BNO085 is indistinguishable
// from an absent one.

#include "sh2_hal_i2c.h"
#include "sh2_err.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "sh2_hal";

#define SHTP_HEADER_LEN     4
#define SHTP_CONTINUATION   0x8000
#define XFER_TIMEOUT_MS     100
// Datasheet: t1 ~90 ms init + t2 ~4 ms config after reset release.
#define RESET_SETTLE_MS     200

typedef struct {
    sh2_Hal_t hal;              // must stay first: sh2 passes this back as self
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    gpio_num_t sda, scl, intn, nrst;
    uint8_t addr;
    uint32_t scl_hz;
    bool bus_ready;
} i2c_hal_t;

static i2c_hal_t g_hal;

// ---------------------------------------------------------------------------
static void pins_init(i2c_hal_t *h)
{
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << h->nrst,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    // H_INTN is driven by the BNO085. Input only, with a pull-up so the level
    // is defined if the part is missing.
    gpio_config_t in = {
        .pin_bit_mask = 1ULL << h->intn,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);
}

static void reset_cycle(i2c_hal_t *h)
{
    // BOOTN is held high by R4 on this board, so a plain NRST pulse always
    // lands in the application, never the bootloader.
    gpio_set_level(h->nrst, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(h->nrst, 1);
    vTaskDelay(pdMS_TO_TICKS(RESET_SETTLE_MS));
}

// ---------------------------------------------------------------------------
static int hal_open(sh2_Hal_t *self)
{
    i2c_hal_t *h = (i2c_hal_t *)self;

    if (!h->bus_ready) {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = -1,
            .sda_io_num = h->sda,
            .scl_io_num = h->scl,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = false,   // R1/R2 on the board
        };
        if (i2c_new_master_bus(&bus_cfg, &h->bus) != ESP_OK) {
            ESP_LOGE(TAG, "i2c_new_master_bus failed");
            return SH2_ERR;
        }
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = h->addr,
            .scl_speed_hz = h->scl_hz,
            // The BNO085 stretches the clock instead of NAKing when it has no
            // data. The default register timeout is far too short for that.
            .scl_wait_us = 100000,
        };
        if (i2c_master_bus_add_device(h->bus, &dev_cfg, &h->dev) != ESP_OK) {
            ESP_LOGE(TAG, "i2c_master_bus_add_device failed");
            i2c_del_master_bus(h->bus);
            return SH2_ERR;
        }
        h->bus_ready = true;
    }

    pins_init(h);
    reset_cycle(h);
    return SH2_OK;
}

static void hal_close(sh2_Hal_t *self)
{
    i2c_hal_t *h = (i2c_hal_t *)self;
    gpio_set_level(h->nrst, 0);          // hold the part in reset
    if (h->bus_ready) {
        i2c_master_bus_rm_device(h->dev);
        i2c_del_master_bus(h->bus);
        h->bus_ready = false;
    }
}

static int hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
{
    i2c_hal_t *h = (i2c_hal_t *)self;

    // Nothing to collect unless the part has asserted its interrupt.
    if (gpio_get_level(h->intn) != 0) return 0;
    *t_us = (uint32_t)esp_timer_get_time();

    uint8_t hdr[SHTP_HEADER_LEN];
    if (i2c_master_receive(h->dev, hdr, sizeof(hdr), XFER_TIMEOUT_MS) != ESP_OK) return 0;

    uint16_t cargo = (uint16_t)((hdr[1] << 8) | hdr[0]) & ~SHTP_CONTINUATION;
    if (cargo <= SHTP_HEADER_LEN) return 0;      // header-only / empty packet
    if (cargo > len) {
        // sh2's buffer is smaller than this packet. Reading a truncated cargo
        // would desync SHTP, so drop it and say nothing arrived.
        ESP_LOGW(TAG, "packet of %u B exceeds the %u B sh2 buffer, dropped", cargo, len);
        return 0;
    }

    // Re-read from the start: the cargo includes the header again.
    if (i2c_master_receive(h->dev, pBuffer, cargo, XFER_TIMEOUT_MS) != ESP_OK) return 0;
    return cargo;
}

static int hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    i2c_hal_t *h = (i2c_hal_t *)self;
    if (len == 0) return 0;
    if (i2c_master_transmit(h->dev, pBuffer, len, XFER_TIMEOUT_MS) != ESP_OK) return 0;
    return len;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self)
{
    (void)self;
    return (uint32_t)esp_timer_get_time();
}

// ---------------------------------------------------------------------------
sh2_Hal_t *sh2_hal_i2c_create(gpio_num_t sda, gpio_num_t scl,
                              gpio_num_t intn, gpio_num_t nrst,
                              uint8_t i2c_addr, uint32_t scl_hz)
{
    memset(&g_hal, 0, sizeof(g_hal));
    g_hal.hal.open      = hal_open;
    g_hal.hal.close     = hal_close;
    g_hal.hal.read      = hal_read;
    g_hal.hal.write     = hal_write;
    g_hal.hal.getTimeUs = hal_get_time_us;
    g_hal.sda = sda; g_hal.scl = scl;
    g_hal.intn = intn; g_hal.nrst = nrst;
    g_hal.addr = i2c_addr;
    g_hal.scl_hz = scl_hz;
    return &g_hal.hal;
}

i2c_master_bus_handle_t sh2_hal_i2c_bus(void)
{
    return g_hal.bus_ready ? g_hal.bus : NULL;
}
