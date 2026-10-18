// Is the BNO085 talking UART instead of I2C?
//
// PS1/PS0 (U1.5/U1.6) are sampled at reset and select the host interface.
// Both are on GND on this board, but they sit in the same corner as the
// supply pins that were already open once, and an open strap can read as 1:
//   PS1 PS0 = 0 1 -> UART-RVC  : streams 19-byte frames, 0xAA 0xAA header,
//                                100 Hz at 115200 baud, H_INTN unused
//   PS1 PS0 = 1 0 -> UART-SHTP : 3 Mbaud, sends an advertisement after reset
// Either way the part never ACKs I2C and leaves H_INTN floating -- exactly
// what this board shows. Its UART TX is pin 20 = H_SDA = D4/GPIO5, so just
// listen there.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "board.h"

static const char *TAG = "uart_sniff";

#define BASELINE_MS 200
#define LISTEN_MS   400

// Count bytes for ms milliseconds, printing the first few.
static int collect(int baud, const char *phase, int ms, int *rvc_headers)
{
    uint8_t buf[256];
    int total = 0, shown = 0;
    TickType_t end = xTaskGetTickCount() + pdMS_TO_TICKS(ms);
    while (xTaskGetTickCount() < end) {
        int n = uart_read_bytes(UART_NUM_1, buf, sizeof(buf), pdMS_TO_TICKS(50));
        if (n <= 0) continue;
        total += n;
        for (int i = 0; i + 1 < n; i++)
            if (buf[i] == 0xAA && buf[i + 1] == 0xAA) (*rvc_headers)++;
        if (shown < 24) {
            printf("    %d baud, %s:", baud, phase);
            for (int i = 0; i < n && shown < 24; i++, shown++) printf(" %02X", buf[i]);
            printf("\n");
        }
    }
    return total;
}

// Returns the number of bytes attributable to U1, i.e. beyond the baseline.
//
// The baseline is taken with U1 held in reset, when it cannot transmit. That
// matters because the D3 test clock runs on the header pin right next to SDA,
// and its edges couple onto SDA as short spikes that a 3 Mbaud receiver
// happily decodes as 0xFE/0xFC bytes. Only traffic that appears after reset
// is released is really coming from the BNO085.
static int listen_at(int baud)
{
    uart_config_t cfg = {
        .baud_rate = baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART_PIN_NO_CHANGE, PIN_SDA,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    gpio_config_t out = {
        .pin_bit_mask = 1ULL << PIN_IMU_NRST,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    int rvc_headers = 0, ignored = 0;
    gpio_set_level(PIN_IMU_NRST, 0);                 // U1 silent: noise only
    vTaskDelay(pdMS_TO_TICKS(5));
    uart_flush_input(UART_NUM_1);
    int baseline = collect(baud, "U1 in reset", BASELINE_MS, &ignored);

    // Release reset with the receiver already listening so a one-shot SHTP
    // advertisement right after boot is not missed.
    gpio_set_level(PIN_IMU_NRST, 1);
    int active = collect(baud, "U1 running ", LISTEN_MS, &rvc_headers);
    uart_driver_delete(UART_NUM_1);

    // Scale the baseline to the longer listening window before comparing.
    int noise = baseline * LISTEN_MS / BASELINE_MS;
    int excess = active - noise;
    if (active == 0) {
        ESP_LOGI(TAG, "    %7d baud: silent", baud);
    } else if (rvc_headers > 0) {
        ESP_LOGW(TAG, "    %7d baud: %d bytes, %d x 'AA AA' -> UART-RVC: PS0 (U1.6) is NOT reading GND",
                 baud, active, rvc_headers);
        return active;
    } else if (excess <= active / 4 + 8) {
        ESP_LOGI(TAG, "    %7d baud: %d bytes running vs %d in reset -> same noise either way,"
                 " not U1 (test-clock crosstalk onto SDA?)", baud, active, baseline);
    } else {
        ESP_LOGW(TAG, "    %7d baud: %d bytes running vs %d in reset -> U1 is transmitting:"
                 " UART mode, PS0/PS1 (U1.6/U1.5) not reading GND", baud, active, baseline);
        return excess;
    }
    return 0;
}

bool uart_sniff_run(void)
{
    ESP_LOGI(TAG, "[3b] listening on SDA (U1.20 = UART TX in UART modes) ...");
    int any = listen_at(115200) + listen_at(3000000);
    if (!any) ESP_LOGI(TAG, "    no UART traffic from U1 on SDA: not a PS0/PS1 strap problem");
    return any > 0;
}
