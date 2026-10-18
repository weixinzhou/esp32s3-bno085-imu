// NimBLE peripheral for the IMU stream. Advertises the service UUID (for the
// host to filter on) with the name in the scan response, keeps one central,
// and notifies DATA / LOG only while the central has subscribed to them.
// Everything else stays on the serial console as before.

#include "ble_link.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "ble_link";
static const char *DEVICE_NAME = "IMU-BNO085";

// Full power (+20 dBm, default is +9). 2.4 GHz loses roughly 2-3 dB per cm
// of fresh water, so a shallow-diving robot needs every dB. Costs current
// on transmit; drop to ESP_PWR_LVL_P9 for bench use on a small battery.
#define BLE_TX_POWER ESP_PWR_LVL_P20

// 7a4c000N-5b2e-4f6d-9c3a-2e1d8b6f0a51, bytes little-endian.
#define IMU_UUID(n) BLE_UUID128_INIT(0x51, 0x0a, 0x6f, 0x8b, 0x1d, 0x2e, 0x3a, 0x9c, \
                                     0x6d, 0x4f, 0x2e, 0x5b, (n), 0x00, 0x4c, 0x7a)
static const ble_uuid128_t SVC_UUID = IMU_UUID(0x01);
static const ble_uuid128_t DATA_UUID = IMU_UUID(0x02);
static const ble_uuid128_t LOG_UUID = IMU_UUID(0x03);

static uint16_t s_data_handle, s_log_handle;
static volatile uint16_t s_conn = BLE_HS_CONN_HANDLE_NONE;
static volatile bool s_data_on, s_log_on;
static uint8_t s_addr_type;

// Notify-only characteristics; a read just returns nothing.
static int chr_access(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;
}

static const struct ble_gatt_svc_def SERVICES[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            { .uuid = &DATA_UUID.u, .access_cb = chr_access,
              .val_handle = &s_data_handle, .flags = BLE_GATT_CHR_F_NOTIFY },
            { .uuid = &LOG_UUID.u, .access_cb = chr_access,
              .val_handle = &s_log_handle, .flags = BLE_GATT_CHR_F_NOTIFY },
            { 0 },
        },
    },
    { 0 },
};

static int gap_event(struct ble_gap_event *event, void *arg);

static void advertise(void)
{
    struct ble_hs_adv_fields adv = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .uuids128 = &SVC_UUID,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    // Flags + a 128-bit UUID already use 21 of the 31 advertising bytes, so
    // the name goes in the scan response.
    struct ble_hs_adv_fields rsp = {
        .name = (const uint8_t *)DEVICE_NAME,
        .name_len = strlen(DEVICE_NAME),
        .name_is_complete = 1,
    };
    int rc = ble_gap_adv_set_fields(&adv);
    if (rc == 0) rc = ble_gap_adv_rsp_set_fields(&rsp);
    struct ble_gap_adv_params params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    if (rc == 0) rc = ble_gap_adv_start(s_addr_type, NULL, BLE_HS_FOREVER, &params, gap_event, NULL);
    if (rc != 0) ESP_LOGE(TAG, "advertising failed (%d)", rc);
    else ESP_LOGI(TAG, "advertising as \"%s\"", DEVICE_NAME);
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            advertise();
            break;
        }
        s_conn = event->connect.conn_handle;
        ESP_LOGI(TAG, "connected");
        // The default power type covers new links; set this one explicitly
        // too where the controller has a slot for its handle (0-8).
        if (s_conn <= 8) esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0 + s_conn, BLE_TX_POWER);
        // 15-30 ms interval: 50 samples/s then go out in one or two events
        // each instead of waiting behind a slow default interval.
        struct ble_gap_upd_params p = {
            .itvl_min = 12, .itvl_max = 24, .latency = 0, .supervision_timeout = 400,
        };
        ble_gap_update_params(s_conn, &p);
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnected (reason %d)", event->disconnect.reason);
        s_conn = BLE_HS_CONN_HANDLE_NONE;
        s_data_on = s_log_on = false;
        advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == s_data_handle) s_data_on = event->subscribe.cur_notify;
        if (event->subscribe.attr_handle == s_log_handle) s_log_on = event->subscribe.cur_notify;
        break;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU %d", event->mtu.value);
        break;
    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    if (ble_hs_id_infer_auto(0, &s_addr_type) != 0) {
        ESP_LOGE(TAG, "no usable BLE address");
        return;
    }
    advertise();
}

static void host_task(void *param)
{
    nimble_port_run();                  // returns only on nimble_port_stop()
    nimble_port_freertos_deinit();
}

void ble_link_start(void)
{
    // NVS holds the radio's PHY calibration.
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    if (nimble_port_init() != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE init failed; continuing on serial only");
        return;
    }
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, BLE_TX_POWER);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, BLE_TX_POWER);
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(SERVICES));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(SERVICES));
    ble_svc_gap_device_name_set(DEVICE_NAME);
    nimble_port_freertos_init(host_task);
}

bool ble_link_connected(void)
{
    return s_conn != BLE_HS_CONN_HANDLE_NONE;
}

// NimBLE host calls are safe from any task. If the link is slower than the
// sensor the mbuf pool runs dry and samples are dropped, which is what a
// live display wants rather than a growing backlog.
static void notify(uint16_t handle, const void *data, size_t len)
{
    uint16_t conn = s_conn;
    if (conn == BLE_HS_CONN_HANDLE_NONE) return;
    size_t room = ble_att_mtu(conn) - 3;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len < room ? len : room);
    if (om) ble_gatts_notify_custom(conn, handle, om);
}

void ble_link_send_sample(const ble_sample_t *s)
{
    if (s_data_on) notify(s_data_handle, s, sizeof(*s));
}

void ble_link_log(const char *fmt, ...)
{
    if (!s_log_on) return;
    char buf[200];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) notify(s_log_handle, buf, n < (int)sizeof(buf) ? (size_t)n : sizeof(buf) - 1);
}
