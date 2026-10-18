// Read the BNO085 through CEVA's sh2 driver and print the data over the USB
// serial port as CSV.
//
// Enabled reports: rotation vector (fused, uses the magnetometer for heading),
// game rotation vector (no magnetometer: drifts slowly in heading but never
// jumps near steel or electronics), calibrated accelerometer, gyroscope and
// magnetometer, and linear acceleration (gravity removed by the BNO085's own
// fusion). Each arrives as its own sh2 event; the callback keeps the latest
// of each and emits one combined line on every OUTPUT_HZ tick of the
// rotation vector. New columns go on the end so older parsers still line up.
//
// Lines starting with '#' are commentary (column header, per-sensor rates),
// so a host script can skip them and parse the rest as CSV.

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_hal_i2c.h"
#include "board.h"
#include "ble_link.h"
#include "afe.h"

static const char *TAG = "imu_stream";

#define REPORT_HZ        100     // rotation vectors, accel, gyro, linear accel
#define MAG_HZ           50
#define OUTPUT_HZ        50      // CSV lines per second; must divide REPORT_HZ
#define STATS_PERIOD_MS  5000

// 10k pull-ups are on the slow side for 400 kHz. Try it first, and fall back
// to 100 kHz if the sh2 handshake does not get through.
static const uint32_t SCL_SPEEDS_HZ[] = { 400000, 100000 };

enum { CNT_RV, CNT_ACC, CNT_GYRO, CNT_MAG, CNT_N };

static struct {
    float qw, qx, qy, qz;
    float ax, ay, az;          // m/s^2
    float gx, gy, gz;          // rad/s
    float mx, my, mz;          // uT
    uint8_t rv_status, mag_status;   // 0 unreliable .. 3 high accuracy
    float gqw, gqx, gqy, gqz;  // game rotation vector
    float lx, ly, lz;          // linear acceleration, m/s^2, sensor frame
} s_imu = { .qw = 1.0f, .gqw = 1.0f };

static uint32_t s_count[CNT_N];
static volatile bool s_saw_reset;
static uint32_t s_resets;

// Reset cause codes reported in the product ID response (SH-2 reference
// manual, Product ID Response). Tells a power dip from a crash from a
// glitch on NRST.
static const char *reset_cause_str(uint8_t cause)
{
    switch (cause) {
    case 1:  return "power-on reset (supply dipped or a power joint lost contact)";
    case 2:  return "internal system reset";
    case 3:  return "watchdog timeout (BNO085 firmware hung)";
    case 4:  return "external reset (NRST was pulled low)";
    case 5:  return "other";
    default: return "not reported";
    }
}

// ---------------------------------------------------------------------------
// Float to the BNO085's fixed-point Q format, saturating.
static int16_t q_fmt(float v, int q)
{
    float x = v * (float)(1 << q);
    return x > 32767.0f ? 32767 : x < -32768.0f ? -32768 : (int16_t)lroundf(x);
}

static void send_ble(uint64_t t_us)
{
    if (!ble_link_connected()) return;
    ble_sample_t b = {
        .t_ms = (uint32_t)(t_us / 1000),
        .q = { q_fmt(s_imu.qw, 14), q_fmt(s_imu.qx, 14), q_fmt(s_imu.qy, 14), q_fmt(s_imu.qz, 14) },
        .acc = { q_fmt(s_imu.ax, 8), q_fmt(s_imu.ay, 8), q_fmt(s_imu.az, 8) },
        .gyr = { q_fmt(s_imu.gx, 9), q_fmt(s_imu.gy, 9), q_fmt(s_imu.gz, 9) },
        .mag = { q_fmt(s_imu.mx, 4), q_fmt(s_imu.my, 4), q_fmt(s_imu.mz, 4) },
        .rv_status = s_imu.rv_status,
        .mag_status = s_imu.mag_status,
        .gq = { q_fmt(s_imu.gqw, 14), q_fmt(s_imu.gqx, 14), q_fmt(s_imu.gqy, 14), q_fmt(s_imu.gqz, 14) },
        .lin = { q_fmt(s_imu.lx, 8), q_fmt(s_imu.ly, 8), q_fmt(s_imu.lz, 8) },
    };
    float mv[AFE_SLOTS];
    afe_latest(mv);
    for (int i = 0; i < AFE_SLOTS; i++) b.afe[i] = (int16_t)lroundf(mv[i] / 0.125f);
    ble_link_send_sample(&b);
}

static void print_line(uint64_t t_us)
{
    send_ble(t_us);

    float r = s_imu.qw, i = s_imu.qx, j = s_imu.qy, k = s_imu.qz;
    float yaw   = atan2f(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k)) * 57.29578f;
    float pitch = asinf(fmaxf(-1.0f, fminf(1.0f, 2.0f * (r * j - k * i))))     * 57.29578f;
    float roll  = atan2f(2.0f * (r * i + j * k), 1.0f - 2.0f * (i * i + j * j)) * 57.29578f;

    printf("%lu,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%u,%u,"
           "%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f",
           (unsigned long)(t_us / 1000),
           r, i, j, k, yaw, pitch, roll,
           s_imu.ax, s_imu.ay, s_imu.az,
           s_imu.gx, s_imu.gy, s_imu.gz,
           s_imu.mx, s_imu.my, s_imu.mz,
           s_imu.rv_status, s_imu.mag_status,
           s_imu.gqw, s_imu.gqx, s_imu.gqy, s_imu.gqz,
           s_imu.lx, s_imu.ly, s_imu.lz);
    // Resistance front end, millivolts: RCH0-5 then the supply (0 if absent).
    float mv[AFE_SLOTS];
    afe_latest(mv);
    printf(",%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", mv[0], mv[1], mv[2], mv[3], mv[4], mv[5], mv[6]);
}

static void sensor_cb(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t v;
    if (sh2_decodeSensorEvent(&v, event) != SH2_OK) return;

    switch (v.sensorId) {
    case SH2_ACCELEROMETER:
        s_imu.ax = v.un.accelerometer.x;
        s_imu.ay = v.un.accelerometer.y;
        s_imu.az = v.un.accelerometer.z;
        s_count[CNT_ACC]++;
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        s_imu.gx = v.un.gyroscope.x;
        s_imu.gy = v.un.gyroscope.y;
        s_imu.gz = v.un.gyroscope.z;
        s_count[CNT_GYRO]++;
        break;
    case SH2_MAGNETIC_FIELD_CALIBRATED:
        s_imu.mx = v.un.magneticField.x;
        s_imu.my = v.un.magneticField.y;
        s_imu.mz = v.un.magneticField.z;
        s_imu.mag_status = v.status & 0x03;
        s_count[CNT_MAG]++;
        break;
    case SH2_ROTATION_VECTOR:
        s_imu.qw = v.un.rotationVector.real;
        s_imu.qx = v.un.rotationVector.i;
        s_imu.qy = v.un.rotationVector.j;
        s_imu.qz = v.un.rotationVector.k;
        s_imu.rv_status = v.status & 0x03;
        if (++s_count[CNT_RV] % (REPORT_HZ / OUTPUT_HZ) == 0) print_line(v.timestamp);
        break;
    case SH2_GAME_ROTATION_VECTOR:
        s_imu.gqw = v.un.gameRotationVector.real;
        s_imu.gqx = v.un.gameRotationVector.i;
        s_imu.gqy = v.un.gameRotationVector.j;
        s_imu.gqz = v.un.gameRotationVector.k;
        break;
    case SH2_LINEAR_ACCELERATION:
        s_imu.lx = v.un.linearAcceleration.x;
        s_imu.ly = v.un.linearAcceleration.y;
        s_imu.lz = v.un.linearAcceleration.z;
        break;
    default:
        break;
    }
}

static void event_cb(void *cookie, sh2_AsyncEvent_t *evt)
{
    (void)cookie;
    if (evt->eventId == SH2_RESET) {
        // Every sensor config is lost on a reset and has to be re-sent.
        s_saw_reset = true;
    }
}

static int enable_reports(void)
{
    static const struct { sh2_SensorId_t id; uint32_t hz; } reports[] = {
        { SH2_ROTATION_VECTOR,           REPORT_HZ },
        { SH2_ACCELEROMETER,             REPORT_HZ },
        { SH2_GYROSCOPE_CALIBRATED,      REPORT_HZ },
        { SH2_MAGNETIC_FIELD_CALIBRATED, MAG_HZ },
        { SH2_GAME_ROTATION_VECTOR,      REPORT_HZ },
        { SH2_LINEAR_ACCELERATION,       REPORT_HZ },
    };
    for (size_t n = 0; n < sizeof(reports) / sizeof(reports[0]); n++) {
        sh2_SensorConfig_t cfg = { .reportInterval_us = 1000000 / reports[n].hz };
        int rc = sh2_setSensorConfig(reports[n].id, &cfg);
        if (rc != SH2_OK) {
            ESP_LOGE(TAG, "enabling sensor 0x%02X failed (%d)", reports[n].id, rc);
            return rc;
        }
    }
    return SH2_OK;
}

// ---------------------------------------------------------------------------
// Open sh2 at the given SCL speed and prove the link with a product-ID round
// trip. Leaves sh2 open on success, closed on failure.
static bool open_at(uint8_t addr, uint32_t scl_hz)
{
    ESP_LOGI(TAG, "[4] opening sh2 at 0x%02X, %lu kHz ...", addr, (unsigned long)(scl_hz / 1000));
    sh2_Hal_t *hal = sh2_hal_i2c_create(PIN_SDA, PIN_SCL, PIN_IMU_INT, PIN_IMU_NRST,
                                        addr, scl_hz);
    int rc = sh2_open(hal, event_cb, NULL);
    if (rc != SH2_OK) {
        ESP_LOGW(TAG, "    sh2_open failed (%d)", rc);
        sh2_close();
        return false;
    }

    sh2_ProductIds_t ids;
    memset(&ids, 0, sizeof(ids));
    rc = sh2_getProdIds(&ids);
    if (rc != SH2_OK || ids.numEntries == 0) {
        ESP_LOGW(TAG, "    no product-ID response (%d)", rc);
        sh2_close();
        return false;
    }
    for (int n = 0; n < ids.numEntries; n++) {
        ESP_LOGI(TAG, "    part %lu  sw %u.%u.%u  build %lu",
                 (unsigned long)ids.entry[n].swPartNumber,
                 ids.entry[n].swVersionMajor, ids.entry[n].swVersionMinor,
                 ids.entry[n].swVersionPatch,
                 (unsigned long)ids.entry[n].swBuildNumber);
    }
    ESP_LOGI(TAG, "    last reset cause: %s", reset_cause_str(ids.entry[0].resetCause));
    return true;
}

static void stream_forever(uint32_t scl_hz)
{
    sh2_setSensorCallback(sensor_cb, NULL);
    s_saw_reset = false;
    if (enable_reports() != SH2_OK) return;

    ESP_LOGI(TAG, "=== streaming: %d Hz sensors, %d Hz output, I2C %lu kHz ===",
             REPORT_HZ, OUTPUT_HZ, (unsigned long)(scl_hz / 1000));
    ble_link_log("I streaming: %d Hz sensors, %d Hz output", REPORT_HZ, OUTPUT_HZ);
    printf("# t_ms,qw,qx,qy,qz,yaw_deg,pitch_deg,roll_deg,"
           "ax,ay,az[m/s2],gx,gy,gz[rad/s],mx,my,mz[uT],rv_status,mag_status,"
           "gqw,gqx,gqy,gqz,lx,ly,lz[m/s2],rch0..rch5,vsup[mV]\n");

    uint32_t last[CNT_N] = { 0 };
    int64_t last_stats_us = esp_timer_get_time();

    while (1) {
        sh2_service();                       // drains H_INTN, fires callbacks
        afe_poll();                          // one step of the resistance sweep
        if (s_saw_reset) {
            s_saw_reset = false;
            s_resets++;
            sh2_ProductIds_t ids;
            memset(&ids, 0, sizeof(ids));
            int rc = sh2_getProdIds(&ids);
            const char *cause = rc == SH2_OK && ids.numEntries
                ? reset_cause_str(ids.entry[0].resetCause) : "product ID request failed";
            ESP_LOGW(TAG, "BNO085 reset itself (#%lu, cause: %s) - re-enabling reports",
                     (unsigned long)s_resets, cause);
            ble_link_log("W BNO085 reset itself (#%lu, cause: %s) - re-enabling reports",
                         (unsigned long)s_resets, cause);
            enable_reports();
        }

        int64_t now = esp_timer_get_time();
        if (now - last_stats_us >= STATS_PERIOD_MS * 1000LL) {
            float sec = (now - last_stats_us) / 1e6f;
            char rates[160];
            snprintf(rates, sizeof(rates),
                     "# rate Hz: rv=%.1f acc=%.1f gyro=%.1f mag=%.1f | status rv=%u mag=%u (0..3)"
                     " | BNO resets %lu",
                     (s_count[CNT_RV]   - last[CNT_RV])   / sec,
                     (s_count[CNT_ACC]  - last[CNT_ACC])  / sec,
                     (s_count[CNT_GYRO] - last[CNT_GYRO]) / sec,
                     (s_count[CNT_MAG]  - last[CNT_MAG])  / sec,
                     s_imu.rv_status, s_imu.mag_status, (unsigned long)s_resets);
            printf("%s\n", rates);
            ble_link_log("%s", rates);
            memcpy(last, s_count, sizeof(last));
            last_stats_us = now;
        }
        vTaskDelay(1);                       // 1 ms with CONFIG_FREERTOS_HZ=1000
    }
}

bool imu_stream_run(uint8_t addr)
{
    for (size_t n = 0; n < sizeof(SCL_SPEEDS_HZ) / sizeof(SCL_SPEEDS_HZ[0]); n++) {
        if (open_at(addr, SCL_SPEEDS_HZ[n])) {
            // The ADS1115 shares the BNO085's bus; same task, so no locking.
            i2c_master_bus_handle_t bus = sh2_hal_i2c_bus();
            if (bus && i2c_master_probe(bus, ADS1115_ADDR, 50) == ESP_OK && afe_init(bus))
                ESP_LOGI(TAG, "    resistance front end streaming: RCH0-RCH%d + supply", AFE_CHANNELS - 1);
            stream_forever(SCL_SPEEDS_HZ[n]);
            sh2_close();                     // only reached if enabling reports failed
        }
    }
    return false;
}
