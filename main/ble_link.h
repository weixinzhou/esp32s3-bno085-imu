// BLE peripheral that streams IMU samples to the host dashboard.
//
// Service 7a4c0001-5b2e-4f6d-9c3a-2e1d8b6f0a51 ("IMU-BNO085"):
//   7a4c0002-...  DATA  notify  one ble_sample_t per notification
//   7a4c0003-...  LOG   notify  UTF-8 text: rate lines, sensor resets, errors
#pragma once
#include <stdbool.h>
#include <stdint.h>

// One sample, little-endian, fixed point in the BNO085's own Q formats so
// nothing is lost against the sensor's resolution. 60 bytes: needs an ATT
// MTU of at least 63, which every current central negotiates.
typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    int16_t q[4];          // rotation vector w, x, y, z           Q14
    int16_t acc[3];        // accelerometer, m/s^2                  Q8
    int16_t gyr[3];        // gyroscope, rad/s                      Q9
    int16_t mag[3];        // magnetometer, uT                      Q4
    uint8_t rv_status;     // 0..3
    uint8_t mag_status;    // 0..3
    int16_t gq[4];         // game rotation vector w, x, y, z      Q14
    int16_t lin[3];        // linear acceleration, m/s^2            Q8
    int16_t afe[7];        // ADS1115 counts (0.125 mV): RCH0-5, then supply
} ble_sample_t;

_Static_assert(sizeof(ble_sample_t) == 60, "BLE sample layout changed: update the host decoder");

void ble_link_start(void);
bool ble_link_connected(void);
void ble_link_send_sample(const ble_sample_t *s);
void ble_link_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
