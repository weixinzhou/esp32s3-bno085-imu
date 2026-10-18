// Pin map for the v7 carrier
// (Netlist_ESP32-S3_XIAO_ADS1115_Resistance_Detector_7_2026-10-13.tel).
//
// XIAO ESP32-S3 silkscreen -> GPIO. J1.N carries D(N-1).
//
// Differences from the earlier carrier that matter to firmware:
//   - BOOTN (U1.4) is no longer wired to the XIAO. R4 (10k) holds it high, so
//     the part always boots its application. D2/GPIO3 is unused.
//   - NRST (U1.11) has its own 10k pull-up (R3), so the BNO085 runs even
//     before the ESP32 drives GPIO2.
//   - 32.768 kHz crystal X1 on XIN32/XOUT32 with CLKSEL0 (U1.10) on GND.
//   - Host I2C pull-ups R1/R2 are 10k.
#pragma once
#include "driver/gpio.h"

#define PIN_IMU_INT   GPIO_NUM_1   // D0, J1.1 -> U1.14 H_INTN (driven by the BNO085)
#define PIN_IMU_NRST  GPIO_NUM_2   // D1, J1.2 -> U1.11 NRST   (active low, R3 pull-up)
#define PIN_SDA       GPIO_NUM_5   // D4, J1.5 -> U1.20 H_SDA  (R1 pull-up)
#define PIN_SCL       GPIO_NUM_6   // D5, J1.6 -> U1.19 H_SCL  (R2 pull-up)

// D3, J1.4: not connected on v7. Carries a 32.768 kHz test clock that can be
// fly-wired to XIN32 (C5's non-ground pad) through a 10k resistor.
#define PIN_TEST_CLK  GPIO_NUM_4

// Resistance front end on the second half of the board (no netlist; read
// off the PCB layout): CD74HC4067 select lines on the XIAO's right header,
// common pin MEAS_NODE on ADS1115 AIN0, pulled to 3V3 through a reference
// resistor. FPC RCH0-RCH5 land on mux inputs I0-I5; I6-I15 are unrouted.
#define PIN_MUX_S0    GPIO_NUM_8    // D9,  J2.3
#define PIN_MUX_S1    GPIO_NUM_9    // D10, J2.4
#define PIN_MUX_S2    GPIO_NUM_44   // D7,  J2.1 (UART0 RX; free on a XIAO)
#define PIN_MUX_S3    GPIO_NUM_7    // D8,  J2.2
#define ADS1115_ADDR  0x48          // ADDR to GND
#define AFE_R_REF_OHM 10000.0f      // assumed: 10k as on the first board revision
#define AFE_CHANNELS  6             // RCH0-RCH5 on the FPC
#define AFE_OPEN_CH   15            // unrouted input: reads the pull-up, i.e. the supply

#define BNO_ADDR_SA0_LOW   0x4A    // SA0 (U1.17) is on GND, so expect this one
#define BNO_ADDR_SA0_HIGH  0x4B
#define BNO_ADDR_BOOT_LOW  0x28    // bootloader addresses; would mean BOOTN read low
#define BNO_ADDR_BOOT_HIGH 0x29
