# Hardware

Carrier boards for a Seeed XIAO ESP32-S3, drawn in EasyEDA Pro (立创 EDA 专业版).
Board contents below were read from each package's `FlyingProbeTesting.json`
(parts and per-pin net names) and from the netlists.

## Gerber

| File | Board | Size | What is on it |
|---|---|---|---|
| `gerber/Gerber_PCB1_9_2026-09-25.zip` | IMU carrier | 62.7 × 27.9 mm | BNO085 (U1), 32.768 kHz crystal (X1), I2C / NRST / BOOTN pull-ups, XIAO headers J1/J2 |
| `gerber/Gerber_PCB1_10_2026-09-25all.zip` | IMU + resistance front end | 83.6 × 27.9 mm | the IMU half above, plus ADS1115 (U2), an **8:1 mux, 16-pin** (U3, 74HC4051 footprint) and a 14-pin FPC (RCH0–RCH5) |

**The board currently on the bench is newer than both packages.** Its resistance
half uses a **CD74HC4067 (16:1, 24-pin)** with select lines S0–S3 and was not
exported as Gerber; the firmware's pin map for it (`main/board.h`) was read off
the layout by hand. Export that revision and add it here.

Differences between `PCB1_10` and the bench board's resistance half:

| | PCB1_10 (in this folder) | Bench board |
|---|---|---|
| Mux | 8:1, S0–S2 | CD74HC4067 16:1, S0–S3 |
| J2.1 / J2.2 / J2.3 / J2.4 | MUX_S0 / MUX_S1 / MUX_S2 / ADC_ALERT | S2 / S3 / S0 / S1 |
| Divider node | `ADC_AIN0`, pulled up by R5 | `MEAS_NODE` on AIN0 |

`PCB1_10` per-pin nets, for reference:

```
J1   1 IMU_INT  2 IMU_RST  5 I2C_SDA  6 I2C_SCL
J2   1 MUX_S0   2 MUX_S1   3 MUX_S2   4 ADC_ALERT  5 3V3  6 GND
U2   ADS1115: 1 ADDR=GND (0x48)  2 ALERT  4 AIN0  8 VDD  9 SDA  10 SCL
U3   mux:     3 COM=AIN0  6 EN=GND  9 S2  10 S1  11 S0
              13 RCH2  14 RCH1  15 RCH0  12 RCH3  5 RCH5  4 RCH4
FPC1 12 RCH0  10 RCH1  8 RCH2  6 RCH3  4 RCH4  2 RCH5, GND on the odd pins
```

## Netlists

EasyEDA netlist exports (`.tel`) of the schematic, oldest first.

| File | Contents |
|---|---|
| `Netlist_..._4_2026-08-01.tel` | ADS1115 front end only |
| `Netlist_..._1_2026-09-10.tel`, `..._1_2026-09-18.tel` | first combined board: BNO085 + ADS1115 + CD74HC4067, no crystal |
| `Netlist_..._7_2026-10-13.tel` | IMU carrier v7: BNO085 with 32.768 kHz crystal, PS0/PS1/SA0 to GND, BOOTN pulled up |

## Bring-up notes

- BNO085 on I2C at 0x4A (SA0 to GND), ADS1115 at 0x48 (ADDR to GND).
- CEVA recommends pull-ups on ENV_SDA / ENV_SCL (U1 pins 15/16) even when no
  environmental sensors are fitted; v7 leaves them open. The part still works.
- The LGA-28 is hard to hand-solder: on the first v7 build the pin-1 corner
  (power, PS straps, crystal) did not wet. `main/main.c` prints pin-level
  diagnostics that find this kind of fault.
