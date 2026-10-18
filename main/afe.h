// Resistance front end: CD74HC4067 16:1 mux into ADS1115 AIN0.
#pragma once
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "board.h"

// Attach the ADS1115 to an existing bus and set up the mux select lines.
bool afe_init(i2c_master_bus_handle_t bus);
void afe_deinit(void);

// Select mux input ch (0-15), convert, and return the MEAS_NODE voltage.
bool afe_read_mv(int ch, float *mv);

// Read every mux input once and print a table of voltages and resistances.
void afe_probe(void);

// Streaming: RCH0..RCH5 plus the open input (the supply), in that order.
#define AFE_SLOTS (AFE_CHANNELS + 1)

// Call often from the sensor loop. Never waits for a conversion: each call
// either starts one on the next channel or collects the one in flight.
void afe_poll(void);

// Latest millivolts per slot; false when no front end is attached.
bool afe_latest(float mv[AFE_SLOTS]);
