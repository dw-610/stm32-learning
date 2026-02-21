#ifndef DHT20_H
#define DHT20_H

#include "board.h"

/* ==========================================================================
 * DHT20 sensor constants (DHT20 Datasheet Section 7)
 * ==========================================================================*/

/* 7-bit I2C address (Datasheet Section 7.3, Figure: device address = 0x38) */
#define DHT20_ADDR  0x38

/* Trigger measurement command (Datasheet Section 7.4 step 2) */
#define DHT20_CMD_TRIGGER  0xAC
#define DHT20_CMD_DATA0    0x33
#define DHT20_CMD_DATA1    0x00

/* Status check command (Datasheet Section 7.4 step 1) */
#define DHT20_CMD_STATUS   0x71

/* ==========================================================================
 * Function declarations
 * ==========================================================================*/

/* Initialize DHT20 sensor: check calibration status (Datasheet Section 7.4 step 1) */
void dht20_init(void);

/* Read temperature and humidity from DHT20.
 * temp_x10: temperature in tenths of degrees C (e.g., 234 = 23.4°C)
 * humi_x10: humidity in tenths of %RH (e.g., 567 = 56.7% RH)
 * Returns 0 on success, -1 on error. */
int dht20_read(int32_t *temp_x10, uint32_t *humi_x10);

#endif /* DHT20_H */
