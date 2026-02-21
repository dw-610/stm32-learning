#include "dht20.h"

/* ==========================================================================
 * DHT20 Initialization
 *
 * DHT20 Datasheet Section 7.1: After power-on, wait 100ms for stabilization.
 * DHT20 Datasheet Section 7.4 step 1: Read status word by sending 0x71.
 *   If (status & 0x18) != 0x18, the sensor needs initialization of
 *   registers 0x1B, 0x1C, 0x1E (refer to manufacturer website for details).
 * ==========================================================================*/
void dht20_init(void)
{
    /* Wait 100ms after power-on for sensor stabilization
     * (DHT20 Datasheet Section 7.1) */
    delay_ms(100);

    /* Read status byte (DHT20 Datasheet Section 7.4 step 1)
     * Send command 0x71 and read back one status byte */
    i2c1_start();
    i2c1_send_addr(DHT20_ADDR, 0);  /* Write mode */
    i2c1_write(DHT20_CMD_STATUS);
    i2c1_stop();

    delay_ms(10);

    i2c1_start();
    i2c1_send_addr(DHT20_ADDR, 1);  /* Read mode */
    uint8_t status = i2c1_read_nack();

    /* Check calibration bits (DHT20 Datasheet Section 7.3, Table 9)
     * Bit[3] = CAL Enable: 1 = Calibrated
     * Status word & 0x18 should equal 0x18 for calibrated state */
    if ((status & 0x18) != 0x18) {
        /* Sensor not calibrated - in production, you would initialize
         * registers 0x1B, 0x1C, 0x1E per manufacturer instructions.
         * For most DHT20 modules this is already done at factory. */
    }
}

/* ==========================================================================
 * DHT20 Read Measurement
 *
 * Follows DHT20 Datasheet Section 7.4 "Sensor Reading Process":
 *   Step 2: Send 0xAC command with parameters 0x33, 0x00
 *   Step 3: Wait 80ms for measurement
 *   Step 4: Read status + 5 data bytes + CRC (7 bytes total)
 *   Step 5: Convert raw values
 *
 * Data format (DHT20 Datasheet Section 7.4 / Section 8):
 *   Byte 0: Status
 *   Bytes 1-2 + upper 4 bits of byte 3: 20-bit humidity raw (S_RH)
 *   Lower 4 bits of byte 3 + bytes 4-5: 20-bit temperature raw (S_T)
 *   Byte 6: CRC
 *
 * Signal conversion (DHT20 Datasheet Section 8):
 *   RH[%] = (S_RH / 2^20) * 100
 *   T[°C] = (S_T / 2^20) * 200 - 50
 * ==========================================================================*/
int dht20_read(int32_t *temp_x10, uint32_t *humi_x10)
{
    uint8_t data[7];

    /* Step 2: Send trigger measurement command (Datasheet Section 7.4 step 2)
     * Command 0xAC, followed by DATA0=0x33 and DATA1=0x00 */
    i2c1_start();
    i2c1_send_addr(DHT20_ADDR, 0);      /* Write mode */
    i2c1_write(DHT20_CMD_TRIGGER);       /* 0xAC */
    i2c1_write(DHT20_CMD_DATA0);         /* 0x33 */
    i2c1_write(DHT20_CMD_DATA1);         /* 0x00 */
    i2c1_wait_btf();
    i2c1_stop();

    /* Step 3: Wait 80ms for measurement to complete (Datasheet Section 7.4 step 3) */
    delay_ms(80);

    /* Step 4: Read status + data (Datasheet Section 7.4 step 3-4)
     * First read the status byte and check busy bit [7] */
    i2c1_start();
    i2c1_send_addr(DHT20_ADDR, 1);  /* Read mode */

    /* Read 7 bytes: status(1) + humidity(2.5) + temperature(2.5) + CRC(1)
     * ACK for first 6 bytes, NACK for the last byte */
    for (int i = 0; i < 6; i++) {
        data[i] = i2c1_read_ack();
    }
    data[6] = i2c1_read_nack();  /* Last byte with NACK + STOP */

    /* Check busy bit (Datasheet Section 7.3 Table 9: Bit[7] = busy indication)
     * 0 = idle (measurement complete), 1 = busy */
    if (data[0] & 0x80) {
        return -1;  /* Sensor still busy */
    }

    /* Step 5: Extract raw values (Datasheet Section 7.4, Section 8)
     *
     * Humidity raw (20 bits): data[1] << 12 | data[2] << 4 | data[3] >> 4
     * Temperature raw (20 bits): (data[3] & 0x0F) << 16 | data[4] << 8 | data[5]
     */
    uint32_t raw_humi = ((uint32_t)data[1] << 12)
                      | ((uint32_t)data[2] << 4)
                      | ((uint32_t)data[3] >> 4);

    uint32_t raw_temp = (((uint32_t)data[3] & 0x0F) << 16)
                      | ((uint32_t)data[4] << 8)
                      | ((uint32_t)data[5]);

    /* Convert to physical values in tenths for integer math
     *
     * Humidity (Datasheet Section 8.1):
     *   RH[%] = (S_RH / 2^20) * 100
     *   RH_x10 = S_RH * 1000 / 1048576
     *
     * Temperature (Datasheet Section 8.2):
     *   T[°C] = (S_T / 2^20) * 200 - 50
     *   T_x10 = (S_T * 2000 / 1048576) - 500
     */
    *humi_x10 = (raw_humi * 1000) / 1048576;
    *temp_x10 = (int32_t)((raw_temp * 2000) / 1048576) - 500;

    return 0;
}
