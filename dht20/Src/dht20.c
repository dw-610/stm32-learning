#include "dht20.h"

/* ==========================================================================
 * Software delay
 * ==========================================================================*/
void delay(uint32_t count)
{
    while (count--) {
        __asm__("nop");
    }
}

/* Approximate millisecond delay assuming 16 MHz HSI (default clock).
 * Each loop iteration of delay() is roughly 4 cycles (nop + branch + decrement),
 * so ~4000 iterations per ms at 16 MHz. */
static void delay_ms(uint32_t ms)
{
    delay(ms * 4000);
}

/* ==========================================================================
 * I2C1 low-level helpers
 *
 * These follow the transfer sequence diagrams in RM0383 Section 18.3.3:
 *   - Figure 164: Controller transmitter sequence (EV5, EV6, EV8, EV8_2)
 *   - Figure 165: Controller receiver sequence (EV5, EV6, EV7, EV7_1)
 * ==========================================================================*/

/* Generate START condition (RM0383 Section 18.3.3 "Start condition")
 * Sets START bit in CR1, waits for SB flag in SR1 (EV5). */
static void i2c1_start(void)
{
    I2C1_CR1 |= (1 << 8);   /* CR1 bit 8: START generation (18.6.1) */
    while (!(I2C1_SR1 & (1 << 0)))  /* SR1 bit 0: SB - Start bit (18.6.6) */
        ;
}

/* Send 7-bit address + R/W bit (RM0383 Section 18.3.3 "Target address transmission")
 * addr: 7-bit I2C address
 * read: 0 = write, 1 = read
 * Waits for ADDR flag (EV6), then clears it by reading SR1 + SR2 (18.6.7 note). */
static void i2c1_send_addr(uint8_t addr, uint8_t read)
{
    I2C1_DR = (uint32_t)((addr << 1) | (read & 1));  /* DR (18.6.5) */
    while (!(I2C1_SR1 & (1 << 1)))  /* SR1 bit 1: ADDR (18.6.6) */
        ;
    /* Clear ADDR by reading SR1 then SR2 (RM0383 Section 18.6.7 note) */
    (void)I2C1_SR1;
    (void)I2C1_SR2;
}

/* Write one byte (RM0383 Section 18.3.3 "Controller transmitter", EV8)
 * Waits for TxE (SR1 bit 7) before writing to DR. */
static void i2c1_write(uint8_t data)
{
    while (!(I2C1_SR1 & (1 << 7)))  /* SR1 bit 7: TxE (18.6.6) */
        ;
    I2C1_DR = data;
}

/* Read one byte with ACK (RM0383 Section 18.3.3 "Controller receiver", EV7)
 * ACK bit set so the controller acknowledges after receiving. */
static uint8_t i2c1_read_ack(void)
{
    I2C1_CR1 |= (1 << 10);  /* CR1 bit 10: ACK enable (18.6.1) */
    while (!(I2C1_SR1 & (1 << 6)))  /* SR1 bit 6: RxNE (18.6.6) */
        ;
    return (uint8_t)I2C1_DR;
}

/* Read one byte with NACK (RM0383 Section 18.3.3 "Closing the communication")
 * Clear ACK, set STOP before reading last byte. */
static uint8_t i2c1_read_nack(void)
{
    I2C1_CR1 &= ~(1 << 10);  /* Clear ACK (18.6.1) */
    I2C1_CR1 |= (1 << 9);    /* CR1 bit 9: STOP generation (18.6.1) */
    while (!(I2C1_SR1 & (1 << 6)))  /* SR1 bit 6: RxNE (18.6.6) */
        ;
    return (uint8_t)I2C1_DR;
}

/* Generate STOP condition (RM0383 Section 18.3.3 "Closing the communication", EV8_2) */
static void i2c1_stop(void)
{
    I2C1_CR1 |= (1 << 9);  /* CR1 bit 9: STOP (18.6.1) */
}

/* Wait for BTF (Byte Transfer Finished) flag.
 * RM0383 Section 18.6.6: SR1 bit 2 = BTF */
static void i2c1_wait_btf(void)
{
    while (!(I2C1_SR1 & (1 << 2)))
        ;
}

/* ==========================================================================
 * I2C1 Initialization
 *
 * GPIO configuration: RM0383 Section 8.3, Table 24 (AF open-drain for I2C)
 * I2C clock setup: RM0383 Section 18.3.3 (required sequence for controller mode)
 * Pin mapping: RM0383 Section 7.3, Table 9 (AF4 for I2C1 on PB6/PB7)
 * ==========================================================================*/
void i2c1_init(void)
{
    /* --- Enable clocks --- */
    /* GPIOB clock: AHB1ENR bit 1 (RM0383 Section 6.3.10) */
    RCC_AHB1ENR |= (1 << 1);

    /* I2C1 clock: APB1ENR bit 21 (RM0383 Section 6.3.11) */
    RCC_APB1ENR |= (1 << 21);

    /* --- Configure PB6 (SCL) and PB7 (SDA) for I2C1 ---
     * RM0383 Section 8.3 Table 24: I2C pins must be configured as
     * alternate function, open-drain output.
     * RM0383 Section 7.3 Table 9: AF4 = I2C1..3 for PB6/PB7 */

    /* MODER: alternate function mode = 10 (RM0383 Section 8.4.1)
     * PB6 = bits [13:12], PB7 = bits [15:14] */
    GPIOB_MODER &= ~((3 << 12) | (3 << 14));  /* Clear */
    GPIOB_MODER |= (2 << 12) | (2 << 14);     /* Set to AF (10) */

    /* OTYPER: open-drain = 1 (RM0383 Section 8.4.2)
     * PB6 = bit 6, PB7 = bit 7 */
    GPIOB_OTYPER |= (1 << 6) | (1 << 7);

    /* OSPEEDR: high speed = 10 (RM0383 Section 8.4.3)
     * PB6 = bits [13:12], PB7 = bits [15:14] */
    GPIOB_OSPEEDR &= ~((3 << 12) | (3 << 14));
    GPIOB_OSPEEDR |= (2 << 12) | (2 << 14);

    /* PUPDR: pull-up = 01 (RM0383 Section 8.4.4)
     * PB6 = bits [13:12], PB7 = bits [15:14] */
    GPIOB_PUPDR &= ~((3 << 12) | (3 << 14));
    GPIOB_PUPDR |= (1 << 12) | (1 << 14);

    /* AFRL: select AF4 for PB6 and PB7 (RM0383 Section 8.4.9)
     * AFRL covers pins 0-7. Each pin gets 4 bits.
     * PB6 = bits [27:24], PB7 = bits [31:28]
     * AF4 = 0x4 (RM0383 Section 7.3 Table 9) */
    GPIOB_AFRL &= ~((0xF << 24) | (0xF << 28));
    GPIOB_AFRL |= (4 << 24) | (4 << 28);

    /* --- Configure I2C1 peripheral ---
     * RM0383 Section 18.3.3: Required sequence for controller mode:
     *   1. Program peripheral clock in CR2
     *   2. Configure clock control register (CCR)
     *   3. Configure rise time register (TRISE)
     *   4. Enable peripheral (PE in CR1) */

    /* Disable I2C1 before configuration (RM0383 Section 18.6.1: PE bit 0) */
    I2C1_CR1 &= ~(1 << 0);

    /* Software reset to clear any stuck state (RM0383 Section 18.6.1: SWRST bit 15) */
    I2C1_CR1 |= (1 << 15);
    I2C1_CR1 &= ~(1 << 15);

    /* CR2: Set peripheral clock frequency = 16 MHz (RM0383 Section 18.6.2)
     * FREQ[5:0] bits = 16 (APB1 clock = 16 MHz from HSI) */
    I2C1_CR2 = 16;

    /* CCR: Standard mode 100 kHz (RM0383 Section 18.6.8)
     * Sm mode: Thigh = CCR * TPCLK1, Tlow = CCR * TPCLK1
     * SCL period = 2 * CCR * TPCLK1
     * For 100 kHz: CCR = FPCLK1 / (2 * FSCL) = 16000000 / (2 * 100000) = 80
     * F/S bit 15 = 0 (Sm mode) */
    I2C1_CCR = 80;

    /* TRISE: Maximum rise time (RM0383 Section 18.6.9)
     * Sm mode max SCL rise time = 1000 ns (I2C spec)
     * TRISE = (1000ns / TPCLK1) + 1 = (1000ns / 62.5ns) + 1 = 16 + 1 = 17 */
    I2C1_TRISE = 17;

    /* Enable I2C1 (RM0383 Section 18.6.1: PE bit 0) */
    I2C1_CR1 |= (1 << 0);
}

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
