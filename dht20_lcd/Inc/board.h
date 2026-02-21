#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* ==========================================================================
 * Base addresses (RM0383 Section 2.3 Memory map, Table 1)
 * ==========================================================================*/
#define RCC_BASE    0x40023800UL  /* Reset and Clock Control */
#define GPIOA_BASE  0x40020000UL  /* GPIO Port A */
#define GPIOB_BASE  0x40020400UL  /* GPIO Port B */
#define I2C1_BASE   0x40005400UL  /* I2C1 peripheral */

/* ==========================================================================
 * RCC registers (RM0383 Section 6.3)
 * ==========================================================================*/

/* AHB1 peripheral clock enable (RM0383 Section 6.3.10, offset 0x30)
 * Bit 0: GPIOAEN, Bit 1: GPIOBEN */
#define RCC_AHB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x30))

/* APB1 peripheral clock enable (RM0383 Section 6.3.11, offset 0x40)
 * Bit 21: I2C1EN */
#define RCC_APB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x40))

/* ==========================================================================
 * GPIOA registers (RM0383 Section 8.4)
 * ==========================================================================*/
#define GPIOA_MODER  (*(volatile uint32_t *)(GPIOA_BASE + 0x00))  /* Mode register */
#define GPIOA_ODR    (*(volatile uint32_t *)(GPIOA_BASE + 0x14))  /* Output data register */

/* ==========================================================================
 * GPIOB registers (RM0383 Section 8.4)
 * Used for I2C1: PB6 = SCL, PB7 = SDA
 * ==========================================================================*/
#define GPIOB_MODER   (*(volatile uint32_t *)(GPIOB_BASE + 0x00))  /* Mode (8.4.1) */
#define GPIOB_OTYPER  (*(volatile uint32_t *)(GPIOB_BASE + 0x04))  /* Output type (8.4.2) */
#define GPIOB_OSPEEDR (*(volatile uint32_t *)(GPIOB_BASE + 0x08))  /* Output speed (8.4.3) */
#define GPIOB_PUPDR   (*(volatile uint32_t *)(GPIOB_BASE + 0x0C))  /* Pull-up/pull-down (8.4.4) */
#define GPIOB_AFRL    (*(volatile uint32_t *)(GPIOB_BASE + 0x20))  /* Alternate function low (8.4.9) */

/* ==========================================================================
 * I2C1 registers (RM0383 Section 18.6)
 * ==========================================================================*/
#define I2C1_CR1   (*(volatile uint32_t *)(I2C1_BASE + 0x00))  /* Control register 1 (18.6.1) */
#define I2C1_CR2   (*(volatile uint32_t *)(I2C1_BASE + 0x04))  /* Control register 2 (18.6.2) */
#define I2C1_OAR1  (*(volatile uint32_t *)(I2C1_BASE + 0x08))  /* Own address 1 (18.6.3) */
#define I2C1_DR    (*(volatile uint32_t *)(I2C1_BASE + 0x10))  /* Data register (18.6.5) */
#define I2C1_SR1   (*(volatile uint32_t *)(I2C1_BASE + 0x14))  /* Status register 1 (18.6.6) */
#define I2C1_SR2   (*(volatile uint32_t *)(I2C1_BASE + 0x18))  /* Status register 2 (18.6.7) */
#define I2C1_CCR   (*(volatile uint32_t *)(I2C1_BASE + 0x1C))  /* Clock control (18.6.8) */
#define I2C1_TRISE (*(volatile uint32_t *)(I2C1_BASE + 0x20))  /* Rise time (18.6.9) */

/* ==========================================================================
 * Function declarations
 * ==========================================================================*/

/* Simple software delay */
void delay(uint32_t count);

/* Approximate millisecond delay assuming 16 MHz HSI */
void delay_ms(uint32_t ms);

/* Initialize I2C1 peripheral on PB6 (SCL) / PB7 (SDA) at 100 kHz */
void i2c1_init(void);

/* Scan I2C1 bus for all responding devices.
 * found[]: filled with 7-bit addresses that ACK'd (up to max_found entries)
 * Returns the number of devices found. */
int i2c1_scan(uint8_t *found, int max_found);

/* I2C1 low-level transfer primitives */
void i2c1_start(void);
void i2c1_send_addr(uint8_t addr, uint8_t read);
void i2c1_write(uint8_t data);
void i2c1_stop(void);
uint8_t i2c1_read_ack(void);
uint8_t i2c1_read_nack(void);
void i2c1_wait_btf(void);

#endif /* BOARD_H */
