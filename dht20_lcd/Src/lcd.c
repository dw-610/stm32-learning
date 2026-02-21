#include "lcd.h"

/* Write one byte to the PCF8574T I/O expander over I2C1 */
static void pcf8574_write(uint8_t data)
{
    i2c1_start();
    i2c1_send_addr(LCD_I2C_ADDR, 0);
    i2c1_write(data);
    i2c1_stop();
}

/* Send a 4-bit nibble to the LCD with an EN pulse.
 * nibble: data in bits 4-7 (already shifted to upper nibble position)
 * rs: LCD_RS for data, 0 for command */
static void lcd_send_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t byte = nibble | rs | LCD_BL;

    pcf8574_write(byte | LCD_EN);   /* EN high */
    delay(400);                      /* ~25us, well above 450ns minimum */
    pcf8574_write(byte);            /* EN low — latch data */
    delay(400);
}

/* Send a full byte to the LCD in two 4-bit transfers (high nibble first).
 * rs: LCD_RS for data, 0 for command */
static void lcd_send_byte(uint8_t byte, uint8_t rs)
{
    lcd_send_nibble(byte & 0xF0, rs);         /* High nibble */
    lcd_send_nibble((byte << 4) & 0xF0, rs);  /* Low nibble */
}

/* Send a command byte to the LCD (RS = 0) */
static void lcd_command(uint8_t cmd)
{
    lcd_send_byte(cmd, 0);
    delay_ms(2);  /* Most commands need 1.52ms */
}

/* Send a data byte (character) to the LCD (RS = 1) */
static void lcd_data(uint8_t ch)
{
    lcd_send_byte(ch, LCD_RS);
    delay(400);  /* ~25us settle time */
}

/* Initialize the HD44780 LCD in 4-bit mode.
 * Follows the HD44780 datasheet initialization-by-instruction sequence
 * (Figure 24: 4-bit interface initialization). */
void lcd_init(void)
{
    /* Wait > 40ms after power-on (HD44780 datasheet) */
    delay_ms(50);

    /* Put LCD into known state — send 0x30 three times in 8-bit mode
     * (HD44780 Figure 24: initialization by instruction) */
    lcd_send_nibble(0x30, 0);
    delay_ms(5);              /* Wait > 4.1ms */

    lcd_send_nibble(0x30, 0);
    delay_ms(1);              /* Wait > 100us */

    lcd_send_nibble(0x30, 0);
    delay_ms(1);

    /* Switch to 4-bit mode */
    lcd_send_nibble(0x20, 0);
    delay_ms(1);

    /* Now in 4-bit mode — can use lcd_command() for full-byte commands */

    /* Function set: 4-bit mode, 2 lines, 5x8 font (0x28) */
    lcd_command(0x28);

    /* Display on, cursor off, blink off (0x0C) */
    lcd_command(0x0C);

    /* Clear display (0x01) — needs extra time */
    lcd_command(0x01);
    delay_ms(2);

    /* Entry mode: increment cursor, no display shift (0x06) */
    lcd_command(0x06);
}

/* Print a null-terminated string at the current cursor position */
void lcd_print(const char *str)
{
    while (*str) {
        lcd_data((uint8_t)*str++);
    }
}

/* Set cursor to row (0-1) and column (0-15).
 * HD44780 DDRAM addresses: row 0 starts at 0x00, row 1 at 0x40.
 * Set DDRAM address command = 0x80 | address. */
void lcd_set_cursor(uint8_t row, uint8_t col)
{
    uint8_t addr = col + (row == 0 ? 0x00 : 0x40);
    lcd_command(0x80 | addr);
}

/* Clear display and return cursor home */
void lcd_clear(void)
{
    lcd_command(0x01);
    delay_ms(2);
}
