#ifndef LCD_H
#define LCD_H

#include "board.h"

/* PCF8574T I2C address (7-bit) — confirmed on bus scan as 0x27 */
#define LCD_I2C_ADDR  0x27

/* PCF8574T pin mapping to HD44780 LCD (active-high):
 * P0 = RS, P1 = RW, P2 = EN, P3 = Backlight, P4-P7 = D4-D7 */
#define LCD_RS  (1 << 0)
#define LCD_RW  (1 << 1)
#define LCD_EN  (1 << 2)
#define LCD_BL  (1 << 3)

/* Initialize HD44780 LCD in 4-bit mode via PCF8574T I2C backpack */
void lcd_init(void);

/* Print a null-terminated string to the LCD at the current cursor position */
void lcd_print(const char *str);

/* Set cursor position: row 0-1, col 0-15 */
void lcd_set_cursor(uint8_t row, uint8_t col);

/* Clear the display */
void lcd_clear(void);

#endif /* LCD_H */
