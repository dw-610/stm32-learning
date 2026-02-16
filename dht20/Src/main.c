/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : DHT20 temperature/humidity sensor reader
 ******************************************************************************
 * Reads temperature and humidity from a DHT20 sensor over I2C1 (PB6/PB7)
 * and stores results in global variables for inspection via GDB/SWD.
 * PA5 LED blinks as a heartbeat indicator.
 *
 * Hardware connections:
 *   DHT20 VDD -> 3.3V
 *   DHT20 GND -> GND
 *   DHT20 SCL -> PB6 (I2C1_SCL, AF4)
 *   DHT20 SDA -> PB7 (I2C1_SDA, AF4)
 *   (DHT20 Datasheet Section 5, Table 5: Pin 1=VDD, 2=SDA, 3=GND, 4=SCL)
 *
 * The DHT20 has internal pull-ups on SDA/SCL
 * (DHT20 Datasheet Section 5.3 / Figure 8 notes).
 ******************************************************************************
 */

#include "dht20.h"

/* Global sensor readings — inspect these with GDB:
 *   (gdb) print g_temperature_f_x10
 *   (gdb) print g_humidity_x10
 * Values are in tenths: 734 = 73.4°F, 567 = 56.7% RH */
volatile int32_t  g_temperature_f_x10 = 0;
volatile uint32_t g_humidity_x10 = 0;
volatile int32_t  g_read_status = -1;  /* 0 = last read OK, -1 = error */

int main(void)
{
    /* Enable GPIOA clock for LED on PA5 (RM0383 Section 6.3.10: bit 0) */
    RCC_AHB1ENR |= (1 << 0);

    /* Configure PA5 as general-purpose output (RM0383 Section 8.4.1)
     * MODER bits [11:10] = 01 (output mode) */
    GPIOA_MODER &= ~(3 << 10);
    GPIOA_MODER |= (1 << 10);

    /* Initialize I2C1 peripheral (PB6=SCL, PB7=SDA, 100 kHz standard mode) */
    i2c1_init();

    /* Initialize DHT20 sensor — checks calibration status
     * (DHT20 Datasheet Section 7.4 step 1) */
    dht20_init();

    /* Main loop: read sensor every ~2 seconds
     * (DHT20 Datasheet Section 4.4: recommended measurement interval >= 2s
     *  to keep sensor temperature rise < 0.1°C) */
    for (;;) {
        int32_t  temp;
        uint32_t humi;

        g_read_status = dht20_read(&temp, &humi);

        if (g_read_status == 0) {
            /* Convert C to F in tenths: F_x10 = C_x10 * 9 / 5 + 320 */
            g_temperature_f_x10 = temp * 9 / 5 + 320;
            g_humidity_x10 = humi;
        }

        /* Toggle PA5 LED as heartbeat (RM0383 Section 8.4.6: ODR bit 5) */
        GPIOA_ODR ^= (1 << 5);

        /* Delay ~2 seconds (DHT20 Datasheet Section 4.4) */
        delay(8000000);
    }
}
