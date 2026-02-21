# DHT20 + LCD Temperature & Humidity Display

Bare-metal STM32F411RE firmware that reads temperature and humidity from a DHT20 sensor over I2C and displays the readings on a 16x2 HD44780 LCD via a PCF8574T I2C backpack. All using direct register access — no HAL, no CMSIS.

## Hardware

**Board**: STM32F411RE Nucleo (or any STM32F411xE)
**Sensor**: ASAIR DHT20 (I2C address 0x38)
**Display**: 16x2 HD44780 LCD with PCF8574T I2C backpack (I2C address 0x27)

Both peripherals share the I2C1 bus on PB6 (SCL) / PB7 (SDA).

### Wiring

| Device    | Pin  | STM32 Pin |
|-----------|------|-----------|
| DHT20     | VDD  | 3.3V      |
| DHT20     | SDA  | PB7       |
| DHT20     | GND  | GND       |
| DHT20     | SCL  | PB6       |
| LCD (I2C) | VCC  | 5V        |
| LCD (I2C) | SDA  | PB7       |
| LCD (I2C) | GND  | GND       |
| LCD (I2C) | SCL  | PB6       |

PB6 and PB7 are configured as I2C1 via alternate function 4 (AF4). The DHT20 has internal pull-ups, so external resistors aren't required. A 100nF decoupling cap between VDD and GND close to the sensor is recommended.

## Project Structure

```
dht20_lcd/
├── Inc/
│   ├── board.h           Register definitions, delay, and I2C primitives
│   ├── dht20.h           DHT20 sensor constants and API
│   └── lcd.h             HD44780 LCD constants and API
├── Src/
│   ├── main.c            Entry point — init, read loop, LCD display
│   ├── board.c           Delay functions, I2C1 init/scan/transfer primitives
│   ├── dht20.c           DHT20 sensor init and read
│   ├── lcd.c             HD44780 LCD driver (4-bit mode via PCF8574T)
│   ├── startup_stm32f411xx.S  Vector table and reset handler
│   ├── syscall.c         Newlib system call stubs
│   └── sysmem.c          Heap management (_sbrk)
├── docs/
│   └── dht20.pdf         DHT20 datasheet
├── CMakeLists.txt        Build configuration
├── stm32f411xe_flash.ld  Linker script (512K flash, 128K RAM)
└── cmake/                Toolchain and auto-generated CMake files
```

The code is layered so that `board.h`/`board.c` provide the platform primitives (registers, delays, I2C), while `dht20` and `lcd` are independent device drivers that both depend on the board layer.

## How It Works

1. **Clock setup** — GPIOA, GPIOB, and I2C1 peripheral clocks are enabled via RCC
2. **GPIO config** — PA5 is set as push-pull output (LED heartbeat). PB6/PB7 are configured as open-drain alternate function for I2C1
3. **I2C1 init** — Standard mode (100 kHz) with clock control derived from the 16 MHz HSI oscillator
4. **Bus scan** — Probes all 7-bit I2C addresses to discover connected devices (expects DHT20 at 0x38, LCD at 0x27)
5. **Sensor init** — After a 100ms power-on delay, the driver reads the DHT20 status register to verify calibration
6. **LCD init** — HD44780 initialization-by-instruction sequence in 4-bit mode through the PCF8574T I2C expander
7. **Read loop** — Every ~2 seconds, the firmware sends a trigger command (0xAC), waits 80ms for conversion, then reads back 7 bytes containing status, 20-bit humidity, 20-bit temperature, and a CRC. Values are converted to Fahrenheit and %RH in tenths, displayed on the LCD (row 0: temperature, row 1: humidity), and stored in globals for GDB inspection

All register accesses in the code are annotated with the corresponding section of either the DHT20 datasheet or the STM32F411 reference manual (RM0383).
