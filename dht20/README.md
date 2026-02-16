# DHT20 Temperature & Humidity Sensor

Bare-metal STM32F411RE firmware that reads temperature and humidity from a DHT20 sensor over I2C using only direct register access.

## Hardware

**Board**: STM32F411RE Nucleo (or any STM32F411xE)
**Sensor**: ASAIR DHT20 (I2C, 3.3V)

### Wiring

| DHT20 Pin | Name | STM32 Pin |
|-----------|------|-----------|
| 1         | VDD  | 3.3V      |
| 2         | SDA  | PB7       |
| 3         | GND  | GND       |
| 4         | SCL  | PB6       |

PB6 and PB7 are configured as I2C1 via alternate function 4 (AF4). The DHT20 has internal pull-ups, so external resistors aren't required. A 100nF decoupling cap between VDD and GND close to the sensor is recommended.

## Project Structure

```
dht20/
├── Inc/
│   └── dht20.h          Register definitions and driver API
├── Src/
│   ├── main.c            Entry point — init, read loop, global variables
│   ├── dht20.c           I2C1 setup and DHT20 communication driver
│   ├── startup_stm32f411xx.S  Vector table and reset handler
│   ├── syscall.c         Newlib system call stubs
│   └── sysmem.c          Heap management (_sbrk)
├── docs/
│   └── dht20.pdf         DHT20 datasheet
├── CMakeLists.txt        Build configuration
├── stm32f411xe_flash.ld  Linker script (512K flash, 128K RAM)
└── cmake/                Toolchain and auto-generated CMake files
```

## How It Works

1. **Clock setup** — GPIOA, GPIOB, and I2C1 peripheral clocks are enabled via RCC
2. **GPIO config** — PA5 is set as push-pull output (LED heartbeat). PB6/PB7 are configured as open-drain alternate function for I2C1
3. **I2C1 init** — Standard mode (100 kHz) with clock control derived from the 16 MHz HSI oscillator
4. **Sensor init** — After a 100ms power-on delay, the driver reads the DHT20 status register to verify calibration
5. **Read loop** — Every ~2 seconds, the firmware sends a trigger command (0xAC), waits 80ms for conversion, then reads back 7 bytes containing status, 20-bit humidity, 20-bit temperature, and a CRC. Raw values are converted to Fahrenheit and %RH in tenths and stored in global variables (`g_temperature_f_x10`, `g_humidity_x10`, `g_read_status`) for inspection via GDB/SWD

All register accesses in the code are annotated with the corresponding section of either the DHT20 datasheet or the STM32F411 reference manual (RM0383).
