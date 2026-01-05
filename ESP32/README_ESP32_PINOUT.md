# Pin references for ESP32-S3 DEV KIT-C

## Guide reference

[ESP32-S3 DevKitC pinout guide](https://randomnerdtutorials.com/esp32-s3-devkitc-pinout-guide/)

## SPI Flash and PSRAM

GPIOs 26 to 32 are connected to the integrated SPI flash and PSRAM and are not recommended for other uses. They are not exposed in this particular board, but if they are exposed on your board, avoid using them:

* GPIO 26 (Flash/PSRAM SPICS1)
* GPIO 27 (Flash/PSRAM SPIHD)
* GPIO 28 (Flash/PSRAM SPIWP)
* GPIO 29 (Flash/PSRAM SPICS0)
* GPIO 30 (Flash/PSRAM SPICLK)
* GPIO 31 (Flash/PSRAM SPIQ)
* GPIO 32 (Flash/PSRAM SPID)

## Capacitive touch GPIOs

The ESP32-S3 has 14 internal capacitive touch GPIOs. These can sense variations in anything that holds an electrical charge, like the human skin. So they can detect variations induced when touching the GPIOs with a finger. These pins can be easily integrated into capacitive pads and replace mechanical buttons. The capacitive touch pins can also be used to wake up the ESP32 from deep sleep.

Those internal touch sensors are connected to these GPIOs:

* T1 (GPIO 1)
* T2 (GPIO 2)
* T3 (GPIO 3)
* T4 (GPIO 4)
* T5 (GPIO 5)
* T6 (GPIO 6)
* T7 (GPIO 7)
* T8 (GPIO 8)
* T9 (GPIO 9)
* T10 (GPIO 10)
* T11 (GPIO 11)
* T12 (GPIO 12)
* T13 (GPIO 13)
* T14 (GPIO 14)

## Analog to Digital Converter (ADC)

The ESP32 has 20x 12-bit ADC input channels. These are the GPIOs that can be used as ADC and respective channels:

* ADC1_CH0 (GPIO 1)
* ADC1_CH1 (GPIO 2)
* ADC1_CH2 (GPIO 3)
* ADC1_CH3 (GPIO 4)
* ADC1_CH4 (GPIO 5)
* ADC1_CH5 (GPIO 6)
* ADC1_CH6 (GPIO 7)
* ADC1_CH7 (GPIO 8)
* ADC1_CH8 (GPIO 9)
* ADC1_CH9 (GPIO 10)
* ADC2_CH0 (GPIO 11)
* ADC2_CH1 (GPIO 12)
* ADC2_CH2 (GPIO 13)
* ADC2_CH3 (GPIO 14)
* ADC2_CH4 (GPIO 15)
* ADC2_CH5 (GPIO 16)
* ADC2_CH6 (GPIO 17)
* ADC2_CH7 (GPIO 18)
* ADC2_CH8 (GPIO 19)
* ADC2_CH9 (GPIO 20)

The ADC input channels have a 12-bit resolution. This means that you can get analog readings ranging from 0 to 4095, in which 0 corresponds to 0 V and 4095 to 3.3 V. You can also set the resolution of your channels on the code and the ADC range.

## RTC GPIOs

There is RTC GPIO support on the ESP32-S3. The GPIOs routed to the RTC low-power subsystem can be used when the ESP32 is in deep sleep. These RTC GPIOs can be used to wake up the ESP32 from deep sleep when the Ultra Low Power (ULP) coprocessor is running. The following GPIOs can be used as an external wake-up source.

* RTC_GPIO0  (GPIO0)
* RTC_GPIO1 (GPIO1)
* RTC_GPIO2 (GPIO2)
* RTC_GPIO3 (GPIO3)
* RTC_GPIO4 (GPIO4)
* RTC_GPIO5 (GPIO5)
* RTC_GPIO6 (GPIO6)
* RTC_GPIO7 (GPIO7)
* RTC_GPIO8 (GPIO8)
* RTC_GPIO9 (GPIO9)
* RTC_GPIO10 (GPIO10)
* RTC_GPIO11 (GPIO11)
* RTC_GPIO12 (GPIO12)
* RTC_GPIO13 (GPIO13)
* RTC_GPIO14 (GPIO14)
* RTC_GPIO15 (GPIO15)
* RTC_GPIO16 (GPIO16)
* RTC_GPIO17 (GPIO17)
* RTC_GPIO18 (GPIO18)
* RTC_GPIO19 (GPIO19)
* RTC_GPIO20 (GPIO20)
* RTC_GPIO21 (GPIO21)

## PWM

The ESP32-S3 has an LED PWM controller with 8 PWM channels that can be configured to generate PWM signals with different properties. All pins that can act as outputs can be used as PWM pins.

To set a PWM signal, you need to define these parameters in the code:

Signal’s frequency;
Duty cycle;
PWM channel (optional);
GPIO where you want to output the signal.

## I2C

When using the ESP32-S3 with the Arduino IDE, these are the ESP32 I2C default pins:

GPIO 8 (SDA)
GPIO 9 (SCL)

## SPI

The ESP32 integrates 4 SPI peripherals: SPI0, SPI1, SPI2 (commonly referred to as HSPI), and SPI3 (commonly referred to as VSPI).

SP0 and SP1 are used internally to communicate with the built-in flash memory, and you should not use them for other tasks.

You can use HSPI and VSPI to communicate with other devices. HSPI and VSPI have independent bus signals.

| SPI | MOSI | MISO | CLK | CS |
| --- | --- | --- | --- | --- |
| HSPI (SPI 2) | GPIO 11 | GPIO 13 | GPIO 12 | GPIO 10 |
| VSPI (SPI 3) | GPIO 35 | GPIO 37 | GPIO 36 | GPIO 39 |

## Interrupts

All GPIOs can be configured as interrupts.

## UART Pins – Serial Communication

The ESP32-S3 supports multiple UART (Universal Asynchronous Receiver-Transmitter) interfaces that allow serial communication with various devices. The ESP32 supports up to three UART interfaces: UART0, UART1, and UART2, depending on the ESP32 board model you’re using.

Like I2C and SPI, these UARTs can be mapped to any GPIO pin, although they have default pin assignments on most board models.

The following table shows the default UART0, UART1, and UART2 RX and TX pins for the ESP32-S3:

| UART Port | TX | RX | Remarks |
| --- | --- | --- | --- |
| UART0 | GPIO 43 | GPIO 44 | Cannot be changed |
| UART1 | GPIO 17 | GPIO 18 | Can be assigned to other GPIOs |
| UART2 | — | — | Assign any pins of your choice |
