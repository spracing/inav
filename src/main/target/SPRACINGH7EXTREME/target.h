/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#define TARGET_BOARD_IDENTIFIER "SP7E"
#define USBD_PRODUCT_STRING "SPRacingH7EXTREME"

#define USE_TARGET_CONFIG

#undef USE_PWM_OUTPUT // disabled until H7 timer support is added.
#undef USE_RX_PPM     // disabled until H7 timer support is added.
#undef USE_TIMER      // disabled until H7 timer support is added.

#define LED0_PIN                PE3

//#define USE_BEEPER
#define BEEPER_PIN              PD7
#define BEEPER_INVERTED

/* Button support not ported yet.
// Force two buttons to look at the single button so config reset on button works
#define USE_BUTTONS
#define BUTTON_A_PIN            PE4
#define BUTTON_A_PIN_INVERTED // Active high
#define BUTTON_B_PIN            PE4
#define BUTTON_B_PIN_INVERTED // Active high
*/

/* QuadSPI support not ported yet.
#define USE_QUADSPI
#define USE_QUADSPI_DEVICE_1

#define QUADSPI1_SCK_PIN PB2

#define QUADSPI1_BK1_IO0_PIN PD11
#define QUADSPI1_BK1_IO1_PIN PD12
#define QUADSPI1_BK1_IO2_PIN PE2
#define QUADSPI1_BK1_IO3_PIN PD13
#define QUADSPI1_BK1_CS_PIN PB10

#define QUADSPI1_BK2_IO0_PIN PE7
#define QUADSPI1_BK2_IO1_PIN PE8
#define QUADSPI1_BK2_IO2_PIN PE9
#define QUADSPI1_BK2_IO3_PIN PE10
#define QUADSPI1_BK2_CS_PIN NONE

#define QUADSPI1_MODE QUADSPI_MODE_BK1_ONLY
#define QUADSPI1_CS_FLAGS (QUADSPI_BK1_CS_HARDWARE | QUADSPI_BK2_CS_NONE | QUADSPI_CS_MODE_LINKED)
*/

//#define USE_FLASH_CHIP // BF define

/* Config in External Flash not ported yet.
#define CONFIG_IN_EXTERNAL_FLASH
//#define CONFIG_IN_SDCARD
//#define CONFIG_IN_RAM
#if !defined(CONFIG_IN_RAM) && !defined(CONFIG_IN_SDCARD) && !defined(CONFIG_IN_EXTERNAL_FLASH)
#error "EEPROM storage location not defined"
#endif
*/ 

#define CONFIG_IN_RAM // temporarily use RAM for config until CONFIG_IN_EXTERNAL_FLASH  is ported.

//#define USE_UART // BF define

#define USE_UART1
#define UART1_RX_PIN            PB15
#define UART1_TX_PIN            PB14

#define USE_UART2
#define UART2_RX_PIN            NONE
#define UART2_TX_PIN            PD5   // TX pin is bidirectional.

#define USE_UART3
#define UART3_RX_PIN            PD9
#define UART3_TX_PIN            PD8

#define USE_UART4
#define UART4_RX_PIN            PD0
#define UART4_TX_PIN            PD1

#define USE_UART5
#define UART5_RX_PIN            PB5
#define UART5_TX_PIN            PB13

#define USE_UART6
#define UART6_RX_PIN            PC7 // aka M7
#define UART6_TX_PIN            PC6 // aka M8

#define USE_UART8
#define UART8_RX_PIN            PE0
#define UART8_TX_PIN            PE1

#define USE_VCP
//#define USE_USB_ID 

#define SERIAL_PORT_COUNT       8

/*
#define USE_SPI

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PD3
#define SPI2_MISO_PIN           PC2
#define SPI2_MOSI_PIN           PC3
#define SPI2_NSS_PIN            PB12

#define USE_SPI_DEVICE_3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PD6
#define SPI3_NSS_PIN            PA15

#define USE_SPI_DEVICE_4
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14
#define SPI4_NSS_PIN            PE11
*/

/*
#define USE_I2C
#define USE_I2C_DEVICE_1
#define I2C1_SCL                PB8
#define I2C1_SDA                PB9
#define I2C_DEVICE              (I2CDEV_1)
*/

#define MAG_I2C_BUS             BUS_I2C1
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
//#define USE_MAG_IST8310
//#define USE_MAG_IST8308
//#define USE_MAG_MAG3110
//#define USE_MAG_LIS3MDL

#define USE_BARO
//#define USE_BARO_BMP388 // BMP388 support not ported yet.

#define USE_TARGET_IMU_HARDWARE_DESCRIPTORS

#define USE_IMU_MPU6500
#define IMU_1_ALIGN                 IMU_MPU6500_1_ALIGN
#define IMU_2_ALIGN                 IMU_MPU6500_2_ALIGN

//#define USE_GYRO // BF define
//#define USE_GYRO_SPI_MPU6500
#define USE_MULTI_GYRO
#undef USE_GYRO_REGISTER_DUMP

/*
#define USE_EXTI
// #define USE_GYRO_EXTI // BF define
#define GYRO_1_EXTI_PIN         PD4
#define GYRO_2_EXTI_PIN         PE15
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
*/

#define GYRO_1_CS_PIN           SPI3_NSS_PIN
#define GYRO_1_SPI_INSTANCE     BUS_SPI3

#define GYRO_2_CS_PIN           SPI2_NSS_PIN
#define GYRO_2_SPI_INSTANCE     BUS_SPI2

#define MPU6500_1_CS_PIN                    GYRO_1_CS_PIN
#define MPU6500_1_SPI_BUS                   GYRO_1_SPI_INSTANCE
#define MPU6500_2_CS_PIN                    GYRO_2_CS_PIN
#define MPU6500_2_SPI_BUS                   GYRO_2_SPI_INSTANCE

#define IMU_MPU6500_1_ALIGN                 CW180_DEG
#define IMU_MPU6500_2_ALIGN                 CW0_DEG // FIXME requires ALIGN_CUSTOM support to be ported, gyro 2 is at 225 degrees yaw/z offset to gyro 1.
// Custom gyro alignment not ported yet.
//#define IMU_MPU6500_2_ALIGN                 ALIGN_CUSTOM
//#define IMU_MPU6500_2_ALIGN_CUSTOM_ALIGN    SENSOR_ALIGNMENT(  0, 0, 225)

#define IMU_1_ALIGN                 IMU_MPU6500_1_ALIGN
#define IMU_2_ALIGN                 IMU_MPU6500_2_ALIGN

//#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH // BF define

#define USE_FLASHFS
#define USE_FLASH_TOOLS
//#define USE_FLASH_W25N01G // W25N01G support not ported yet
//#define FLASH_QUADSPI_INSTANCE    QUADSPI // QuadSPI support not ported yet

//#define USE_PID_AUDIO // PID audio not ported yet.

//#define USE_TRANSPONDER // Transporter support not ported yet.
//#define TRANSPONDER_PIN                   PB11 // TIM2 CH4

#define USE_OSD
#define USE_MAX7456
#define MAX7456_SPI_BUS         BUS_SPI4
#define MAX7456_CS_PIN          SPI4_NSS_PIN

// RTC6705 support not ported yet
/*
#define USE_VTX_RTC6705
#define USE_VTX_RTC6705_SOFTSPI
#define VTX_RTC6705_OPTIONAL    // VTX/OSD board is OPTIONAL

#define RTC6705_POWER_PIN                   PB1  // J14-6
#define RTC6705_CS_PIN                      PB0  // J14-5
#define RTC6705_SPICLK_PIN                  PA7  // J14-4
#define RTC6705_SPI_MOSI_PIN                PA6  // J14-3
*/

#define USE_LED_STRIP
#define WS2811_PIN                          PA8 // TIM1 CH1

// BF DMA spec not ported
/*
#ifdef USE_DMA_SPEC
//#define UART1_TX_DMA_OPT        0
//#define UART2_TX_DMA_OPT        1
//#define UART3_TX_DMA_OPT        2
//#define UART4_TX_DMA_OPT        3
//#define UART5_TX_DMA_OPT        4
//#define UART6_TX_DMA_OPT        5
//#define UART7_TX_DMA_OPT        6
//#define UART8_TX_DMA_OPT        7
#define ADC1_DMA_OPT 8
#define ADC3_DMA_OPT 9
//#define ADC2_DMA_OPT 10 // ADC2 not used.
#else
#define ADC1_DMA_STREAM DMA2_Stream0
#define ADC3_DMA_STREAM DMA2_Stream1
//#define ADC2_DMA_STREAM DMA2_Stream2  // ADC2 not used.
#endif
*/

// SD card via SDIO not ported yet.
/*
#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDCARD_DETECT_PIN PD10
#define SDCARD_DETECT_INVERTED
#define SDIO_DEVICE             SDIODEV_1
#define SDIO_USE_4BIT           true
#define SDIO_CK_PIN             PC12
#define SDIO_CMD_PIN            PD2
#define SDIO_D0_PIN             PC8
#define SDIO_D1_PIN             PC9
#define SDIO_D2_PIN             PC10
#define SDIO_D3_PIN             PC11

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT
*/

// ADC support not ported yet.
// Ideally need to be able to specify ADC instance *and* PIN for each channel, BF is not optimal and always uses first ADC device that supports the pin
// when a single ADC device instance *could* support all the pins.  Using the first ADC device is also BAD when you want to reserve the ADC instance for
// other non-battery/current/rssi uses... (e.g. timer-linked DMA reads of video signals on ADC2), and/or when you want to keep DMA streams free.
// E.g.
/*
#define USE_ADC

#define ADC_CHANNEL_1_PIN                   PC0
#define ADC_CHANNEL_1_INSTANCE              ADC1
#define ADC_CHANNEL_2_PIN                   PC1
#define ADC_CHANNEL_2_INSTANCE              ADC1
#define ADC_CHANNEL_3_PIN                   PC4
#define ADC_CHANNEL_3_INSTANCE              ADC1
#define ADC_CHANNEL_4_PIN                   PC5
#define ADC_CHANNEL_4_INSTANCE              ADC1

// H7 has internal ADC channels on ADC3, no PINs (for VREFInt and Core Temperature)
#define USE_ADC_INTERNAL
#define ADC_INTERNAL_INSTANCE               ADC3

#define RSSI_ADC_CHANNEL                    ADC_CHANNEL_3_PIN  // ADC123
#define VBAT_ADC_CHANNEL                    ADC_CHANNEL_2_PIN  // ADC12
#define CURRENT_METER_ADC_CHANNEL           ADC_CHANNEL_1_PIN  // ADC123
#define AIRSPEED_ADC_CHANNEL                ADC_CHANNEL_4_PIN  // ADC12
*/

#define CURRENT_METER_SCALE                 225


#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
// Disabled some features until H7 support is added.
//#define DEFAULT_FEATURES        (FEATURE_TRANSPONDER | FEATURE_RSSI_ADC | FEATURE_TELEMETRY | FEATURE_OSD | FEATURE_LED_STRIP | FEATURE_VBAT | FEATURE_CURRENT_METER | FEATURE_BLACKBOX)
#define DEFAULT_FEATURES        (FEATURE_TELEMETRY | FEATURE_OSD)

#define GPS_UART                            SERIAL_PORT_USART3
#define SERIALRX_UART                       SERIAL_PORT_USART1
#define SERIALRX_PROVIDER                   SERIALRX_SBUS
// UART2 often for telemetry
// UART4 often used for SmartAudio/Tramp

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
#define TARGET_IO_PORTF 0xffff
#define TARGET_IO_PORTG 0xffff

//#define USABLE_TIMER_CHANNEL_COUNT 19
//#define USED_TIMERS  ( TIM_N(1) | TIM_N(2) | TIM_N(3) | TIM_N(4) | TIM_N(5) | TIM_N(8) | TIM_N(12) | TIM_N(15) )

#define MAX_PWM_OUTPUT_PORTS    12

#define USE_DSHOT
#define USE_ESC_SENSOR
