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

#include <stdint.h>

#include "platform.h"
#include "drivers/io.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "drivers/dma.h"
#include "drivers/bus.h"
#include "drivers/sensor.h"

#if defined(USE_SPI)
// Register both MPU6500
BUSDEV_REGISTER_SPI_TAG(busdev_mpu6500_1,     DEVHW_MPU6500,      MPU6500_1_SPI_BUS,    MPU6500_1_CS_PIN,     GYRO_1_EXTI_PIN,  0,  DEVFLAGS_NONE,  IMU_1_ALIGN);
BUSDEV_REGISTER_SPI_TAG(busdev_mpu6500_2,     DEVHW_MPU6500,      MPU6500_2_SPI_BUS,    MPU6500_2_CS_PIN,     GYRO_2_EXTI_PIN,  0,  DEVFLAGS_NONE,  IMU_2_ALIGN);
#endif

#if defined(USE_TIMER)
const timerHardware_t timerHardware[] = {
    DEF_TIM(TIM1,  CH1, PA8,  TIM_USE_LED,                 0,  10), // LED Strip
    //DEF_TIM(TIM2,  CH4, PB11, TIM_USE_TRANSPONDER,         0,  11), // Transponder

    DEF_TIM(TIM12, CH2, PB15, TIM_USE_PPM | TIM_USE_PWM,   0,  0), // Also USART1 RX

    //DEF_TIM(TIM15, CH1, PE5,  TIM_USE_CAMERA_CONTROL,      0,  0), // Camera control
    //DEF_TIM(TIM15, CH2, PE6,  TIM_USE_NONE,                0,  0), // Spare channel, on UART5 connector

    DEF_TIM(TIM5,  CH1, PA0,  TIM_USE_MC_MOTOR,               0,  0), // M1
    DEF_TIM(TIM5,  CH2, PA1,  TIM_USE_MC_MOTOR,               0,  1),
    DEF_TIM(TIM5,  CH3, PA2,  TIM_USE_MC_MOTOR,               0,  2),
    DEF_TIM(TIM5,  CH4, PA3,  TIM_USE_MC_MOTOR,               0,  3), // M4

    DEF_TIM(TIM4,  CH1, PB6,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  4), // M5
    DEF_TIM(TIM4,  CH2, PB7,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  5), // M6

    DEF_TIM(TIM8,  CH1, PC6,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  6), // M7
    DEF_TIM(TIM8,  CH2, PC7,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  7), // M8

    DEF_TIM(TIM4,  CH3, PD14, TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  12), // M9
    DEF_TIM(TIM4,  CH4, PD15, TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  0), // M10 // Note: No DMA on TIM4_CH4, can use with BURST DSHOT using TIM4_UP

    DEF_TIM(TIM3,  CH1, PA6,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  0), // Also TIM13/CH1
    DEF_TIM(TIM3,  CH2, PA7,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  0), // Also TIM14/CH1
    DEF_TIM(TIM3,  CH3, PB0,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  0), // Also TIM8/CH2_N
    DEF_TIM(TIM3,  CH4, PB1,  TIM_USE_FW_SERVO | TIM_USE_PWM, 0,  0), // Also TIM8/CH3_N
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
#endif
