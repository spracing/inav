/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * Dominic Clifton - Port baseflight STM32F10x to STM32F30x for cleanflight
 * J. Ihlein - Code from FocusFlight32
 * Bill Nesbitt - Code from AutoQuad
 * Hamasaki/Timecop - Initial baseflight code
*/

#include <stdbool.h>
#include <stdint.h>

#include <platform.h>

#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/nvic.h"
#include "rcc.h"

#include "serial.h"
#include "serial_uart.h"
#include "serial_uart_impl.h"

#ifdef USE_UART1
#ifndef UART1_TX_PIN
#define UART1_TX_PIN        PA9  // PA9
#endif
#ifndef UART1_RX_PIN
#define UART1_RX_PIN        PA10 // PA10
#endif
#endif

#ifdef USE_UART2
#ifndef UART2_TX_PIN
#define UART2_TX_PIN        PD5 // PD5
#endif
#ifndef UART2_RX_PIN
#define UART2_RX_PIN        PD6 // PD6
#endif
#endif

#ifdef USE_UART3
#ifndef UART3_TX_PIN
#define UART3_TX_PIN        PB10 // PB10 (AF7)
#endif
#ifndef UART3_RX_PIN
#define UART3_RX_PIN        PB11 // PB11 (AF7)
#endif
#endif

#ifdef USE_UART4
#ifndef UART4_TX_PIN
#define UART4_TX_PIN        PC10 // PC10 (AF5)
#endif
#ifndef UART4_RX_PIN
#define UART4_RX_PIN        PC11 // PC11 (AF5)
#endif
#endif

#ifdef USE_UART5
#ifndef UART5_TX_PIN             // The real UART5_RX is on PD2, no board is using.
#define UART5_TX_PIN        PC12 // PC12 (AF5)
#endif
#ifndef UART5_RX_PIN
#define UART5_RX_PIN        PC12 // PC12 (AF5)
#endif
#endif

#ifdef USE_UART1
static uartPort_t uartPort1;
#endif
#ifdef USE_UART2
static uartPort_t uartPort2;
#endif
#ifdef USE_UART3
static uartPort_t uartPort3;
#endif
#ifdef USE_UART4
static uartPort_t uartPort4;
#endif
#ifdef USE_UART5
static uartPort_t uartPort5;
#endif


void uartGetPortPins(UARTDevice_e device, serialPortPins_t * pins)
{
    switch(device) {
#ifdef USE_UART1
        case UARTDEV_1:
            pins->txPin = IO_TAG(UART1_TX_PIN);
            pins->rxPin = IO_TAG(UART1_RX_PIN);
            break;
#endif
#ifdef USE_UART2
        case UARTDEV_2:
            pins->txPin = IO_TAG(UART2_TX_PIN);
            pins->rxPin = IO_TAG(UART2_RX_PIN);
            break;
#endif
#ifdef USE_UART3
        case UARTDEV_3:
            pins->txPin = IO_TAG(UART3_TX_PIN);
            pins->rxPin = IO_TAG(UART3_RX_PIN);
            break;
#endif
#ifdef USE_UART4
        case UARTDEV_4:
            pins->txPin = IO_TAG(UART4_TX_PIN);
            pins->rxPin = IO_TAG(UART4_RX_PIN);
            break;
#endif
#ifdef USE_UART5
        case UARTDEV_5:
            pins->txPin = IO_TAG(UART5_TX_PIN);
            pins->rxPin = IO_TAG(UART5_RX_PIN);
            break;
#endif
        default:
            pins->txPin = IO_TAG(NONE);
            pins->rxPin = IO_TAG(NONE);
            break;
    }
}

void serialUARTInit(IO_t tx, IO_t rx, portMode_t mode, portOptions_t options, uint8_t af, uint8_t index)
{
    if (options & SERIAL_BIDIR) {
        ioConfig_t ioCfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_OType_PP : GPIO_OType_OD,
            ((options & SERIAL_INVERTED) || (options & SERIAL_BIDIR_PP)) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP
        );

        IOInit(tx, OWNER_SERIAL, RESOURCE_UART_TXRX, index);
        IOConfigGPIOAF(tx, ioCfg, af);

        if (!(options & SERIAL_INVERTED))
            IOLo(tx);   // OpenDrain output should be inactive
    } else {
        ioConfig_t ioCfg = IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, (options & SERIAL_INVERTED) ? GPIO_PuPd_DOWN : GPIO_PuPd_UP);
        if (mode & MODE_TX) {
            IOInit(tx, OWNER_SERIAL, RESOURCE_UART_TX, index);
            IOConfigGPIOAF(tx, ioCfg, af);
        }

        if (mode & MODE_RX) {
            IOInit(rx, OWNER_SERIAL, RESOURCE_UART_RX, index);
            IOConfigGPIOAF(rx, ioCfg, af);
        }
    }
}

#ifdef USE_UART1
uartPort_t *serialUART1(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;

    static volatile uint8_t rx1Buffer[UART1_RX_BUFFER_SIZE];
    static volatile uint8_t tx1Buffer[UART1_TX_BUFFER_SIZE];

    s = &uartPort1;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBuffer = rx1Buffer;
    s->port.txBuffer = tx1Buffer;
    s->port.rxBufferSize = UART1_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART1_TX_BUFFER_SIZE;

    s->USARTx = USART1;

    RCC_ClockCmd(RCC_APB2(USART1), ENABLE);

    serialUARTInit(IOGetByTag(IO_TAG(UART1_TX_PIN)), IOGetByTag(IO_TAG(UART1_RX_PIN)), mode, options, GPIO_AF_7, 1);

    NVIC_SetPriority(USART1_IRQn, NVIC_PRIO_SERIALUART);
    NVIC_EnableIRQ(USART1_IRQn);

    return s;
}
#endif

#ifdef USE_UART2
uartPort_t *serialUART2(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;

    static volatile uint8_t rx2Buffer[UART2_RX_BUFFER_SIZE];
    static volatile uint8_t tx2Buffer[UART2_TX_BUFFER_SIZE];

    s = &uartPort2;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBufferSize = UART2_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART2_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx2Buffer;
    s->port.txBuffer = tx2Buffer;

    s->USARTx = USART2;

    RCC_ClockCmd(RCC_APB1(USART2), ENABLE);

    serialUARTInit(IOGetByTag(IO_TAG(UART2_TX_PIN)), IOGetByTag(IO_TAG(UART2_RX_PIN)), mode, options, GPIO_AF_7, 2);

    NVIC_SetPriority(USART2_IRQn, NVIC_PRIO_SERIALUART);
    NVIC_EnableIRQ(USART2_IRQn);

    return s;
}
#endif

#ifdef USE_UART3
uartPort_t *serialUART3(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;

    static volatile uint8_t rx3Buffer[UART3_RX_BUFFER_SIZE];
    static volatile uint8_t tx3Buffer[UART3_TX_BUFFER_SIZE];

    s = &uartPort3;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBufferSize = UART3_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART3_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx3Buffer;
    s->port.txBuffer = tx3Buffer;

    s->USARTx = USART3;

    RCC_ClockCmd(RCC_APB1(USART3), ENABLE);

    serialUARTInit(IOGetByTag(IO_TAG(UART3_TX_PIN)), IOGetByTag(IO_TAG(UART3_RX_PIN)), mode, options, GPIO_AF_7, 3);

    NVIC_SetPriority(USART3_IRQn, NVIC_PRIO_SERIALUART);
    NVIC_EnableIRQ(USART3_IRQn);

    return s;
}
#endif

#ifdef USE_UART4
uartPort_t *serialUART4(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx4Buffer[UART4_RX_BUFFER_SIZE];
    static volatile uint8_t tx4Buffer[UART4_TX_BUFFER_SIZE];

    s = &uartPort4;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBufferSize = UART4_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART4_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx4Buffer;
    s->port.txBuffer = tx4Buffer;

    s->USARTx = UART4;

    RCC_ClockCmd(RCC_APB1(UART4), ENABLE);

    serialUARTInit(IOGetByTag(IO_TAG(UART4_TX_PIN)), IOGetByTag(IO_TAG(UART4_RX_PIN)), mode, options, GPIO_AF_5, 4);

    NVIC_SetPriority(UART4_IRQn, NVIC_PRIO_SERIALUART);
    NVIC_EnableIRQ(UART4_IRQn);

    return s;
}
#endif

#ifdef USE_UART5
uartPort_t *serialUART5(uint32_t baudRate, portMode_t mode, portOptions_t options)
{
    uartPort_t *s;
    static volatile uint8_t rx5Buffer[UART5_RX_BUFFER_SIZE];
    static volatile uint8_t tx5Buffer[UART5_TX_BUFFER_SIZE];

    s = &uartPort5;
    s->port.vTable = uartVTable;

    s->port.baudRate = baudRate;

    s->port.rxBufferSize = UART5_RX_BUFFER_SIZE;
    s->port.txBufferSize = UART5_TX_BUFFER_SIZE;
    s->port.rxBuffer = rx5Buffer;
    s->port.txBuffer = tx5Buffer;

    s->USARTx = UART5;

    RCC_ClockCmd(RCC_APB1(UART5), ENABLE);

    serialUARTInit(IOGetByTag(IO_TAG(UART5_TX_PIN)), IOGetByTag(IO_TAG(UART5_RX_PIN)), mode, options, GPIO_AF_5, 5);

    NVIC_SetPriority(UART5_IRQn, NVIC_PRIO_SERIALUART);
    NVIC_EnableIRQ(UART5_IRQn);

    return s;
}
#endif

void usartIrqHandler(uartPort_t *s)
{
    uint32_t ISR = s->USARTx->ISR;

    if (ISR & USART_FLAG_RXNE) {
        if (s->port.rxCallback) {
            s->port.rxCallback(s->USARTx->RDR, s->port.rxCallbackData);
        } else {
            s->port.rxBuffer[s->port.rxBufferHead++] = s->USARTx->RDR;
            if (s->port.rxBufferHead >= s->port.rxBufferSize) {
                s->port.rxBufferHead = 0;
            }
        }
    }

    if (ISR & USART_FLAG_TXE) {
        if (s->port.txBufferTail != s->port.txBufferHead) {
            USART_SendData(s->USARTx, s->port.txBuffer[s->port.txBufferTail++]);
            if (s->port.txBufferTail >= s->port.txBufferSize) {
                s->port.txBufferTail = 0;
            }
        } else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }

    if (ISR & USART_FLAG_ORE)
    {
        USART_ClearITPendingBit (s->USARTx, USART_IT_ORE);
    }
}

void uartClearIdleFlag(uartPort_t *s)
{
    USART_ClearITPendingBit(s->USARTx, USART_IT_IDLE);
}

#ifdef USE_UART1
void USART1_IRQHandler(void)
{
    uartPort_t *s = &uartPort1;

    usartIrqHandler(s);
}
#endif

#ifdef USE_UART2
void USART2_IRQHandler(void)
{
    uartPort_t *s = &uartPort2;

    usartIrqHandler(s);
}
#endif

#ifdef USE_UART3
void USART3_IRQHandler(void)
{
    uartPort_t *s = &uartPort3;

    usartIrqHandler(s);
}
#endif

#ifdef USE_UART4
void UART4_IRQHandler(void)
{
    uartPort_t *s = &uartPort4;

    usartIrqHandler(s);
}
#endif

#ifdef USE_UART5
void UART5_IRQHandler(void)
{
    uartPort_t *s = &uartPort5;

    usartIrqHandler(s);
}
#endif
