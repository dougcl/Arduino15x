/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_FBA_ 
#define _VARIANT_FBA_

/*----------------------------------------------------------------------------
 *        Definitions
 *----------------------------------------------------------------------------*/

/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			64000000

/** USB Stuff used in USBCore.cpp */
#define UDD_USB_INT_LEVEL 		5 // By default USB interrupt have low priority
#define MAX_ENDPOINTS			7
#define EP0 					0 //Control endpoint is endpoint zero
#define EP_TYPE_CONTROL 		UDP_CSR_EPTYPE_CTRL
#define EP_TYPE_INTERRUPT_IN	UDP_CSR_EPTYPE_INT_IN
#define EP_TYPE_BULK_OUT		UDP_CSR_EPTYPE_BULK_OUT
#define EP_TYPE_BULK_IN			UDP_CSR_EPTYPE_BULK_IN 

/** end of USB stuff */
/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "Arduino.h"
#ifdef __cplusplus
#include "UARTClass.h"
#include "USARTClass.h"
#endif

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

/**
 * Libc porting layers
 */
#if defined (  __GNUC__  ) /* GCC CS3 */
#    include <syscalls.h> /** RedHat Newlib minimal stub */
#endif

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

// Number of pins defined in PinDescription array
#define PINS_COUNT           (27u)
#define NUM_DIGITAL_PINS     (8u)
#define NUM_ANALOG_INPUTS    (8u)

#define digitalPinToPort(P)        ( g_APinDescription[P].pPort )
#define digitalPinToBitMask(P)     ( g_APinDescription[P].ulPin )
//#define analogInPinToBit(P)        ( )
#define portOutputRegister(port)   ( &(port->PIO_ODSR) )
#define portInputRegister(port)    ( &(port->PIO_PDSR) )
#define digitalPinHasPWM(P)        ( g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER )

/*
 * portModeRegister(..) should return a register to set pin mode
 * INPUT or OUTPUT by setting the corresponding bit to 0 or 1.
 * Unfortunately on SAM architecture the PIO_OSR register is
 * read-only and can be set only through the enable/disable registers
 * pair PIO_OER/PIO_ODR.
 */
// #define portModeRegister(port)   ( &(port->PIO_OSR) )

/*
 * digitalPinToTimer(..) is AVR-specific and is not defined for SAM
 * architecture. If you need to check if a pin supports PWM you must
 * use digitalPinHasPWM(..).
 *
 * https://github.com/arduino/Arduino/issues/1833
 */
// #define digitalPinToTimer(P)

// Interrupts
#define digitalPinToInterrupt(p)  ((p) < NUM_DIGITAL_PINS ? (p) : -1)

// LEDs
//#define PIN_LED_13           (13u)
//#define PIN_LED_RXL          (72u)
//#define PIN_LED_TXL          (73u)
//#define PIN_LED              PIN_LED_13
//#define PIN_LED2             PIN_LED_RXL
//#define PIN_LED3             PIN_LED_TXL
//#define LED_BUILTIN          13

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define SPI_INTERFACE        SPI0
#define SPI_INTERFACE_ID     ID_SPI
#define SPI_CHANNELS_NUM 4
#define PIN_SPI_SS0          (13u)
#define PIN_SPI_SS1          (15u)
#define PIN_SPI_SS2          (14u)
#define PIN_SPI_SS3          (18u)
#define PIN_SPI_MOSI         (11u)
#define PIN_SPI_MISO         (12u)
#define PIN_SPI_SCK          (10u)
#define BOARD_SPI_SS0        PIN_SPI_SS0
#define BOARD_SPI_SS1        PIN_SPI_SS1
#define BOARD_SPI_SS2        PIN_SPI_SS2
#define BOARD_SPI_SS3        PIN_SPI_SS3
#define BOARD_SPI_DEFAULT_SS BOARD_SPI_SS0

#define BOARD_PIN_TO_SPI_PIN(x) \
	(x==BOARD_SPI_SS0 ? PIN_SPI_SS0 : \
	(x==BOARD_SPI_SS1 ? PIN_SPI_SS1 : \
	(x==BOARD_SPI_SS2 ? PIN_SPI_SS2 : PIN_SPI_SS3 )))
#define BOARD_PIN_TO_SPI_CHANNEL(x) \
	(x==BOARD_SPI_SS0 ? 0 : \
	(x==BOARD_SPI_SS1 ? 1 : \
	(x==BOARD_SPI_SS2 ? 2 : 3)))

static const uint8_t SS   = BOARD_SPI_SS0;
static const uint8_t SS1  = BOARD_SPI_SS1;
static const uint8_t SS2  = BOARD_SPI_SS2;
static const uint8_t SS3  = BOARD_SPI_SS3;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA        (16u)
#define PIN_WIRE_SCL        (24u)
#define WIRE_INTERFACE      TWI1
#define WIRE_INTERFACE_ID   ID_TWI1
#define WIRE_ISR_HANDLER    TWI1_Handler
#define WIRE_ISR_ID         TWI1_IRQn

//This one uses custom I2C library.
#define PIN_WIRE1_SDA         (20u)
#define PIN_WIRE1_SCL         (19u)
#define WIRE1_INTERFACE       TWI0
#define WIRE1_INTERFACE_ID    ID_TWI0
#define WIRE1_ISR_HANDLER     TWI0_Handler
#define WIRE1_ISR_ID          TWI0_IRQn


/*
 * UART/USART Interfaces
 */
// Serial
//#define PINS_UART            (81u)
// Serial1
//#define PINS_USART0          (82u)
// Serial2
//#define PINS_USART1          (83u)
// Serial3
//#define PINS_USART3          (84u)

/*
 * USB Interfaces
 */
// #define PINS_USB             (85u) //Due appears to be pointing to UOTGID which we don't have

/*
 * Analog pins
 */
static const uint8_t A0  = 0;
static const uint8_t A1  = 1;
static const uint8_t A2  = 2;
static const uint8_t A3  = 3;
static const uint8_t A4  = 4;
static const uint8_t A5  = 5;
static const uint8_t A6  = 6;
static const uint8_t A7  = 7;
//static const uint8_t A8  = 62;
//static const uint8_t A9  = 63;
//static const uint8_t A10 = 64;
//static const uint8_t A11 = 65;
//static const uint8_t DAC0 = 66;
//static const uint8_t DAC1 = 67;
//static const uint8_t CANRX = 68;
//static const uint8_t CANTX = 69;
#define ADC_RESOLUTION		12

/*
 * Complementary CAN pins
 */
//static const uint8_t CAN1RX = 88;
//static const uint8_t CAN1TX = 89;

// CAN0
//#define PINS_CAN0            (90u)
// CAN1
//#define PINS_CAN1            (91u)


/*
 * DACC
 */
//#define DACC_INTERFACE		DACC
//#define DACC_INTERFACE_ID	ID_DACC
//#define DACC_RESOLUTION		12
//#define DACC_ISR_HANDLER    DACC_Handler
//#define DACC_ISR_ID         DACC_IRQn

/*
 * PWM
 */
//#define PWM_INTERFACE		PWM
//#define PWM_INTERFACE_ID	ID_PWM
//#define PWM_FREQUENCY		1000
//#define PWM_MAX_DUTY_CYCLE	255
//#define PWM_MIN_DUTY_CYCLE	0
//#define PWM_RESOLUTION		8

/*
 * TC
 */
//#define TC_INTERFACE        TC0
//#define TC_INTERFACE_ID     ID_TC0
//#define TC_FREQUENCY        1000
//#define TC_MAX_DUTY_CYCLE   255
//#define TC_MIN_DUTY_CYCLE   0
//#define TC_RESOLUTION		8

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus

extern UARTClass Serial;
extern USARTClass Serial1;
extern USARTClass Serial2;
extern USARTClass Serial3;

#endif

// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
//SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_USBVIRTUAL      SerialUSB
//#define SERIAL_PORT_HARDWARE_OPEN   Serial1
//#define SERIAL_PORT_HARDWARE_OPEN1  Serial2
//#define SERIAL_PORT_HARDWARE_OPEN2  Serial3
//#define SERIAL_PORT_HARDWARE        Serial
//#define SERIAL_PORT_HARDWARE1       Serial1
//#define SERIAL_PORT_HARDWARE2       Serial2
//#define SERIAL_PORT_HARDWARE3       Serial3

#endif /* _VARIANT_FBA_ */

