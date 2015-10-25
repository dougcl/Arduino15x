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

#include "variant.h"

/*
 * Pin             |  PORT  | Label
 * ----------------+--------+-------
 *   0       AD0   |  PA17  | "AI-AD0"
 *   1       AD1   |  PA18  | "AI-AD1"
 *   2       AD2   |  PA19  | "AI-AD2"
 *   3       AD3   |  PA20  | "AI-AD3"
 *   4       AD4   |  PB0   | "AI-AD4"
 *   5       AD5   |  PB1   | "AI-AD5"
 *   6       AD6   |  PB2   | "AI-AD6"
 *   7       AD7   |  PB3   | "AI-AD7"
 *   8             |  PA16  | "DO-REM-ENBL"
 *   9             |  PA15  | "DI-REM-ENBL"
 *  10       SPCK  |  PA14  | "SPCK"
 *  11       MOSI  |  PA13  | "MOSI"
 *  12       MISO  |  PA12  | "MISO"
 *  13       NPCS0 |  PA11  | "NPCS0"
 *  14       NPCS2 |  PA10  | "NPCS2"
 *  15       NPCS1 |  PA9   | "NPCS1"
 *  16       TWD1  |  PB4   | "SDA1"
 *  17             |  PA6   |
 *  18       NPCS3 |  PA5   | "NPCS3"
 *  19       TWCK0 |  PA4   | "SCL0"
 *  20       TWD0  |  PA3   | "SDA0"
 *  21             |  PA2   | "DO-AI-SELECT"
 *  22             |  PA1   | "BUS1"
 *  23             |  PA0   | "BUS0"
 *  24       TWCK1 |  PB5   | "SCL1"
 */

#ifdef __cplusplus
extern "C" {
#endif



/* USB Functions used by USBCore.cpp */

/* end USB Functions */


/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
  //0 .. 7 - Analog input pins
  // ----------------------
  { PIOA, PIO_PA17X1_AD0,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC0,   ADC0,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD0
  { PIOA, PIO_PA18X1_AD1,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC1,   ADC1,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD1
  { PIOA, PIO_PA19X1_AD2,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC2,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD2
  { PIOA, PIO_PA20X1_AD3,    ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC3,   ADC3,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD3
  // 4
  { PIOB, PIO_PB0X1_AD4,     ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC4,   ADC4,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD4
  { PIOB, PIO_PB1X1_AD5,     ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC5,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD5
  { PIOB, PIO_PB2X1_AD6,     ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC6,   ADC6,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD6
  { PIOB, PIO_PB3X1_AD7,     ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC7,   ADC7,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD7
  // 8..21 - Digital pins
  // ----------------------
  // 8/9 Remote enable
  { PIOA, PIO_PA16,            ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // LED
  { PIOA, PIO_PA15,            ID_PIOA, PIO_INPUT, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // Button
  // 10/11/12 - SPI
  { PIOA, PIO_PA14A_SPCK, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // SPCK
  { PIOA, PIO_PA13A_MOSI, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MOSI
  { PIOA, PIO_PA12A_MISO, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // MISO
  // 13 - SPI CS0
  { PIOA, PIO_PA11A_NPCS0,ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS0
  // 14 - SPI CS2
  { PIOA, PIO_PA10B_NPCS2,ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS2
  // 15 - SPI CS1
  { PIOA, PIO_PA9B_NPCS1, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS1
  // 16 - TWD1
  { PIOB, PIO_PB4A_TWD1,       ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD1 - SDA1
  // 17 unused
  { PIOA, PIO_PA6,             ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // PIN 17
  // 18 - SPI CS3
  { PIOA, PIO_PA5B_NPCS3, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // NPCS3
  // 19/20 - TWI0
  { PIOA, PIO_PA4A_TWCK0,      ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK0 - SCL0
  { PIOA, PIO_PA3A_TWD0,       ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWD0 - SDA0
  // 21 AI Select
  { PIOA, PIO_PA2,            ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // DO-AI_SELECT
  // 22/23 Bus select
  { PIOA, PIO_PA1,             ID_PIOA, PIO_INPUT, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // BUS1
  { PIOA, PIO_PA0,             ID_PIOA, PIO_INPUT, PIO_DEFAULT, PIN_ATTR_DIGITAL,                 NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // BUS0
  // 24 - TWCK1
  { PIOB, PIO_PB5A_TWCK1,       ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER }, // TWCK1 - SCL1
  
  
  // 25- TWI0 all pins
  { PIOA, PIO_PA3A_TWD0|PIO_PA4A_TWCK0, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },
  // 26 - TWI1 all pins
  { PIOB, PIO_PB4A_TWD1|PIO_PB5A_TWCK1, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  
  
  // END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
//RingBuffer rx_buffer1;

//UARTClass Serial(UART, UART_IRQn, ID_UART, &rx_buffer1);
//void serialEvent() __attribute__((weak));
//void serialEvent() { }

// IT handlers
//void UART_Handler(void)
//{
//  Serial.IrqHandler();
//}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
//RingBuffer rx_buffer2;
//RingBuffer rx_buffer3;
//RingBuffer rx_buffer4;

//USARTClass Serial1(USART0, USART0_IRQn, ID_USART0, &rx_buffer2);
//void serialEvent1() __attribute__((weak));
//void serialEvent1() { }
//USARTClass Serial2(USART1, USART1_IRQn, ID_USART1, &rx_buffer3);
//void serialEvent2() __attribute__((weak));
//void serialEvent2() { }
//USARTClass Serial3(USART3, USART3_IRQn, ID_USART3, &rx_buffer4);
//void serialEvent3() __attribute__((weak));
//void serialEvent3() { }

// IT handlers
//void USART0_Handler(void)
//{
//  Serial1.IrqHandler();
//}

//void USART1_Handler(void)
//{
//  Serial2.IrqHandler();
//}

//void USART3_Handler(void)
//{
//  Serial3.IrqHandler();
//}

// ----------------------------------------------------------------------------

//void serialEventRun(void)
//{
//  if (Serial.available()) serialEvent();
//  if (Serial1.available()) serialEvent1();
//  if (Serial2.available()) serialEvent2();
//  if (Serial3.available()) serialEvent3();
//}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
  SystemInit();

  // Set Systick to 1ms interval, common to all SAM3 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
    while (true);
  }

  // Disable watchdog
  WDT_Disable(WDT);

  // Initialize C library
  __libc_init_array();

  // Disable pull-up on every pin
  for (unsigned i = 0; i < PINS_COUNT; i++)
	  digitalWrite(i, LOW);

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;
  //PIOC->PIO_OWER = 0xFFFFFFFF;
  //PIOD->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port U(S)ART pins
  //PIO_Configure(
  //  g_APinDescription[PINS_UART].pPort,
  //  g_APinDescription[PINS_UART].ulPinType,
  //  g_APinDescription[PINS_UART].ulPin,
  //  g_APinDescription[PINS_UART].ulPinConfiguration);
  //digitalWrite(0, HIGH); // Enable pullup for RX0
  //PIO_Configure(
  //  g_APinDescription[PINS_USART0].pPort,
  //  g_APinDescription[PINS_USART0].ulPinType,
  //  g_APinDescription[PINS_USART0].ulPin,
  //  g_APinDescription[PINS_USART0].ulPinConfiguration);
  //PIO_Configure(
  //  g_APinDescription[PINS_USART1].pPort,
  //  g_APinDescription[PINS_USART1].ulPinType,
  //  g_APinDescription[PINS_USART1].ulPin,
  //  g_APinDescription[PINS_USART1].ulPinConfiguration);
  //PIO_Configure(
  //  g_APinDescription[PINS_USART3].pPort,
  //  g_APinDescription[PINS_USART3].ulPinType,
  //  g_APinDescription[PINS_USART3].ulPin,
  //  g_APinDescription[PINS_USART3].ulPinConfiguration);

  // Initialize USB pins
  //PIO_Configure(
  //  g_APinDescription[PINS_USB].pPort,
  //  g_APinDescription[PINS_USB].ulPinType,
  //  g_APinDescription[PINS_USB].ulPin,
  //  g_APinDescription[PINS_USB].ulPinConfiguration);

  // Initialize CAN pins
  //PIO_Configure(
  //  g_APinDescription[PINS_CAN0].pPort,
  //  g_APinDescription[PINS_CAN0].ulPinType,
  //  g_APinDescription[PINS_CAN0].ulPin,
  //  g_APinDescription[PINS_CAN0].ulPinConfiguration);
  //PIO_Configure(
  //  g_APinDescription[PINS_CAN1].pPort,
  //  g_APinDescription[PINS_CAN1].ulPinType,
  //  g_APinDescription[PINS_CAN1].ulPin,
  //  g_APinDescription[PINS_CAN1].ulPinConfiguration);

  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

  // Initialize analogOutput module
  //analogOutputInit();
  
  //Add this here or delay(1) fails in main.cpp due to SysTick not firing. Not sure why.
  cpu_irq_enable();
}

#ifdef __cplusplus
}
#endif

