/**
 * \file
 *
 * \brief User board definition template
 *
 */

 /* This file is intended to contain definitions and configuration details for
 * features and devices that are available on the board, e.g., frequency and
 * startup time for an external crystal, external memory devices, LED and USART
 * pins.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef USER_BOARD_H
#define USER_BOARD_H

#include <conf_board.h>

// External oscillator settings.
// Uncomment and set correct values if external oscillator is used.

// External oscillator frequency
//#define BOARD_XOSC_HZ          8000000

// External oscillator type.
//!< External clock signal
//#define BOARD_XOSC_TYPE        XOSC_TYPE_EXTERNAL
//!< 32.768 kHz resonator on TOSC
//#define BOARD_XOSC_TYPE        XOSC_TYPE_32KHZ
//!< 0.4 to 16 MHz resonator on XTALS
//#define BOARD_XOSC_TYPE        XOSC_TYPE_XTAL

// External oscillator startup time
//#define BOARD_XOSC_STARTUP_US  500000

#define EN_VPA_GPIO			IOPORT_CREATE_PIN(PORTF, 7)
#define EN_VPRE_GPIO		IOPORT_CREATE_PIN(PORTF, 6)

//#define SD_MMC_ENABLE

//! \name SPI microSD
//@{

#define SD_MMC_SPI_MEM_CNT              1
#define SD_MMC_SPI_USES_USART_SPI_SERVICE // To signal that is a USART in SPI mode#ifdef CONF_BOARD_SD_MMC_SPI
#ifdef CONF_BOARD_SD_MMC_SPI
#define SD_MMC_0_CD_GPIO            IOPORT_CREATE_PIN(PORTD, 4)
#define SD_MMC_0_CD_DETECT_VALUE        0
#endif

#define SD_MMC_SPI                      &USARTD0
#define SD_MMC_SPI_SCK                  IOPORT_CREATE_PIN(PORTD, 1)
#define SD_MMC_SPI_MISO                 IOPORT_CREATE_PIN(PORTD, 2)
#define SD_MMC_SPI_MOSI                 IOPORT_CREATE_PIN(PORTD, 3)
#define SD_MMC_SPI_0_CS                 IOPORT_CREATE_PIN(PORTD, 0)
//@}

//DDS
#define DDS_SPI		                    &USARTC0
#define DDS_SPI_SCK						IOPORT_CREATE_PIN(PORTC, 1)
#define DDS_SPI_MISO					IOPORT_CREATE_PIN(PORTC, 2)
#define DDS_SPI_MOSI					IOPORT_CREATE_PIN(PORTC, 3)
#define DDS_SPI_0_CS					IOPORT_CREATE_PIN(PORTC, 0)

#define DDS_IOUPDATE					IOPORT_CREATE_PIN(PORTF, 5)
#define DDS_PS0							IOPORT_CREATE_PIN(PORTC, 5)
#define DDS_PS1							IOPORT_CREATE_PIN(PORTC, 6)
#define DDS_PWRDWN						IOPORT_CREATE_PIN(PORTC, 7)
#define DDS_OSK							IOPORT_CREATE_PIN(PORTF, 4)
#define DDS_RESET						IOPORT_CREATE_PIN(PORTC, 4)

//LED
#define LED0							IOPORT_CREATE_PIN(PORTE, 0)
#define LED1							IOPORT_CREATE_PIN(PORTE, 1)
#define LED2							IOPORT_CREATE_PIN(PORTE, 4)

#define LED_ON(led)						ioport_set_pin_low(led)
#define LED_OFF(led)					ioport_set_pin_high(led)

//PA
#define PA_LEVEL_3W			1700
#define PA_LEVEL_2W			1550
#define PA_LEVEL_1W			1400
#define PA_LEVEL_0_5W		1310

#endif // USER_BOARD_H
