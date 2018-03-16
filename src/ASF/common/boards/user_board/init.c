/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

void board_init(void)
{
	/* This function is meant to contain board-specific initialization code
	 * for, e.g., the I/O pins. The initialization can rely on application-
	 * specific board configuration, found in conf_board.h.
	 */
	ioport_configure_pin(EN_VPA_GPIO, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(EN_VPRE_GPIO, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	
	//LED
	ioport_configure_pin(LED0, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(LED1, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(LED2, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	
	//MMC/SD
	ioport_configure_pin(SD_MMC_SPI_0_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	#ifdef CONF_BOARD_SD_MMC_SPI
	ioport_configure_pin(SD_MMC_0_CD_GPIO, IOPORT_DIR_INPUT
	| IOPORT_LEVEL | IOPORT_PULL_UP);
	#endif
	ioport_configure_pin(SD_MMC_SPI_SCK, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(SD_MMC_SPI_MOSI, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(SD_MMC_SPI_MISO, IOPORT_DIR_INPUT);
	
	//DDS
	ioport_configure_pin(DDS_SPI_SCK, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(DDS_SPI_MOSI, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(DDS_SPI_0_CS, IOPORT_DIR_OUTPUT | IOPORT_INIT_HIGH);
	ioport_configure_pin(DDS_SPI_MISO, IOPORT_DIR_INPUT);
	
	ioport_configure_pin(DDS_IOUPDATE, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(DDS_PS0, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(DDS_PS1, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(DDS_PWRDWN, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(DDS_OSK, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
	ioport_configure_pin(DDS_RESET, IOPORT_DIR_OUTPUT | IOPORT_INIT_LOW);
}
