/** ========================================
 
  Copyright Thesis Pte Ltd, 2017
  All Rights Reserved
  UNPUBLISHED, LICENSED SOFTWARE.
 
  CONFIDENTIAL AND PROPRIETARY INFORMATION
  WHICH IS THE PROPERTY OF THESIS PTE LTD.
 
  ========================================
  Version: 1.0
  Written by: WIZnet Co., LTD.
	Modified by: Kenneth Er
  ========================================
	SPI interface for STM32F429 (master) to W5500 (slave)
  
	======================================== */
	
#include "main.h"
#include "stm32f4xx_hal.h"
#include "w5500_spi.h"
#include "stm32_spi.h"
#include "../../ioLibrary/Ethernet/wizchip_conf.h"
#include "peripheral.h"

/**
		@brief: w5500 cs pin select wrapper
*/
void  wizchip_select(void)
{
	Ethernet_ChipSelect(false);
}

/**
		@brief: w5500 cs pin deselect wrapper
*/
void  wizchip_deselect(void)
{
	Ethernet_ChipSelect(true);
}

/**
		@brief: w5500 1 byte read wrapper
		@retval: read byte
*/
uint8_t wizchip_read(void)
{
    uint8_t rb;

    rb = stm32_spi_recv_byte();

    return rb;
}

/**
		@brief: w5500 1 byte write wrapper
		@param[in]: wb = byte for writing
*/
void  wizchip_write(uint8_t wb)
{
    stm32_spi_send_byte(wb);
}

/**
		@brief: w5500 burst read wrapper
		@param[out]: *pBuf = pointer to read buffer
		@param[out]: len = len of read buffer
*/
void wizchip_burst_read(uint8_t* pBuf, uint16_t len)
{
    stm32_spi_read(pBuf, len);
}

/**
		@brief: w5500 burst write wrapper
		@param[in]: *pBuf = pointer to write buffer
		@param[in]: len = len of write buffer
*/
void wizchip_burst_write(uint8_t* pBuf, uint16_t len)
{
    stm32_spi_write(pBuf, len);
}

/**
		@brief: Init the W5500 chip and set the SPI functions
*/
void W5500_Chip_Init(void)
{
	//this 2 dimension array set the sockets to 2kb or 2048bytes of mem each.
	//W5500 has a total of 16kb for tx and rx, so each socket can only be allocated
	//max of 2kb.
    uint8_t memsize[2][8] = { { 2, 2, 2, 2, 2, 2, 2, 2 }, { 2, 2, 2, 2, 2, 2, 2, 2 } };

    //ensure the CSpin deselected
    wizchip_deselect();

    //register functions for W5500 lib
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
    reg_wizchip_spiburst_cbfunc(wizchip_burst_read, wizchip_burst_write);

    /** wizchip initialize*/
    if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1) {
        printf("WIZCHIP Initialized fail.\r\n");
        while (1);
    }
    /*
	do {
		if (ctlwizchip(CW_GET_PHYLINK, (void*) &tmp) == -1)
			printf("Unknown PHY Link stauts.\r\n");
	} while (tmp == PHY_LINK_OFF);
    */
}
