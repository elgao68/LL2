/* ========================================
 
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

#ifndef _W5500_SPI_H
#define _W5500_SPI_H

void  wizchip_select(void);
void  wizchip_deselect(void);
uint8_t wizchip_read(void);
void  wizchip_write(uint8_t wb);
void W5500_Chip_Init(void);

#endif
