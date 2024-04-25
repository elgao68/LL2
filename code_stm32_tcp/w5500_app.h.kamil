/** ========================================

  Copyright Thesis Pte Ltd, 2018
  All Rights Reserved
  UNPUBLISHED, LICENSED SOFTWARE.

  CONFIDENTIAL AND PROPRIETARY INFORMATION
  WHICH IS THE PROPERTY OF THESIS PTE LTD.

  ========================================
  Version: 1.1
	Date: 15th May 2018
  Written by: Kenneth Er
  ========================================
	Application code for W5500 Ethernet chip

	======================================== */
	
#ifndef W5500_APP_H
#define W5500_APP_H
    
#include "stm32f4xx_hal.h"

/** 
		DATA_BUF_SIZE define for tx and rx app buffer.
		*Note* This does not affect the W5500 socket mem allocation
*/
#ifndef DATA_BUF_SIZE
	#define DATA_BUF_SIZE			2048
#endif

/**
		@brief: Set the W5500 chip 6bytes MAC Id
    @param[in]: id0 = Mac ID byte #1
		@param[in]: id1 = Mac ID byte #2
		@param[in]: id2 = Mac ID byte #3
		@param[in]: id3 = Mac ID byte #4
		@param[in]: id4 = Mac ID byte #5
		@param[in]: id5 = Mac ID byte #6
*/
void set_ethernet_w5500_mac(uint8_t id0, uint8_t id1, uint8_t id2, 
																uint8_t id3, uint8_t id4, uint8_t id5);
																
/**
	@brief: Function to display the network information
*/
void Display_Net_Conf(void);

/**
		@brief: The ethernet tcp send function
		@param[in]: *buf = pointer fo write buffer
		@param[in]: size = write length.
*/
void ethernet_w5500_send_data(uint8_t buf[], uint16_t size);

/**
		@brief: The ethernet tcp to reset send
*/
void ethernet_w5500_send_data_clear(void);

/**
		@brief: The ethernet tcp receive function
		@param[out]: *buf = pointer fo read buffer
		@retval: read length. 0 = no data
*/
uint16_t ethernet_w5500_rcv_data(uint8_t buf[]);

/**
		@brief: Check whether TCP is currently connected.
		@retval: 0 = not connected. 1 = connected
*/
uint8_t isTCPConnected(void);

/**
		@brief: The ethernet tcp check if there's new rx data
		@retval: 0 = no new data, 1 = new data
*/
uint8_t ethernet_w5500_new_rcv_data(void);

/**
		@brief: Reset the ethernet DHCP and network
*/
void ethernet_w5500_reset(void);
	
/**
		@brief: The ethernet state which check for DHCP and TCP state
*/
void ethernet_w5500_state(void);

/**  @brief: Init the etherner system
    
    @retval: 0 = OK. Non-zero = error;
*/
uint8_t ethernet_w5500_sys_init(void);

#endif
/** [] END OF FILE */
