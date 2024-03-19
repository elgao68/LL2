/* ========================================
 
  Copyright Thesis Pte Ltd, 2017
  All Rights Reserved
  UNPUBLISHED, LICENSED SOFTWARE.
 
  CONFIDENTIAL AND PROPRIETARY INFORMATION
  WHICH IS THE PROPERTY OF THESIS PTE LTD.
 
  ========================================
  Version: 1.1
	Date: 15th May 2018
  Written by: Kenneth Er
  ========================================
	Basic UART interface for STM32F429
	
	NOTE!
	- enable_rx_intr() needs to be added into 
		the stm32f4xx_it.c file inside the uart
		USARTx_IRQHandler().
  
	======================================== */
	
#ifndef UART_DRIVER_H
#define UART_DRIVER_H
    
#include "stm32f4xx_hal.h"

/*  @brief Enable the UART char rx interrupt
		@retval: 0 = OK. Non-zero = error;
*/
uint8_t enable_rx_intr(void);

/*  @brief Check if the UART is in the midst of transmission
		@retval 1 = busy. 0 = free.
*/
uint8_t is_uart_busy(void);

/*  @brief Wrapper for sending 1 byte via UART
		@retval: 	HAL_OK       = 0x00U,
							HAL_ERROR    = 0x01U,
							HAL_BUSY     = 0x02U,
							HAL_TIMEOUT  = 0x03U
*/
uint8_t uart_send_byte(uint8_t data);

/*  @brief Wrapper for sending 1 byte via UART (blocking)
		@retval: 	HAL_OK       = 0x00U,
							HAL_ERROR    = 0x01U,
							HAL_BUSY     = 0x02U,
							HAL_TIMEOUT  = 0x03U
*/
uint8_t uart_send_byte_block(uint8_t data);

/*  @brief Wrapper for sending a byte array via UART (blocking)
		@retval: 	HAL_OK       = 0x00U,
							HAL_ERROR    = 0x01U,
							HAL_BUSY     = 0x02U,
							HAL_TIMEOUT  = 0x03U
*/
uint8_t uart_send_data_block(uint8_t data[], uint16_t len);

/*  @brief Enqueue the received UART data into the FIFO array
    
    @param[in] uint8_t byte = data to be queued into system
    @retval: 0 = success, 0xff = FIFO is full
*/
uint8_t rx_fifo_enqueue(uint8_t byte);
    
/*  @brief Dequeue the received UART data into the FIFO array

    @param[out] uint8_t *byte = pointer to received data
    @retval: 0 = success, 0xff = FIFO is empty
*/
uint8_t rx_fifo_dequeue(uint8_t *byte);

/*  @brief Read the Rx FIFO current size
    @retval: uint8_t size of FIFO
*/
uint8_t rx_fifo_size(void);

/*  @brief Clear the FIFO
*/
void rx_fifo_clear(void);
    
/*  @brief Uart receive data state. Keep this in a mainloop. 
            It needs to keep listening the UART for new data.
            If there's new data, it will store it into the FIFO.
    @retval: 0 = success, 0xff = FIFO is full.
*/
uint8_t uart_rx_data_state(void);

/**@brief Function to emulate printf. It's a blocking call.
 */
void uart_printf( const char* format, ... );
    
/*  @brief this is to start the uart system
    
    @retval: 0 = OK. Non-zero = error;
*/
uint8_t uart_sys_init(void);
#endif
/* [] END OF FILE */
