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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "uart_driver.h"

#define RX_FIFO_SIZE     128
#define UART_TX_BUF_SIZE 128

typedef struct {
    uint8_t data[RX_FIFO_SIZE];
    uint8_t max_size;
    uint8_t head;
    uint8_t tail;
    uint8_t size;
} uart_rx_fifo_t;

//global variables
extern UART_HandleTypeDef huart3;					//change uart handler to the right module
UART_HandleTypeDef *distal_huart = &huart3;

//private variables
static char tx_buf[UART_TX_BUF_SIZE];
static uint8_t tx_data_byte;
static uart_rx_fifo_t uart_rx_fifo; //FIFO system

static uint8_t ui8RxBuffer[1];
static volatile uint16_t tmpRxBufferSize = 0;
static volatile uint8_t ui8TxBusy = 0;

/*  @brief Custom interrupt callbacks for UART
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//check if it's the right UART modules
	if(huart->Instance == distal_huart->Instance)
	{
			//indicates there's tmp data for queuing into the FIFO system
			tmpRxBufferSize = sizeof(ui8RxBuffer);
	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//check if it's the right UART modules
	if(huart->Instance == distal_huart->Instance)
	{
			//indicate transmission completed
			ui8TxBusy = 0; 
	}
}
/*  end of Custom interrupt callbacks for UART
*/

/*  @brief Enable the UART char rx interrupt
		@retval: 0 = OK. Non-zero = error;
*/
uint8_t enable_rx_intr(void)
{
	//enable interrupt receiving of UART data
	if(HAL_UART_Receive_IT(distal_huart, ui8RxBuffer, sizeof(ui8RxBuffer)) == HAL_ERROR){
		Error_Handler();
		return HAL_ERROR;
	}
	
	return HAL_OK;
}
/*  @brief Check if the UART is in the midst of transmission
		@retval 1 = busy. 0 = free.
*/
uint8_t is_uart_busy(void)
{
	return ui8TxBusy;
}
/*  @brief Wrapper for sending 1 byte via UART
		@retval: 	HAL_OK       = 0x00U,
							HAL_ERROR    = 0x01U,
							HAL_BUSY     = 0x02U,
							HAL_TIMEOUT  = 0x03U
*/
uint8_t uart_send_byte(uint8_t data)
{
	tx_data_byte = data;
	ui8TxBusy  = 1;
	return (uint8_t)HAL_UART_Transmit_IT(distal_huart, &tx_data_byte, 1);
}

/*  @brief Wrapper for sending 1 byte via UART (blocking)
		@retval: 	HAL_OK       = 0x00U,
							HAL_ERROR    = 0x01U,
							HAL_BUSY     = 0x02U,
							HAL_TIMEOUT  = 0x03U
*/
uint8_t uart_send_byte_block(uint8_t data)
{
	uint8_t status;
	tx_data_byte = data;
	ui8TxBusy  = 1;
	status = HAL_UART_Transmit_IT(distal_huart, &tx_data_byte, 1);
	
	//wait while the tx is busy
	while(ui8TxBusy == 1)
	{
	}
	
	return status;
}

/*  @brief Wrapper for sending a byte array via UART (blocking)
		@retval: 	HAL_OK       = 0x00U,
							HAL_ERROR    = 0x01U,
							HAL_BUSY     = 0x02U,
							HAL_TIMEOUT  = 0x03U
*/
uint8_t uart_send_data_block(uint8_t data[], uint16_t len)
{
	uint16_t nn_index;
	uint8_t tmpStat;
	
	for(nn_index = 0; nn_index < len; nn_index++)
	{
		tmpStat = uart_send_byte_block(data[nn_index]);
		if(tmpStat != HAL_OK)
			return tmpStat;
	}
	
	return HAL_OK;
}

/*  @brief Enqueue the received UART data into the FIFO array
    
    @param[in] uint8_t byte = data to be queued into system
    @retval: 0 = success, 0xff = FIFO is full
*/
uint8_t rx_fifo_enqueue(uint8_t byte)
{
    //check if FIFO is full
    if(uart_rx_fifo.size == uart_rx_fifo.max_size)
        return 0xff;
    
    uart_rx_fifo.data[uart_rx_fifo.tail] = byte;
    uart_rx_fifo.size++;
    uart_rx_fifo.tail++;
    if(uart_rx_fifo.tail == uart_rx_fifo.max_size)
        uart_rx_fifo.tail = 0; //circle to the front
    
    return 0;
}

/*  @brief Dequeue the received UART data into the FIFO array
    
    @param[out] uint8_t *byte = pointer to received data
    @retval: 0 = success, 0xff = FIFO is empty
*/
uint8_t rx_fifo_dequeue(uint8_t *byte)
{
    //check if FIFO is empty
    if(uart_rx_fifo.size == 0)
        return 0xff;
    
    *byte = uart_rx_fifo.data[uart_rx_fifo.head];
    uart_rx_fifo.size--;
    uart_rx_fifo.head++;
    if(uart_rx_fifo.head == uart_rx_fifo.max_size)
        uart_rx_fifo.head = 0; //circle to the front
    
    return 0;
}

/*  @brief Read the Rx FIFO current size
    @retval: uint8_t size of FIFO
*/
uint8_t rx_fifo_size(void)
{
    return uart_rx_fifo.size;
}

/*  @brief Clear the FIFO
*/
void rx_fifo_clear(void)
{
    //clear RX FIFO buffer
    memset(&uart_rx_fifo, 0, sizeof(uart_rx_fifo));
    //set RX FIFO
    uart_rx_fifo.max_size = RX_FIFO_SIZE;
}

/*  @brief Uart receive data state. Keep this in a mainloop. 
            It needs to keep listening the UART for new data.
            If there's new data, it will store it into the FIFO.
    @retval: 0 = success, 0xff = FIFO is full.
*/
uint8_t uart_rx_data_state(void)
{
    uint8_t status = 0;
		uint16_t index = 0;
		    
		//queue the tmp buffer into the FIFO
		if(tmpRxBufferSize == sizeof(ui8RxBuffer))
		{
			//reset 
			tmpRxBufferSize = 0;
			
			for(index = 0; index < sizeof(ui8RxBuffer); index++)
			{
        status = rx_fifo_enqueue(ui8RxBuffer[index]);
        if(status != 0){
            return 0xff;
        }
			}
    }
    return 0;
}

/**@brief Function to emulate printf. It's a blocking call.
 */
void uart_printf( const char* format, ... )
{
	int ret_num;
	va_list ap;
	va_start(ap, format);
	ret_num = vsnprintf(tx_buf, sizeof(tx_buf), format, ap);
	
	ui8TxBusy  = 1;
	HAL_UART_Transmit_IT(distal_huart, (uint8_t *)tx_buf, ret_num);
	//wait while the tx is busy
	while(ui8TxBusy == 1)
	{
	}
	
	va_end(ap);
}

/*  @brief this is to start the uart system
    
    @retval: 0 = OK. Non-zero = error;
*/
uint8_t uart_sys_init(void)
{
		uint8_t tmp_stat;
    //enable UART module interrupts
    __HAL_UART_ENABLE_IT(distal_huart, UART_IT_TC);
		__HAL_UART_ENABLE_IT(distal_huart, UART_IT_RXNE);
	
		//enable rx interrrupt
		tmp_stat = enable_rx_intr();
		if(tmp_stat == HAL_ERROR)
		{
			return tmp_stat;
		}
	
    //clear RX FIFO buffer
    rx_fifo_clear();
        
    return HAL_OK;
}
/* [] END OF FILE */
