/* ========================================
 
  Copyright Thesis Pte Ltd, 2017
  All Rights Reserved
  UNPUBLISHED, LICENSED SOFTWARE.
 
  CONFIDENTIAL AND PROPRIETARY INFORMATION
  WHICH IS THE PROPERTY OF THESIS PTE LTD.
 
  ========================================
  Version: 1.0
  Written by: Kenneth Er
  ========================================
	Basic SPI interface for STM32F429
  
	======================================== */

//============================================================================
// INCLUDES
//============================================================================
#include "stm32_spi.h"
#include "lowerlimb_config.h"

//============================================================================
// DEFINES
//============================================================================

//============================================================================
// PRIVATE VARIABLES
//============================================================================
extern SPI_HandleTypeDef hspi3;
static SPI_HandleTypeDef* spi_htm = &hspi3;


/*==============================================================================
 * @START
 * Function: stm32_spi_init
 * IN      : none
 * OUT     : None
 * PRE     : None
 * POST    :
 * RETURN  :
 *
 * INFO    : Init SPI module
 *
 * @END
 *============================================================================*/
void stm32_spi_init(void)
{
    //do nothing for now
}

/*==============================================================================
 * @START
 * Function: stm32_spi_send
 * IN      : value = byte to be sent
 * OUT     : None
 * PRE     : None
 * POST    :
 * RETURN  :
 *
 * INFO    : Send one byte by SPI
 *
 * @END
 *============================================================================*/
void stm32_spi_send_byte(uint8_t value)
{
    if(HAL_SPI_Transmit(spi_htm, &value, 1, 100) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/*==============================================================================
* @START
* Function: stm32_spi_recv
* IN      : None
* OUT     : None
* PRE     : None
* POST    :
* RETURN  : read byte
*
* INFO    : Recv one byte by SPI
*
* @END
*============================================================================*/
uint8_t stm32_spi_recv_byte(void)
{
    uint8_t ui8RxByte;
    if(HAL_SPI_Receive(spi_htm, &ui8RxByte, 1, 100) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
    return ui8RxByte;
}

/*==============================================================================
* @START
* Function: stm32_spi_write
* IN      : *data      = pointer to data array for writing
            length     = length of data
* OUT     : None
* PRE     : None
* POST    :
* RETURN  : always 0
*
* INFO    : write to
* @END
*============================================================================*/
int stm32_spi_write(uint8_t *data, uint16_t length)
{
    if(HAL_SPI_Transmit(spi_htm, data, length, 100) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
    return 0;
}

/*==============================================================================
* @START
* Function: stm32_spi_read
* IN      : *data      = pointer to data array for storing read data
            length     = length of data
* OUT     : None
* PRE     : None
* POST    :
* RETURN  : always 0
* @END
*============================================================================*/
int	stm32_spi_read(uint8_t *data, uint16_t length)
{
    if(HAL_SPI_Receive(spi_htm, data, length, 100) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
    return 0;
}

/*==============================================================================
 * @START
 * Function: stm32_spi_write_read
 * IN      : tx_data      = pointer to data array for writing
             tx_len     = length of data
						 rx_data      = pointer to data array for storing read data
             rx_len     = length of data
 * OUT     : None
 * PRE     : None
 * POST    :
 * RETURN  :
 * @END
 *============================================================================*/
int	stm32_spi_write_read(uint8_t tx_data[], uint16_t tx_len, uint8_t rx_data[], uint16_t rx_len)
{
    if(tx_len > 0) {
        stm32_spi_write(tx_data, tx_len);
    }

    if(rx_len > 0) {
        stm32_spi_read(rx_data, rx_len);
    }

    return 0;
}
