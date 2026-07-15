#ifndef INC_LOOP_FUNCTION_H_
#define INC_LOOP_FUNCTION_H_

#include "stm32f4xx_hal.h"
#include "usbd_cdc.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdint.h>

int cdc_rx_loop();
int cdc_tx_loop();
int spi_rx_loop(CAN_HandleTypeDef* phcan1, CAN_HandleTypeDef* phcan2, UART_HandleTypeDef* phrs485A, UART_HandleTypeDef* phrs485B);
int cana_tx_loop(CAN_HandleTypeDef* phcan1);
int canb_tx_loop(CAN_HandleTypeDef* phcan2);
int rs485a_tx_loop(UART_HandleTypeDef* phRS485A, GPIO_TypeDef* RS485A_Port, uint16_t RS485A_Pin);
int rs485b_tx_loop(UART_HandleTypeDef* phRS485B, GPIO_TypeDef* RS485B_Port, uint16_t RS485B_Pin);
int data_rx_loop();
int init_loop();
int init_task(SPI_HandleTypeDef *hspi,
		      GPIO_TypeDef* RS485A_Port,
			  uint16_t RS485A_Pin,
			  UART_HandleTypeDef *huart_RS485A,
			  DMA_HandleTypeDef *hdma_RS485A_rx,
			  GPIO_TypeDef* RS485B_Port,
			  uint16_t RS485B_Pin,
			  UART_HandleTypeDef *huart_RS485B,
			  DMA_HandleTypeDef *hdma_RS485B_rx,
			  USBD_HandleTypeDef* phUsbDeviceFS,
			  osThreadId_t* phcanaTXTask,
			  osThreadId_t* phcanbTXTask,
			  osThreadId_t* phrs485aTXTask,
			  osThreadId_t* phrs485bTXTask);
int update_usart(UART_HandleTypeDef* phusart, uint32_t bitrate, uint8_t format, uint8_t paritytype, uint8_t datatype);
int update_can(CAN_HandleTypeDef* phcan, uint32_t bitrate, uint32_t ActiveITs);
int remove_me();

void callback_can_rxfifo0_msgpending(CAN_HandleTypeDef *hcan);
void callback_can_rxfifo1_msgpending(CAN_HandleTypeDef *hcan);
void callback_spi_txrxcplt(SPI_HandleTypeDef *hspi);
void callback_spi_error(SPI_HandleTypeDef *hspi);
void callback_uartex_rxevent(UART_HandleTypeDef *huart, uint16_t Size, DMA_HandleTypeDef* hdma_usarta_rx, DMA_HandleTypeDef* hdma_usartb_rx);
void callback_uart_txcplt(UART_HandleTypeDef *huart, DMA_HandleTypeDef* hdma_usarta_rx, DMA_HandleTypeDef* hdma_usartb_rx);
void init_cdc_fs(USBD_HandleTypeDef* hUsbDeviceFS);
int8_t cdc_recv_fs(USBD_HandleTypeDef* hUsbDeviceFS, uint8_t* Buf, uint32_t *Len);
#endif /* INC_LOOP_FUNCTION_H_ */
