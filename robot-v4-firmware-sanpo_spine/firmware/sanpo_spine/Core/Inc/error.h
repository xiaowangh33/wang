/*
 * error.h
 *
 *  Created on: Jul 26, 2025
 *      Author: puwan
 */

#ifndef INC_ERROR_H_
#define INC_ERROR_H_

typedef enum _error_t
{
	ERR_PERIPHINIT = 0,
	ERR_CDCRX_MSGLEN,
	ERR_CDCTX_BUSY,
	ERR_SPITX_BUSY,
	ERR_CAN_TXFAIL,
	ERR_CANRXFIFO_OVERFLOW,
	ERR_FULLBUF_CDC2CANTX,
	ERR_FULLBUF_SPI2CANTX,
	ERR_FULLBUF_CDC2USARTTX,
	ERR_FULLBUF_SPI2USARTTX,
	ERR_FULLBUF_CDC2RS485TX,
	ERR_FULLBUF_SPI2RS485TX,
	ERR_FULLBUF_CDCRX,
	ERR_FULLBUF_CDCTX,
	ERR_FULLBUF_SPIRX,
	ERR_FULLBUF_SPITX,
	ERR_FULLBUF_USARTRX,
	ERR_FULLBUF_USARTTX,
	ERR_FULLBUF_ADCRX,
	ERR_FULLBUF_ADCTX,
	ERR_FULLBUF_RS485RX,
	ERR_FULLBUF_RS485TX,
	ERR_FULLBUF_I2CRX,
	ERR_FULLBUF_I2CTX,
	ERR_FULLBUF_CANRX,
	ERR_HAL_SPI_TransmitReceive_IT,
	ERR_HAL_USART_TransmitReceive_DMA,
	ERR_HAL_I2C_TransmitReceive_DMA,
	ERR_MAX
} error_t;


// Prototypes
void error_assert(error_t err);
void error_assert_int(int err);
uint32_t error_timestamp(error_t err);
uint8_t error_occurred(error_t err);
uint32_t error_reg(void);

#endif /* INC_ERROR_H_ */
