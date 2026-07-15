#ifndef INC_WD_CONTROL_H_
#define INC_WD_CONTROL_H_

#include "stm32f4xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void wd_control_init(void);
int wd_control_usb_receive(const uint8_t *data, uint32_t len);
int wd_control_usb_tx_poll(void);
void wd_control_service_1khz(void);
void wd_control_note_control_task_fallback(void);
int wd_control_is_active(void);
int wd_control_can_rx_from_isr(CAN_HandleTypeDef *hcan, uint32_t rx_fifo);
void wd_control_can_tx_complete_from_isr(CAN_HandleTypeDef *hcan,
                                         uint32_t mailbox);
void wd_control_can_tx_abort_from_isr(CAN_HandleTypeDef *hcan,
                                      uint32_t mailbox);

#ifdef __cplusplus
}
#endif

#endif /* INC_WD_CONTROL_H_ */
