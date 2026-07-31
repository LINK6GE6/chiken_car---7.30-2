#ifndef __UART3_TO_HTP_H__
#define __UART3_TO_HTP_H__

#include "main.h"

#define STM_SEND_CMD_ENCODER 0x01
#define STM_SEND_CMD_IMU 0x0a

void uart3_send_to_htp(uint8_t cmd, uint8_t *data, uint8_t len);

#endif
