#ifndef BDBG_H
#define BDBG_H
#include <stdint.h>
#include "main.h"
#include "logger_types.h"
#include "bdbg_protocol.h"
#define BDBG_ADDR_LOWER 0x01U
#define BDBG_ADDR_UPPER 0x02U
typedef struct { uint8_t deviceAddr; BdbgRequest bdbgMode; uint16_t expectedLength; } BdbgReq;
/* BDBG_Data объявлен в bdbg_protocol.h. */
typedef enum { BDBG_RX_WAIT = 0, BDBG_RX_READY, BDBG_RX_UART_ERROR } BDBG_RxStatus;
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length);
uint8_t BDBG_BuildRequest(const BdbgReq *request, uint8_t output[BDBG_REQUEST_MAX_SIZE], uint8_t *outputLength);
void BDBG_Init(void);
uint8_t BDBG_StartRequest(UART_HandleTypeDef *uart, BdbgRequest command);
void BDBG_StopReceive(void);
BDBG_RxStatus BDBG_GetRxStatus(void);
uint8_t BDBG_ParseResponse(BdbgRequest command, BDBG_Data *data);
void BDBG_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size);
void BDBG_UartErrorCallback(UART_HandleTypeDef *uart);
#endif