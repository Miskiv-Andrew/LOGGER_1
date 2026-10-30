#ifndef BDBG_H
#define BDBG_H
#include <stdint.h>
#include "main.h"
#include "logger_types.h"
#include "bdbg_protocol.h"
#define BDBG_ADDR_LOWER 0x01U
#define BDBG_ADDR_UPPER 0x02U
typedef struct { uint8_t deviceAddr; BdbgRequest bdbgMode; uint16_t expectedLength; } BdbgReq;
typedef struct
{
    float paed_usv_h;
    uint8_t status;
    uint16_t temperature_raw;
    uint16_t acquisition_time;
    uint16_t cps;
    uint8_t test;
    uint8_t device_id;
    uint32_t serial_number;
    uint8_t year, month, version_major, version_minor;
    uint16_t spectrum[BDBG_SPECTRUM_CHANNELS];
} BDBG_Data;
typedef enum { BDBG_RX_WAIT = 0, BDBG_RX_READY, BDBG_RX_UART_ERROR } BDBG_RxStatus;
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length);
uint8_t BDBG_BuildRequest(const BdbgReq *request, uint8_t output[BDBG_REQUEST_MAX_SIZE], uint8_t *outputLength);
void BDBG_Init(void);
uint8_t BDBG_StartRequest(UART_HandleTypeDef *uart, BdbgRequest command);
void BDBG_StopReceive(void);
BDBG_RxStatus BDBG_GetRxStatus(void);
uint8_t BDBG_ParseResponse(BdbgRequest command, BDBG_Data *data);
void BDBG_RxCompleteCallback(UART_HandleTypeDef *uart);
void BDBG_UartErrorCallback(UART_HandleTypeDef *uart);
#endif
