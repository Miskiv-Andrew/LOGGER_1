#include "bdbg.h"
#include "bdbg_config.h"
#include <string.h>
#include <stddef.h>
/* Буфер в статической RAM: не расходуем стек на 2076 байт. */
static uint8_t rx_buffer[BDBG_SPECTRUM_RESPONSE_SIZE];
static UART_HandleTypeDef *active_uart = NULL;
static volatile BDBG_RxStatus rx_status = BDBG_RX_WAIT;
static uint16_t rx_expected = 0U;
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length)
{ return BDBG_ProtocolCalculateChecksum(data, length); }
uint8_t BDBG_BuildRequest(const BdbgReq *req, uint8_t out[BDBG_REQUEST_MAX_SIZE], uint8_t *n)
{ if (req == NULL) return 0U; return BDBG_ProtocolBuildRequest(req->deviceAddr, req->bdbgMode, out, n); }
void BDBG_Init(void)
{ active_uart = NULL; rx_expected = 0U; rx_status = BDBG_RX_WAIT; memset(rx_buffer, 0, sizeof(rx_buffer)); }
void BDBG_StopReceive(void)
{
    if (active_uart != NULL) (void)HAL_UART_AbortReceive(active_uart);
    active_uart = NULL;
}
uint8_t BDBG_StartRequest(UART_HandleTypeDef *uart, BdbgRequest command)
{
    uint8_t tx[BDBG_REQUEST_MAX_SIZE];
    uint8_t tx_length = 0U;
    if (uart == NULL) return 0U;
    if (!BDBG_ProtocolBuildRequest(BDBG_DEFAULT_ADDRESS, command, tx, &tx_length)) return 0U;
    BDBG_StopReceive();
    rx_expected = ((command == START_SPECTRE_REQ) || (command == GET_SPECTRE_REQ))
                    ? BDBG_SPECTRUM_RESPONSE_SIZE : ((command == PAED_REQ) ? 12U : 8U);
    rx_status = BDBG_RX_WAIT;
    active_uart = uart;
    /* DMA запускается ДО передачи запроса: исключаем потерю первых байтов ответа. */
    if (HAL_UART_Receive_DMA(uart, rx_buffer, rx_expected) != HAL_OK)
    { active_uart = NULL; rx_status = BDBG_RX_UART_ERROR; return 0U; }
    if (HAL_UART_Transmit(uart, tx, tx_length, 100U) != HAL_OK)
    { BDBG_StopReceive(); rx_status = BDBG_RX_UART_ERROR; return 0U; }
    return 1U;
}
BDBG_RxStatus BDBG_GetRxStatus(void) { return rx_status; }
void BDBG_RxCompleteCallback(UART_HandleTypeDef *uart)
{ if ((uart == active_uart) && (rx_status == BDBG_RX_WAIT)) rx_status = BDBG_RX_READY; }
void BDBG_UartErrorCallback(UART_HandleTypeDef *uart)
{ if (uart == active_uart) rx_status = BDBG_RX_UART_ERROR; }
static uint16_t read_u16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t read_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static float read_float(const uint8_t *p)
{ uint32_t raw = read_u32(p); float f; memcpy(&f, &raw, sizeof(f)); return f; }
uint8_t BDBG_ParseResponse(BdbgRequest command, BDBG_Data *data)
{
    if ((data == NULL) || (rx_status != BDBG_RX_READY)) return 0U;
    if ((rx_buffer[0] != 0x55U) || (rx_buffer[1] != 0xAAU) ||
        (rx_buffer[2] != 0x70U) || (rx_buffer[3] != BDBG_DEFAULT_ADDRESS)) return 0U;
    if (BDBG_ProtocolCalculateChecksum(rx_buffer, (uint16_t)(rx_expected - 1U)) != rx_buffer[rx_expected - 1U]) return 0U;
    if ((command == START_SPECTRE_REQ) || (command == GET_SPECTRE_REQ))
    {
        if ((rx_buffer[4] != 0x8BU) || (rx_buffer[5] != 0x00U)) return 0U;
        for (uint16_t i = 0U; i < BDBG_SPECTRUM_CHANNELS; ++i)
        {
            data->spectrum[i] = read_u16(&rx_buffer[6U + 2U * i]);
            if ((command == START_SPECTRE_REQ) && (data->spectrum[i] != 0U)) return 0U;
        }
        data->acquisition_time = read_u16(&rx_buffer[2054]);
        data->paed_usv_h = read_float(&rx_buffer[2056]);
        data->status = rx_buffer[2060]; data->test = rx_buffer[2061];
        data->temperature_raw = read_u16(&rx_buffer[2062]);
        data->cps = read_u16(&rx_buffer[2064]); data->device_id = rx_buffer[2066];
        data->serial_number = read_u32(&rx_buffer[2067]);
        data->year = rx_buffer[2071]; data->month = rx_buffer[2072];
        data->version_major = rx_buffer[2073]; data->version_minor = rx_buffer[2074];
        return 1U;
    }
    if ((command == PAED_REQ) && (rx_buffer[4] == 0x01U))
    { data->paed_usv_h = read_float(&rx_buffer[5]); data->status = rx_buffer[9]; return 1U; }
    if ((command == TEMP_REQ) && (rx_buffer[4] == 0x08U))
    { data->temperature_raw = read_u16(&rx_buffer[5]); return 1U; }
    return 0U;
}
