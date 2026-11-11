#include "bdbg.h"
#include "bdbg_config.h"
#include <string.h>
#include <stddef.h>
/* Буфер в статической RAM: ответ 2076 байт и запас на лишние байты. */
static uint8_t rx_buffer[BDBG_SPECTRUM_RESPONSE_SIZE + BDBG_RX_MARGIN];
static UART_HandleTypeDef *active_uart = NULL;
static volatile BDBG_RxStatus rx_status = BDBG_RX_WAIT;
static uint16_t rx_expected = 0U;
/* Число байт, принятых с начала буфера. */
static volatile uint16_t rx_received = 0U;
/*
 * Управление RS-485:
 *   PC6 RE485 — разрешение приёма, активный уровень 0;
 *   PC7 DE485 — разрешение передачи, активный уровень 1.
 * PC6 = 1, PC7 = 0 — шина неактивна.
 */
static void BDBG_BusIdle(void)
{
    HAL_GPIO_WritePin(DE485_GPIO_Port, DE485_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RE485_GPIO_Port, RE485_Pin, GPIO_PIN_SET);
}
/* Передача: приём выключен, эхо собственного запроса не принимается. */
static void BDBG_BusTransmit(void)
{
    HAL_GPIO_WritePin(RE485_GPIO_Port, RE485_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DE485_GPIO_Port, DE485_Pin, GPIO_PIN_SET);
}
/* Приём ответа прибора. */
static void BDBG_BusReceive(void)
{
    HAL_GPIO_WritePin(DE485_GPIO_Port, DE485_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RE485_GPIO_Port, RE485_Pin, GPIO_PIN_RESET);
}
/*
 * Запуск приёма с позиции rx_received до конца буфера.
 * Событие приёма (HAL_UARTEx_RxEventCallback) наступает по паузе на линии (IDLE)
 * или при заполнении буфера.
 */
static uint8_t BDBG_StartReceive(UART_HandleTypeDef *uart)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(uart, &rx_buffer[rx_received],
                                     (uint16_t)(sizeof(rx_buffer) - rx_received)) != HAL_OK) return 0U;
    /* Событие половины буфера не используется. */
    __HAL_DMA_DISABLE_IT(uart->hdmarx, DMA_IT_HT);
    /*
     * Ошибки кадра и шума (FE, NE) не прерывают приём: искажённый байт
     * вне ответа отбрасывается поиском 55 AA, внутри ответа его отбросит CRC.
     * Иначе HAL останавливает DMA на первой же ошибке, и ответ теряется.
     */
    ATOMIC_CLEAR_BIT(uart->Instance->CR3, USART_CR3_EIE);
    return 1U;
}
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length)
{ return BDBG_ProtocolCalculateChecksum(data, length); }
uint8_t BDBG_BuildRequest(const BdbgReq *req, uint8_t out[BDBG_REQUEST_MAX_SIZE], uint8_t *n)
{ if (req == NULL) return 0U; return BDBG_ProtocolBuildRequest(req->deviceAddr, req->bdbgMode, out, n); }
void BDBG_Init(void)
{ active_uart = NULL; rx_expected = 0U; rx_received = 0U; rx_status = BDBG_RX_WAIT; memset(rx_buffer, 0, sizeof(rx_buffer)); BDBG_BusIdle(); }
void BDBG_StopReceive(void)
{
    if (active_uart != NULL) (void)HAL_UART_AbortReceive(active_uart);
    active_uart = NULL;
    /* Обмен завершён или прерван: шина в неактивное состояние. */
    BDBG_BusIdle();
}

uint8_t BDBG_StartRequest(UART_HandleTypeDef *uart, BdbgRequest command)
{
    uint8_t tx[BDBG_REQUEST_MAX_SIZE];
    uint8_t tx_length = 0U;
    if (uart == NULL) return 0U;
    if (!BDBG_ProtocolBuildRequest(BDBG_DEFAULT_ADDRESS, command, tx, &tx_length)) return 0U;
    BDBG_StopReceive();
    /* Длина ответа задаётся протоколом выбранной модели. */
    rx_expected = BDBG_ProtocolResponseSize(command);
    if ((rx_expected < 2U) || (rx_expected > sizeof(rx_buffer))) return 0U;
    rx_received = 0U;
    rx_status = BDBG_RX_WAIT;
    /*
     * Передача запроса. HAL_UART_Transmit() возвращается после
     * окончания передачи последнего байта (флаг TC).
     */
    BDBG_BusTransmit();
    if (HAL_UART_Transmit(uart, tx, tx_length, 100U) != HAL_OK)
    { BDBG_BusIdle(); rx_status = BDBG_RX_UART_ERROR; return 0U; }
    BDBG_BusReceive();
    /*
     * Приём запускается после передачи: прибор отвечает не раньше,
     * чем примет и обработает запрос.
     * Перед запуском сбрасываем флаги ошибок UART (ORE, NE, PE, FE)
     * и очищаем приёмник: байты, принятые во время передачи
     * (выход приёмника RS-485 отключён), и ответ на предыдущий запрос,
     * закрытый без ожидания (PAED_REQ при смене территории), отбрасываются.
     */
    __HAL_UART_CLEAR_FLAG(uart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(uart, UART_RXDATA_FLUSH_REQUEST);
    active_uart = uart;
    if (BDBG_StartReceive(uart) == 0U)
    { active_uart = NULL; BDBG_BusIdle(); rx_status = BDBG_RX_UART_ERROR; return 0U; }
    return 1U;
}

BDBG_RxStatus BDBG_GetRxStatus(void) { return rx_status; }
/*
 * Событие приёма UART4 (из HAL_UARTEx_RxEventCallback, контекст прерывания).
 * size — число байт, принятых с момента последнего запуска приёма.
 *
 * Ответ готов, если в буфере есть 55 AA и от него принято не меньше
 * rx_expected байт. Лишние байты до и после ответа допускаются.
 *
 * Если полного ответа ещё нет (например, перед ответом пришёл
 * одиночный ложный байт и линия замолчала), приём продолжается
 * с текущей позиции. Ответ, который не пришёл, завершит таймаут TIM3.
 */
void BDBG_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if ((uart != active_uart) || (rx_status != BDBG_RX_WAIT)) return;
    rx_received = (uint16_t)(rx_received + size);
    if (rx_received > sizeof(rx_buffer)) rx_received = (uint16_t)sizeof(rx_buffer);
    for (uint16_t i = 0U; ((uint32_t)i + rx_expected) <= rx_received; ++i)
    {
        if ((rx_buffer[i] == 0x55U) && (rx_buffer[i + 1U] == 0xAAU))
        { rx_status = BDBG_RX_READY; return; }
    }
    if ((rx_received >= sizeof(rx_buffer)) || (BDBG_StartReceive(uart) == 0U))
        rx_status = BDBG_RX_UART_ERROR;
}
void BDBG_UartErrorCallback(UART_HandleTypeDef *uart)
{ if (uart == active_uart) rx_status = BDBG_RX_UART_ERROR; }
/*
 * Разбор ответа выполняет протокол выбранной модели (bdbg_protocol_NN.c).
 * Перебираем позиции 55 AA, от которых принят полный ответ:
 * ложное 55 AA среди лишних байт отбросит проверка заголовка и CRC.
 */
uint8_t BDBG_ParseResponse(BdbgRequest command, BDBG_Data *data)
{
    if ((data == NULL) || (rx_status != BDBG_RX_READY)) return 0U;
    for (uint16_t i = 0U; ((uint32_t)i + rx_expected) <= rx_received; ++i)
    {
        if ((rx_buffer[i] == 0x55U) && (rx_buffer[i + 1U] == 0xAAU) &&
            (BDBG_ProtocolParseResponse(command, &rx_buffer[i], rx_expected, data) != 0U))
            return 1U;
    }
    return 0U;
}