#include "lidar.h"
#include <string.h>

/* Дескриптор LPUART1 создаётся в main.c кодом CubeMX. */
extern UART_HandleTypeDef hlpuart1;

/*
 * Буфер непосредственно заполняется DMA.
 * Пока DMA активен, разбирать его содержимое нельзя.
 */
static uint8_t lidar_rx_buffer[LIDAR_RX_SIZE];

/*
 * Эти признаки используются и в основном цикле, и в прерываниях.
 * volatile запрещает компилятору кэшировать их значения.
 */
volatile uint8_t lidar_info = 0U;
static volatile uint8_t lidar_rx_active = 0U;
static volatile uint8_t lidar_uart_error = 0U;

/* Результат последнего разбора. Не относится к формату ID22. */
static LIDAR_Data lidar_data;

void LIDAR_Init(void)
{
    /* Инициализация не запускает DMA или TIM3. */
    lidar_info = 0U;
    lidar_rx_active = 0U;
    lidar_uart_error = 0U;
    memset(lidar_rx_buffer, 0, sizeof(lidar_rx_buffer));
    memset(&lidar_data, 0, sizeof(lidar_data));
}

HAL_StatusTypeDef LIDAR_StartReceive(void)
{
    HAL_StatusTypeDef status;

    /*
     * Повторный запуск поверх действующего DMA недопустим.
     * Управление последовательностью опросов остаётся за стейт-машиной.
     */
    if (lidar_rx_active != 0U)
    {
        return HAL_BUSY;
    }

    /* Полностью очищаем состояние предыдущего опроса. */
    lidar_info = 0U;
    lidar_uart_error = 0U;
    memset(lidar_rx_buffer, 0, sizeof(lidar_rx_buffer));
    memset(&lidar_data, 0, sizeof(lidar_data));

    /*
     * DMA в CubeMX настроен как CIRCULAR, однако приём каждого
     * опроса завершается на первом полном заполнении 45 байтов.
     */
    lidar_rx_active = 1U;
    status = HAL_UART_Receive_DMA(&hlpuart1, lidar_rx_buffer, LIDAR_RX_SIZE);

    if (status != HAL_OK)
    {
        /* Не оставляем признак активного опроса при ошибке запуска. */
        lidar_rx_active = 0U;
        lidar_uart_error = 1U;
    }

    return status;
}

void LIDAR_StopReceive(void)
{
    /*
     * Сначала запрещаем учитывать новые callback как данные опроса.
     * Затем прекращаем DMA-приём. Сам LPUART1 остаётся включённым.
     */
    lidar_rx_active = 0U;
    (void)HAL_UART_AbortReceive(&hlpuart1);
}

void LIDAR_RxCpltCallback(void)
{
    /* Callback может поступить после отмены приёма: игнорируем его. */
    if (lidar_rx_active == 0U)
    {
        return;
    }

    /*
     * При циклическом DMA немедленно останавливаем приём, чтобы
     * следующие байты не перезаписали уже собранные 45 байтов.
     * Разбор протокола в обработчике прерывания не выполняется.
     */
    lidar_rx_active = 0U;
    if (HAL_UART_AbortReceive(&hlpuart1) != HAL_OK)
    {
        lidar_uart_error = 1U;
        return;
    }

    /* Публикуем готовый буфер только после остановки DMA. */
    lidar_info = 1U;
}

void LIDAR_NotifyUartError(void)
{
    /* Ошибка текущего сеанса: готового буфера больше нет. */
    if (lidar_rx_active == 0U)
    {
        return;
    }

    lidar_rx_active = 0U;
    lidar_info = 0U;
    lidar_uart_error = 1U;
    (void)HAL_UART_AbortReceive(&hlpuart1);
}

uint8_t LIDAR_HasUartError(void)
{
    return lidar_uart_error;
}

uint8_t LIDAR_Process(LIDAR_Data *out)
{
    uint16_t i;

    if (out == NULL)
    {
        return 0U;
    }

    /* Не выдаём вызывающему коду устаревшие данные. */
    memset(out, 0, sizeof(*out));
    memset(&lidar_data, 0, sizeof(lidar_data));

    /* Разбирать можно только полностью принятый, остановленный буфер. */
    if ((lidar_info == 0U) || (lidar_rx_active != 0U) ||
        (lidar_uart_error != 0U))
    {
        return 0U;
    }

    /* Признак готовности потребляется однократно. */
    lidar_info = 0U;

    /*
     * Сканируем все возможные позиции начала 9-байтового кадра.
     * В 45 байтах последняя допустимая позиция — индекс 36.
     */
    for (i = 0U; i <= (LIDAR_RX_SIZE - LIDAR_FRAME_SIZE); ++i)
    {
        uint16_t distance;
        uint16_t strength;
        uint8_t checksum = 0U;
        uint8_t j;

        /* Два стартовых байта протокола TFmini: 0x59 0x59. */
        if ((lidar_rx_buffer[i] != 0x59U) ||
            (lidar_rx_buffer[i + 1U] != 0x59U))
        {
            continue;
        }

        /* Сумма первых восьми байтов по модулю 256. */
        for (j = 0U; j < (LIDAR_FRAME_SIZE - 1U); ++j)
        {
            checksum = (uint8_t)(checksum + lidar_rx_buffer[i + j]);
        }

        if (checksum != lidar_rx_buffer[i + 8U])
        {
            continue;
        }

        /* Расстояние и мощность представлены в little-endian. */
        distance = (uint16_t)lidar_rx_buffer[i + 2U] |
                   ((uint16_t)lidar_rx_buffer[i + 3U] << 8U);
        strength = (uint16_t)lidar_rx_buffer[i + 4U] |
                   ((uint16_t)lidar_rx_buffer[i + 5U] << 8U);

        /* Проверка согласованных границ допустимых измерений. */
        if ((distance < LIDAR_MIN_DISTANCE_CM) ||
            (distance > LIDAR_MAX_DISTANCE_CM) ||
            (strength < LIDAR_MIN_STRENGTH) ||
            (strength == 0xFFFFU))
        {
            continue;
        }

        /* Первый валидный кадр завершает поиск. */
        lidar_data.distance_cm = distance;
        lidar_data.strength = strength;
        lidar_data.valid = 1U;
        *out = lidar_data;
        return 1U;
    }

    /* Все 45 байтов просмотрены, валидный кадр отсутствует. */
    return 0U;
}
