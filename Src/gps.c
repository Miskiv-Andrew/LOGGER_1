/* GPS LOGGER: USART3 + ReceiveToIdle DMA + кольцевой буфер + NMEA GNRMC.
 * Все функции разбора выполняются из основного цикла, НЕ из прерывания.
 */
#include "gps.h"
#include <string.h>

#define GPS_DMA_SIZE      512U
#define GPS_RING_SIZE    2048U
#define GPS_LINE_SIZE     512U
#define GPS_TIMEOUT_MS   5000U

extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;

static uint8_t dma_buffer[GPS_DMA_SIZE];
static uint8_t ring[GPS_RING_SIZE];
static volatile uint16_t head, tail;
static volatile uint8_t overflow, dma_failure, active;
static uint8_t line[GPS_LINE_SIZE];
static uint16_t line_length;
static uint32_t start_tick;

/* Сброс программных буферов допустим только при остановленном DMA. */
void GPS_Init(void)
{
    head = 0U;
    tail = 0U;
    overflow = 0U;
    dma_failure = 0U;
    active = 0U;
    line_length = 0U;
}

HAL_StatusTypeDef GPS_Start(void)
{
    HAL_StatusTypeDef status;
    /* Исключаем гонку между старым прерыванием и очисткой индексов. */
    active = 0U;
    (void)HAL_UART_AbortReceive(&huart3);
    GPS_Init();
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_buffer, GPS_DMA_SIZE);
    if (status != HAL_OK)
    {
        dma_failure = 1U;
        return status;
    }
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    start_tick = HAL_GetTick();
    active = 1U;
    return HAL_OK;
}

void GPS_Stop(void)
{
    active = 0U;
    (void)HAL_UART_AbortReceive(&huart3);
    /* При повторном старте GPS_Init очистит все буферы и флаги. */
}

/* Вызывается из общего HAL_UARTEx_RxEventCallback(main.c).
 * Никакого парсинга и блокирующих задержек внутри прерывания.
 */
void GPS_RxEventCallback(uint16_t size)
{
    uint16_t i;
    if (active == 0U)
        return;
    if (size > GPS_DMA_SIZE)
    {
        dma_failure = 1U;
        return;
    }
    for (i = 0U; i < size; ++i)
    {
        uint16_t next = (uint16_t)((head + 1U) % GPS_RING_SIZE);
        if (next == tail)
        {
            overflow = 1U;
            break;
        }
        ring[head] = dma_buffer[i];
        head = next;
    }
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_buffer, GPS_DMA_SIZE) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
    else
    {
        /* Ошибка DMA и переполнение кольца — разные причины отказа. */
        dma_failure = 1U;
    }
}

/* Вызывается при аппаратной ошибке USART3 / DMA. */
void GPS_NotifyUartError(void)
{
    if (active != 0U) dma_failure = 1U;
}

/* Преобразование двух десятичных символов с проверкой диапазона. */
static uint8_t TwoDigits(const char *s, uint8_t *result)
{
    if (s[0] < '0' || s[0] > '9' || s[1] < '0' || s[1] > '9')
        return 0U;
    *result = (uint8_t)((s[0] - '0') * 10 + s[1] - '0');
    return 1U;
}

static int8_t HexDigit(uint8_t c)
{
    if (c >= '0' && c <= '9') return (int8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (int8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (int8_t)(c - 'a' + 10);
    return -1;
}

/* ddmm.mmmm / dddmm.mmmm -> градусы. Никаких atof и локалей. */
static uint8_t ParseCoordinate(const char *s, uint8_t degree_digits,
                               char direction, float *value)
{
    uint32_t degrees = 0U;
    uint8_t minutes;
    uint32_t fraction = 0U, scale = 1U;
    uint8_t i, digits = 0U;
    if (strlen(s) < (size_t)(degree_digits + 4U)) return 0U;
    for (i = 0U; i < degree_digits; ++i)
    {
        if (s[i] < '0' || s[i] > '9') return 0U;
        degrees = degrees * 10U + (uint32_t)(s[i] - '0');
    }
    if (!TwoDigits(&s[degree_digits], &minutes))
    {
        /* TwoDigits требует uint8_t: используем отдельную переменную ниже. */
        return 0U;
    }
    if (s[degree_digits + 2U] != '.') return 0U;
    for (i = (uint8_t)(degree_digits + 3U); s[i] != '\0'; ++i)
    {
        if (s[i] < '0' || s[i] > '9' || digits >= 7U) return 0U;
        fraction = fraction * 10U + (uint32_t)(s[i] - '0');
        scale *= 10U;
        ++digits;
    }
    if (digits == 0U || minutes >= 60U) return 0U;
    if (degree_digits == 2U)
    {
        if ((direction != 'N' && direction != 'S') || degrees > 90U ||
            (degrees == 90U && (minutes != 0U || fraction != 0U))) return 0U;
    }
    else
    {
        if ((direction != 'E' && direction != 'W') || degrees > 180U ||
            (degrees == 180U && (minutes != 0U || fraction != 0U))) return 0U;
    }
    *value = (float)degrees + ((float)minutes + (float)fraction / (float)scale) / 60.0f;
    if (direction == 'S' || direction == 'W') *value = -*value;
    return 1U;
}

/* Возвращает 1 только для GNRMC со статусом A, корректной XOR и полями. */
static uint8_t ParseRMC(uint8_t *sentence, uint16_t length, GPS_Fix *fix)
{
    uint16_t i, star = 0U;
    uint8_t checksum = 0U, field_count = 0U, max_day;
    int8_t hi, lo;
    char *fields[12];
    if (length < 20U || sentence[0] != '$') return 0U;
    for (i = 1U; i < length; ++i)
    {
        if (sentence[i] == '*') { star = i; break; }
        checksum ^= sentence[i];
    }
    if (star == 0U || star + 3U != length) return 0U;
    hi = HexDigit(sentence[star + 1U]);
    lo = HexDigit(sentence[star + 2U]);
    if (hi < 0 || lo < 0 || checksum != (uint8_t)((hi << 4) | lo)) return 0U;
    sentence[star] = '\0';
    /* Делим только тело до '*'; лишние поля после даты допустимы. */
    fields[field_count++] = (char *)&sentence[1];
    for (i = 1U; i < star; ++i)
    {
        if (sentence[i] == ',')
        {
            sentence[i] = '\0';
            if (field_count < 12U) fields[field_count++] = (char *)&sentence[i + 1U];
        }
    }
    if (field_count < 10U || strcmp(fields[0], "GNRMC") != 0 ||
        strcmp(fields[2], "A") != 0 || strlen(fields[1]) < 6U ||
        strlen(fields[9]) != 6U || strlen(fields[4]) != 1U ||
        strlen(fields[6]) != 1U) return 0U;
    if (!TwoDigits(fields[1], &fix->hour) ||
        !TwoDigits(fields[1] + 2, &fix->minute) ||
        !TwoDigits(fields[1] + 4, &fix->second) ||
        fix->hour > 23U || fix->minute > 59U || fix->second > 59U) return 0U;
    /* После секунд допускается пустой остаток или дробная часть. */
    if (fields[1][6] != '\0' && fields[1][6] != '.') return 0U;
    if (!ParseCoordinate(fields[3], 2U, fields[4][0], &fix->latitude) ||
        !ParseCoordinate(fields[5], 3U, fields[6][0], &fix->longitude)) return 0U;
    if (!TwoDigits(fields[9], &fix->day) ||
        !TwoDigits(fields[9] + 2, &fix->month) ||
        !TwoDigits(fields[9] + 4, &fix->year) ||
        fix->month < 1U || fix->month > 12U) return 0U;
    max_day = 31U;
    if (fix->month == 4U || fix->month == 6U || fix->month == 9U || fix->month == 11U)
        max_day = 30U;
    if (fix->month == 2U)
    {
        /* Дата трактуется как 20YY. */
        max_day = (fix->year % 4U == 0U) ? 29U : 28U;
    }
    if (fix->day < 1U || fix->day > max_day) return 0U;
    return 1U;
}

GPS_Result GPS_Poll(GPS_Fix *fix)
{
    if (fix == NULL || active == 0U) return GPS_RESULT_RX_ERROR;
    if (overflow != 0U || dma_failure != 0U) return GPS_RESULT_RX_ERROR;
    /* Проверяем тайм-аут до разбора: отладочная задержка не должна
       увеличивать разрешённое время ожидания фиксации. */
    if ((uint32_t)(HAL_GetTick() - start_tick) >= GPS_TIMEOUT_MS)
        return GPS_RESULT_TIMEOUT;
    while (tail != head)
    {
        uint8_t c = ring[tail];
        tail = (uint16_t)((tail + 1U) % GPS_RING_SIZE);
        if (c == '$')
        {
            line_length = 0U;
            line[line_length++] = c;
        }
        else if (line_length != 0U)
        {
            if (c == '\n')
            {
                /* CRLF обязательно; CR в контрольную сумму не входит. */
                if (line_length > 0U && line[line_length - 1U] == '\r' &&
                    ParseRMC(line, (uint16_t)(line_length - 1U), fix))
                {
                    line_length = 0U;
                    return GPS_RESULT_FIX;
                }
                line_length = 0U;
            }
            else if (line_length < GPS_LINE_SIZE - 1U)
            {
                line[line_length++] = c;
            }
            else
            {
                /* Слишком длинная строка: ждём следующего '$'. */
                line_length = 0U;
            }
        }
    }
    return GPS_RESULT_WAIT;
}

HAL_StatusTypeDef GPS_SetRTC(const GPS_Fix *fix)
{
    RTC_TimeTypeDef time = {0};
    RTC_DateTypeDef date = {0};
    uint16_t y;
    uint8_t m, weekday;
    if (fix == NULL) return HAL_ERROR;
    time.Hours = fix->hour;
    time.Minutes = fix->minute;
    time.Seconds = fix->second;
    time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    time.StoreOperation = RTC_STOREOPERATION_RESET;
    date.Year = fix->year;
    date.Month = fix->month;
    date.Date = fix->day;
    /* Вычисление дня недели (1=понедельник ... 7=воскресенье). */
    y = (uint16_t)(2000U + fix->year);
    m = fix->month;
    if (m < 3U) { m = (uint8_t)(m + 12U); --y; }
    weekday = (uint8_t)((fix->day + 13U * (m + 1U) / 5U + y + y / 4U -
                          y / 100U + y / 400U) % 7U); /* 0=суббота */
    date.WeekDay = (uint8_t)(((weekday + 5U) % 7U) + 1U);
    if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) return HAL_ERROR;
    return HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
}
