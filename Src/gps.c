/*
 * ============================================================================
 * gps.c
 * ============================================================================
 *
 * Работа LOGGER с GPS.
 *
 * GPS подключён к USART3 и после включения питания непрерывно передаёт
 * NMEA-предложения.
 *
 * Для LOGGER на текущем этапе используется только:
 *
 *      $GNRMC
 *
 * Приём:
 *
 *      USART3 + HAL_UARTEx_ReceiveToIdle_DMA() + DMA_CIRCULAR
 *
 * ВАЖНО:
 *
 * - разбор NMEA из прерывания НЕ выполняется;
 * - таймер ожидания GPS здесь НЕ реализуется;
 * - контроль 5 секунд выполняется отдельно через TIM3;
 * - если полный $GNRMC ещё не получен, полезный неполный хвост
 *   сохраняется до следующей порции данных.
 * ============================================================================
 */

#include "gps.h"

#include <string.h>


/*
 * ============================================================================
 * Внешние HAL-объекты.
 * ============================================================================
 */

extern UART_HandleTypeDef huart3;
extern RTC_HandleTypeDef hrtc;


/*
 * ============================================================================
 * Внутренние буферы GPS.
 * ============================================================================
 */

/*
 * DMA непосредственно записывает данные USART3 в этот буфер.
 *
 * DMA USART3 настроен в режиме DMA_CIRCULAR.
 */
static uint8_t gps_dma_rx_buffer[GPS_DMA_RX_BUFFER_SIZE];


/*
 * Накопительный буфер.
 *
 * Сюда из callback последовательно переносятся только НОВЫЕ байты
 * циклического DMA-буфера.
 *
 * Данные хранятся здесь до обработки GPS_Process().
 */
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];


/*
 * Количество байтов, которые в настоящий момент находятся
 * в gps_rx_buffer.
 */
static volatile uint16_t gps_rx_length = 0U;


/*
 * Последняя уже обработанная позиция циклического DMA-буфера.
 *
 * Нужна потому, что Size в callback при circular DMA обозначает
 * текущую позицию DMA, а не размер независимого нового пакета.
 */
static uint16_t gps_dma_old_pos = 0U;


/*
 * Флаг активности текущего сеанса приёма GPS.
 *
 * 0 - приём GPS не активен;
 * 1 - приём GPS запущен.
 */
static volatile uint8_t gps_rx_active = 0U;


/*
 * Флаг ошибки USART/DMA или переполнения накопительного буфера.
 *
 * Обработка ошибки выполняется из основной логики, а не из ISR.
 */
static volatile uint8_t gps_rx_error = 0U;


/*
 * Флаг, предусмотренный стейт-машиной.
 *
 * Устанавливается при поступлении новых данных USART3.
 * Проверяется в GPS_PROC.
 */
volatile uint8_t gps_info = 0U;


/*
 * ============================================================================
 * Вспомогательные функции.
 * ============================================================================
 */


/*
 * Преобразование одного HEX-символа контрольной суммы NMEA.
 *
 * Возвращает:
 *
 * 0...15 - корректная HEX-цифра;
 * -1     - некорректный символ.
 */
static int8_t GPS_HexDigit(uint8_t c)
{
    if ((c >= '0') && (c <= '9'))
    {
        return (int8_t)(c - '0');
    }

    if ((c >= 'A') && (c <= 'F'))
    {
        return (int8_t)(c - 'A' + 10);
    }

    if ((c >= 'a') && (c <= 'f'))
    {
        return (int8_t)(c - 'a' + 10);
    }

    return -1;
}


/*
 * Преобразование двух десятичных символов в число.
 *
 * Например:
 *
 * "17" -> 17
 *
 * Возвращает:
 *
 * 1 - успешно;
 * 0 - символы не являются двумя десятичными цифрами.
 */
static uint8_t GPS_ParseTwoDigits(const char *text, uint8_t *value)
{
    if ((text == NULL) || (value == NULL))
    {
        return 0U;
    }

    if ((text[0] < '0') || (text[0] > '9') ||
        (text[1] < '0') || (text[1] > '9'))
    {
        return 0U;
    }

    *value = (uint8_t)(((text[0] - '0') * 10) +
                       (text[1] - '0'));

    return 1U;
}


/*
 * Преобразование координаты NMEA в десятичные градусы.
 *
 * Для широты:
 *
 *      ddmm.mmmm
 *
 * Для долготы:
 *
 *      dddmm.mmmm
 *
 * degree_digits:
 *
 *      2 - широта;
 *      3 - долгота.
 *
 * direction:
 *
 *      N / S / E / W.
 */
static uint8_t GPS_ParseCoordinate(const char *text,
                                   uint8_t degree_digits,
                                   char direction,
                                   float *coordinate)
{
    uint32_t degrees = 0U;
    uint32_t fraction = 0U;
    uint32_t fraction_scale = 1U;

    uint8_t minutes = 0U;
    uint8_t fraction_digits = 0U;
    uint16_t index;


    if ((text == NULL) || (coordinate == NULL))
    {
        return 0U;
    }


    /*
     * Считываем градусы.
     */
    for (index = 0U; index < degree_digits; ++index)
    {
        if ((text[index] < '0') || (text[index] > '9'))
        {
            return 0U;
        }

        degrees = (degrees * 10U) +
                  (uint32_t)(text[index] - '0');
    }


    /*
     * После градусов обязательно должны находиться две цифры минут.
     */
    if (GPS_ParseTwoDigits(&text[degree_digits], &minutes) == 0U)
    {
        return 0U;
    }


    /*
     * После целой части минут ожидаем десятичную точку.
     */
    if (text[degree_digits + 2U] != '.')
    {
        return 0U;
    }


    /*
     * Разбираем дробную часть минут.
     */
    index = (uint16_t)(degree_digits + 3U);

    while (text[index] != '\0')
    {
        if ((text[index] < '0') || (text[index] > '9'))
        {
            return 0U;
        }

        /*
         * Ограничиваем количество цифр дробной части.
         * Для наших координат такой точности более чем достаточно.
         */
        if (fraction_digits >= 7U)
        {
            return 0U;
        }

        fraction =
            (fraction * 10U) +
            (uint32_t)(text[index] - '0');

        fraction_scale *= 10U;

        ++fraction_digits;
        ++index;
    }


    if ((fraction_digits == 0U) || (minutes >= 60U))
    {
        return 0U;
    }


    /*
     * Проверяем физически допустимый диапазон широты.
     */
    if (degree_digits == 2U)
    {
        if ((direction != 'N') && (direction != 'S'))
        {
            return 0U;
        }

        if (degrees > 90U)
        {
            return 0U;
        }

        if ((degrees == 90U) &&
            ((minutes != 0U) || (fraction != 0U)))
        {
            return 0U;
        }
    }

    /*
     * Проверяем физически допустимый диапазон долготы.
     */
    else if (degree_digits == 3U)
    {
        if ((direction != 'E') && (direction != 'W'))
        {
            return 0U;
        }

        if (degrees > 180U)
        {
            return 0U;
        }

        if ((degrees == 180U) &&
            ((minutes != 0U) || (fraction != 0U)))
        {
            return 0U;
        }
    }
    else
    {
        return 0U;
    }


    /*
     * NMEA:
     *
     *      градусы + минуты / 60
     */
    *coordinate =
        (float)degrees +
        (((float)minutes +
          ((float)fraction / (float)fraction_scale)) / 60.0f);


    /*
     * Южная широта и западная долгота имеют отрицательный знак.
     */
    if ((direction == 'S') || (direction == 'W'))
    {
        *coordinate = -*coordinate;
    }


    return 1U;
}


/*
 * Проверка календарной даты, полученной из RMC.
 *
 * Год трактуется как:
 *
 *      20YY
 */
static uint8_t GPS_CheckDate(uint8_t day,
                             uint8_t month,
                             uint8_t year)
{
    uint8_t max_day;


    if ((month < 1U) || (month > 12U))
    {
        return 0U;
    }


    max_day = 31U;


    if ((month == 4U) ||
        (month == 6U) ||
        (month == 9U) ||
        (month == 11U))
    {
        max_day = 30U;
    }
    else if (month == 2U)
    {
        /*
         * Для диапазона 2000...2099 достаточно проверки кратности 4.
         */
        if ((year % 4U) == 0U)
        {
            max_day = 29U;
        }
        else
        {
            max_day = 28U;
        }
    }


    if ((day < 1U) || (day > max_day))
    {
        return 0U;
    }


    return 1U;
}


/*
 * Разбор одного уже полностью полученного NMEA-предложения.
 *
 * sentence начинается с '$'.
 *
 * length НЕ включает CR/LF.
 *
 * Функция принимает только:
 *
 *      $GNRMC
 *
 * Обязательные условия:
 *
 * - корректная NMEA XOR checksum;
 * - тип GNRMC;
 * - статус A;
 * - корректное UTC-время;
 * - корректная широта;
 * - корректная долгота;
 * - корректная дата.
 */
static uint8_t GPS_ParseGNRMC(uint8_t *sentence,
                              uint16_t length,
                              GPS_Fix *fix)
{
    uint16_t index;
    uint16_t star_position = 0U;

    uint8_t checksum = 0U;
    uint8_t field_count = 0U;

    int8_t checksum_hi;
    int8_t checksum_lo;

    char *fields[12];


    if ((sentence == NULL) || (fix == NULL))
    {
        return 0U;
    }
		
		if ((length >= 6U) && (memcmp(sentence, "$GNRMC", 6U) == 0))
		{
				__NOP();  // Breakpoint ставим здесь
		}		


    /*
     * Слишком короткая строка заведомо не может содержать нужный RMC.
     */
    if ((length < 20U) || (sentence[0] != '$'))
    {
        return 0U;
    }


    /*
     * Контрольная сумма NMEA считается XOR всех символов
     * между '$' и '*'.
     */
    for (index = 1U; index < length; ++index)
    {
        if (sentence[index] == '*')
        {
            star_position = index;
            break;
        }

        checksum ^= sentence[index];
    }


    /*
     * После '*' должны находиться ровно две HEX-цифры checksum.
     */
    if ((star_position == 0U) ||
        ((star_position + 3U) != length))
    {
        return 0U;
    }


    checksum_hi = GPS_HexDigit(sentence[star_position + 1U]);
    checksum_lo = GPS_HexDigit(sentence[star_position + 2U]);


    if ((checksum_hi < 0) || (checksum_lo < 0))
    {
        return 0U;
    }


    if (checksum !=
        (uint8_t)(((uint8_t)checksum_hi << 4U) |
                  (uint8_t)checksum_lo))
    {
        return 0U;
    }


    /*
     * Заменяем '*' на конец строки.
     *
     * После этого можно безопасно разделить тело RMC по запятым.
     */
    sentence[star_position] = '\0';


    /*
     * Первое поле начинается сразу после '$'.
     */
    fields[field_count++] = (char *)&sentence[1];


    /*
     * Разделяем предложение на поля.
     */
    for (index = 1U; index < star_position; ++index)
    {
        if (sentence[index] == ',')
        {
            sentence[index] = '\0';

            if (field_count < 12U)
            {
                fields[field_count++] =
                    (char *)&sentence[index + 1U];
            }
        }
    }


    /*
     * Для интересующих нас данных требуется минимум:
     *
     * 0 - GNRMC
     * 1 - UTC time
     * 2 - status
     * 3 - latitude
     * 4 - N/S
     * 5 - longitude
     * 6 - E/W
     * 7 - speed
     * 8 - course
     * 9 - date
     */
    if (field_count < 10U)
    {
        return 0U;
    }


    /*
     * Работаем только с GNRMC.
     */
    if (strcmp(fields[0], "GNRMC") != 0)
    {
        return 0U;
    }


    /*
     * Статус A означает валидные навигационные данные.
     */
    if (strcmp(fields[2], "A") != 0)
    {
        return 0U;
    }


    /*
     * Поля направления должны содержать ровно один символ.
     */
    if ((strlen(fields[4]) != 1U) ||
        (strlen(fields[6]) != 1U))
    {
        return 0U;
    }


    /*
     * Время должно содержать как минимум hhmmss.
     *
     * После секунд может присутствовать дробная часть.
     */
    if (strlen(fields[1]) < 6U)
    {
        return 0U;
    }


    if ((GPS_ParseTwoDigits(&fields[1][0], &fix->hour) == 0U) ||
        (GPS_ParseTwoDigits(&fields[1][2], &fix->minute) == 0U) ||
        (GPS_ParseTwoDigits(&fields[1][4], &fix->second) == 0U))
    {
        return 0U;
    }


    if ((fix->hour > 23U) ||
        (fix->minute > 59U) ||
        (fix->second > 59U))
    {
        return 0U;
    }


    /*
     * После hhmmss допускается:
     *
     * - конец поля;
     * - точка и дробная часть секунды.
     */
    if ((fields[1][6] != '\0') &&
        (fields[1][6] != '.'))
    {
        return 0U;
    }


    /*
     * Широта.
     */
    if (GPS_ParseCoordinate(fields[3],
                            2U,
                            fields[4][0],
                            &fix->latitude) == 0U)
    {
        return 0U;
    }


    /*
     * Долгота.
     */
    if (GPS_ParseCoordinate(fields[5],
                            3U,
                            fields[6][0],
                            &fix->longitude) == 0U)
    {
        return 0U;
    }


    /*
     * Дата RMC имеет формат:
     *
     *      ddmmyy
     */
    if (strlen(fields[9]) != 6U)
    {
        return 0U;
    }


    if ((GPS_ParseTwoDigits(&fields[9][0], &fix->day) == 0U) ||
        (GPS_ParseTwoDigits(&fields[9][2], &fix->month) == 0U) ||
        (GPS_ParseTwoDigits(&fields[9][4], &fix->year) == 0U))
    {
        return 0U;
    }


    if (GPS_CheckDate(fix->day,
                      fix->month,
                      fix->year) == 0U)
    {
        return 0U;
    }


    return 1U;
}


/*
 * ============================================================================
 * GPS_Init()
 * ============================================================================
 */
void GPS_Init(void)
{
    gps_rx_length = 0U;
    gps_dma_old_pos = 0U;

    gps_rx_active = 0U;
    gps_rx_error = 0U;

    gps_info = 0U;

    memset(gps_dma_rx_buffer, 0, sizeof(gps_dma_rx_buffer));
    memset(gps_rx_buffer, 0, sizeof(gps_rx_buffer));
}


/*
 * ============================================================================
 * GPS_Clear()
 * ============================================================================
 *
 * Вызывать при остановленном GPS DMA либо перед запуском нового сеанса.
 */
void GPS_Clear(void)
{
    gps_rx_length = 0U;
    gps_dma_old_pos = 0U;

    gps_rx_error = 0U;
    gps_info = 0U;

    memset(gps_dma_rx_buffer, 0, sizeof(gps_dma_rx_buffer));
    memset(gps_rx_buffer, 0, sizeof(gps_rx_buffer));
}


/*
 * ============================================================================
 * GPS_StartReceive()
 * ============================================================================
 */
HAL_StatusTypeDef GPS_StartReceive(void)
{
    HAL_StatusTypeDef status;


    /*
     * На время подготовки нового сеанса callback не должен
     * воспринимать данные как действующие.
     */
    gps_rx_active = 0U;


    /*
     * На случай, если от предыдущего сеанса USART3 DMA ещё оставался
     * активным, прекращаем старый приём.
     */
    (void)HAL_UART_AbortReceive(&huart3);


    /*
     * Новый сеанс начинается с чистого программного состояния.
     */
    GPS_Clear();


    /*
     * Запускаем ReceiveToIdle DMA.
     *
     * DMA настроен CubeMX как CIRCULAR.
     */
    status =
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, gps_dma_rx_buffer,GPS_DMA_RX_BUFFER_SIZE);


    if (status != HAL_OK)
    {
        gps_rx_error = 1U;
        return status;
    }


    /*
     * Half Transfer для GPS нам не требуется.
     *
     * Получать дополнительное событие при заполнении половины
     * DMA-буфера нет необходимости.
     */
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);


    /*
     * С этого момента callback USART3 может принимать данные.
     */
    gps_rx_active = 1U;


    return HAL_OK;
}


/*
 * ============================================================================
 * GPS_StopReceive()
 * ============================================================================
 */
void GPS_StopReceive(void)
{
    /*
     * Сначала запрещаем обработку новых callback как действующих GPS-данных.
     */
    gps_rx_active = 0U;


    /*
     * Останавливаем текущий DMA-приём USART3.
     */
    (void)HAL_UART_AbortReceive(&huart3);
}


/*
 * ============================================================================
 * GPS_RxEventCallback()
 * ============================================================================
 *
 * Выполняется из контекста HAL UART callback.
 *
 * Здесь только переносим НОВЫЕ байты circular DMA в накопительный буфер.
 */
void GPS_RxEventCallback(uint16_t size)
{
    uint16_t new_data_length = 0U;


    /*
     * Если GPS сейчас не используется, событие игнорируем.
     */
    if (gps_rx_active == 0U)
    {
        return;
    }


    /*
     * Size не может выходить за размер DMA-буфера.
     */
    if (size > GPS_DMA_RX_BUFFER_SIZE)
    {
        gps_rx_error = 1U;
        return;
    }


    /*
     * ------------------------------------------------------------------------
     * СЛУЧАЙ 1.
     *
     * DMA продвинулся вперёд без перехода через конец буфера.
     *
     * Например:
     *
     * old_pos = 100
     * size    = 180
     *
     * Новые данные:
     *
     * [100 ... 179]
     * ------------------------------------------------------------------------
     */
    if (size > gps_dma_old_pos)
    {
        new_data_length =
            (uint16_t)(size - gps_dma_old_pos);


        /*
         * Нельзя частично записывать новую порцию.
         *
         * Если она целиком не помещается, фиксируем ошибку.
         */
        if (((uint32_t)gps_rx_length +
             (uint32_t)new_data_length) <= GPS_RX_BUFFER_SIZE)
        {
            memcpy(&gps_rx_buffer[gps_rx_length],
                   &gps_dma_rx_buffer[gps_dma_old_pos],
                   new_data_length);

            gps_rx_length =
                (uint16_t)(gps_rx_length + new_data_length);
        }
        else
        {
            gps_rx_error = 1U;
        }
    }


    /*
     * ------------------------------------------------------------------------
     * СЛУЧАЙ 2.
     *
     * DMA перешёл через конец циклического буфера.
     *
     * Например:
     *
     * old_pos = 680
     * size    = 20
     *
     * Новые данные находятся:
     *
     * [680 ... 699]
     *
     * и
     *
     * [0 ... 19]
     * ------------------------------------------------------------------------
     */
    else if (size < gps_dma_old_pos)
    {
        uint16_t tail_length;
        uint16_t head_length;


        tail_length =
            (uint16_t)(GPS_DMA_RX_BUFFER_SIZE -
                       gps_dma_old_pos);

        head_length = size;

        new_data_length =
            (uint16_t)(tail_length + head_length);


        /*
         * Проверяем наличие места сразу под оба участка.
         */
        if (((uint32_t)gps_rx_length +
             (uint32_t)new_data_length) <= GPS_RX_BUFFER_SIZE)
        {
            /*
             * Хвост DMA-буфера.
             */
            if (tail_length > 0U)
            {
                memcpy(&gps_rx_buffer[gps_rx_length],
                       &gps_dma_rx_buffer[gps_dma_old_pos],
                       tail_length);

                gps_rx_length =
                    (uint16_t)(gps_rx_length + tail_length);
            }


            /*
             * Начало DMA-буфера после wrap-around.
             */
            if (head_length > 0U)
            {
                memcpy(&gps_rx_buffer[gps_rx_length],
                       &gps_dma_rx_buffer[0],
                       head_length);

                gps_rx_length =
                    (uint16_t)(gps_rx_length + head_length);
            }
        }
        else
        {
            gps_rx_error = 1U;
        }
    }


    /*
     * HAL может сообщить Size == размеру DMA-буфера.
     *
     * После этого следующая физическая позиция circular DMA — 0.
     */
    if (size >= GPS_DMA_RX_BUFFER_SIZE)
    {
        gps_dma_old_pos = 0U;
    }
    else
    {
        gps_dma_old_pos = size;
    }


    /*
     * Флаг стейт-машины поднимаем только если действительно
     * были обнаружены новые данные.
     */
    if ((new_data_length > 0U) &&
        (gps_rx_error == 0U))
    {
        gps_info = 1U;
    }


    /*
     * ВАЖНО:
     *
     * HAL_UARTEx_ReceiveToIdle_DMA() здесь повторно НЕ вызывается.
     *
     * USART3 DMA работает в режиме DMA_CIRCULAR и после события
     * продолжает приём самостоятельно.
     */
}


/*
 * ============================================================================
 * GPS_NotifyUartError()
 * ============================================================================
 */
void GPS_NotifyUartError(void)
{
    if (gps_rx_active != 0U)
    {
        gps_rx_error = 1U;
    }
}


/*
 * ============================================================================
 * GPS_HasRxError()
 * ============================================================================
 */
uint8_t GPS_HasRxError(void)
{
    if (gps_rx_error != 0U)
    {
        return 1U;
    }

    return 0U;
}


/*
 * ============================================================================
 * GPS_Process()
 * ============================================================================
 *
 * Ищем в накопленных данных полный корректный $GNRMC.
 *
 * Функция не предполагает, что gps_rx_buffer начинается с '$'.
 * Приём GPS может быть включён посреди любого NMEA-предложения.
 */
uint8_t GPS_Process(GPS_Fix *fix)
{
    uint16_t search_position = 0U;


    if (fix == NULL)
    {
        return 0U;
    }


    /*
     * Последовательно ищем начало NMEA-предложения.
     */
    while (search_position < gps_rx_length)
    {
        uint16_t start;
        uint16_t end;
        uint16_t sentence_length;

        uint8_t sentence[GPS_DMA_RX_BUFFER_SIZE];


        /*
         * Ищем '$'.
         */
        while ((search_position < gps_rx_length) &&
               (gps_rx_buffer[search_position] != '$'))
        {
            ++search_position;
        }


        /*
         * '$' пока вообще не получен.
         *
         * Эти байты не могут стать частью будущего NMEA-предложения.
         */
        if (search_position >= gps_rx_length)
        {
            gps_rx_length = 0U;
            return 0U;
        }


        start = search_position;


        /*
         * Ищем окончание строки '\n'.
         */
        end = start;

        while ((end < gps_rx_length) &&
               (gps_rx_buffer[end] != '\n'))
        {
            ++end;
        }


        /*
         * Полная строка ещё не получена.
         *
         * Всё до найденного '$' уже не нужно.
         * Но сам '$' и весь неполный хвост ОБЯЗАТЕЛЬНО сохраняем.
         */
        if (end >= gps_rx_length)
        {
            if (start > 0U)
            {
                uint16_t remaining =
                    (uint16_t)(gps_rx_length - start);

                memmove(&gps_rx_buffer[0],
                        &gps_rx_buffer[start],
                        remaining);

                gps_rx_length = remaining;
            }

            return 0U;
        }


        /*
         * Перед '\n' обязательно ожидаем '\r'.
         */
        if ((end <= start) ||
            (gps_rx_buffer[end - 1U] != '\r'))
        {
            /*
             * Это не корректная NMEA-строка.
             * Продолжаем поиск со следующего байта.
             */
            search_position = (uint16_t)(start + 1U);
            continue;
        }


        /*
         * Длина предложения:
         *
         * от '$' до символа перед '\r'.
         *
         * CR/LF в parser не передаём.
         */
        sentence_length =
            (uint16_t)((end - 1U) - start);


        /*
         * Защита локального рабочего буфера.
         */
        if ((sentence_length == 0U) ||
            (sentence_length >= sizeof(sentence)))
        {
            search_position = (uint16_t)(end + 1U);
            continue;
        }


        /*
         * Копируем строку в отдельный рабочий буфер,
         * поскольку parser заменяет '*' и ',' на '\0'.
         *
         * Исходный накопительный буфер при разборе не изменяется.
         */
        memcpy(sentence,
               &gps_rx_buffer[start],
               sentence_length);

        sentence[sentence_length] = '\0';


        /*
         * Пытаемся разобрать именно GNRMC.
         */
        if (GPS_ParseGNRMC(sentence, sentence_length, fix) != 0U)
        {
            uint16_t consumed;
            uint16_t remaining;


            /*
             * Валидный пакет найден.
             *
             * Удаляем из накопительного буфера всё вплоть до конца
             * найденной строки включительно.
             */
            consumed = (uint16_t)(end + 1U);

            remaining =
                (uint16_t)(gps_rx_length - consumed);


            if (remaining > 0U)
            {
                memmove(&gps_rx_buffer[0],
                        &gps_rx_buffer[consumed],
                        remaining);
            }


            gps_rx_length = remaining;


            return 1U;
        }


        /*
         * Текущая полная строка не является валидным GNRMC.
         *
         * Это может быть:
         *
         * - GGA;
         * - GSA;
         * - GSV;
         * - RMC со статусом V;
         * - RMC с неправильной checksum;
         * - повреждённое предложение.
         *
         * Продолжаем искать следующую строку в уже накопленных данных.
         */
        search_position = (uint16_t)(end + 1U);
    }


    /*
     * Мы просмотрели все полные строки и не получили валидный GNRMC.
     *
     * Если после последней полной строки ничего полезного не осталось,
     * очищаем уже просмотренные данные.
     *
     * Неполный NMEA-хвост обрабатывается отдельной веткой выше и здесь
     * не теряется.
     */
    gps_rx_length = 0U;


    return 0U;
}


/*
 * ============================================================================
 * GPS_CheckSpoofing()
 * ============================================================================
 *
 * ВРЕМЕННАЯ ЗАГЛУШКА.
 *
 * Стейт-машина требует в MODE_EXPECT и MODE_WORK:
 *
 * - хранить предыдущую валидную координату;
 * - вычислять расстояние до новой координаты;
 * - отличать нормальное перемещение от нереального скачка;
 * - при нереальном скачке не принимать новую координату как валидную;
 * - вести счётчик подозрений на спуфинг;
 * - после трёх подтверждений сообщить об ошибке и перейти в MODE_ERROR.
 *
 * Пока численные пороги и окончательный алгоритм не определены.
 *
 * Поэтому НИКАКОГО придуманного алгоритма здесь сейчас нет.
 *
 * Временное значение 0 означает:
 *
 *      "спуфинг данной заглушкой не выявлен".
 */
uint8_t GPS_CheckSpoofing(const GPS_Fix *fix)
{
    (void)fix;

    return 0U;
}


/*
 * ============================================================================
 * GPS_CheckReconArea()
 * ============================================================================
 *
 * ВРЕМЕННАЯ ЗАГЛУШКА.
 *
 * Стейт-машина требует определять, находится ли текущая координата
 * внутри области разведки.
 *
 * На данный момент не определены:
 *
 * - формат задания области;
 * - координаты её границ;
 * - алгоритм проверки.
 *
 * Поэтому здесь намеренно НЕ реализуется никакая геометрия.
 *
 * Временное значение 0 означает:
 *
 *      "координата не подтверждена как находящаяся в области разведки".
 *
 * После определения формата области эта функция будет реализована
 * отдельно.
 */
uint8_t GPS_CheckReconArea(const GPS_Fix *fix)
{
    (void)fix;

    return 0U;
}