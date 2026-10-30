/* ============================================================================
 * LOGGER
 *
 * Реализация основной стейт-машины логгера.
 * ========================================================================== */
#include <string.h>
#include <math.h>
#include "main.h"
#include "logger_state_machine.h"
#include "gps.h"
#include "rtc_service.h"
#include "lidar.h"
#include "bdbg.h"
#include "bdbg_config.h"

/*
 * Статический буфер для формирования пакетов RadioLine.
 *
 * Размер: 4200 байт.
 * Максимальный пакет спектра: 2073 байта.
 * Две копии пакета: 4146 байт.
 *
 * Буфер размещается в RAM (секция .bss),
 * не занимает стек и не использует malloc.
 */
#define RADIOLINE_TX_BUFFER_SIZE 4200U

static uint8_t radioline_tx_buffer[RADIOLINE_TX_BUFFER_SIZE];

/*
 * Коэффициент дублирования пакетов RadioLine.
 *
 * Начальное значение — 2.
 * В дальнейшем изменяется командой оператора с земли.
 */
static uint8_t repeat_factor = LOGGER_DEFAULT_REPEAT_FACTOR;

/*
 * Общий накопительный буфер RadioLine.
 *
 * Сам буфер создаётся в main.c и заполняется
 * в HAL_UARTEx_RxEventCallback().
 *
 * Обработка содержимого выполняется только
 * в состоянии COMMAND_INFO.
 */
extern uint8_t radioline_rx_buffer[];


/*
 * Количество байтов, накопленных в буфере RadioLine.
 *
 * Значение изменяется при приёме данных USART2.
 */
extern volatile uint16_t radioline_rx_length;


/*
 * Флаг завершения приёма пакета RadioLine.
 *
 * Устанавливается обработчиком TIM6 после 100 мс
 * отсутствия новых данных RadioLine.
 *
 * 0 - приём ещё не завершён;
 * 1 - накопленные данные готовы к обработке
 *     состоянием COMMAND_INFO.
 */
extern volatile uint8_t com_info;

/*
 * Флаг срабатывания общего таймера контроля ответа прибора TIM3.
 *
 * Устанавливается в HAL_TIM_PeriodElapsedCallback() в main.c.
 */
extern volatile uint8_t device_timeout;

/*
 * Секундный счётчик RTC, определённый в main.c.
 * Увеличивается исключительно в прерывании RTC WakeUp.
 * Здесь счётчик только читается и никогда не обнуляется.
 */
extern volatile uint32_t rtc_seconds;



/*
 * TIM3 используется как общий таймер контроля ответа приборов.
 *
 * Сам объект таймера создаётся и инициализируется в main.c.
 */
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart4;


static HAL_StatusTypeDef RadioLine_SendPacket(
    uint8_t packet_id,
    const uint8_t *payload,
    uint16_t payload_length);

		
		
		
		
		
		
		
		
		
		
		
		
		
		
		
		

/*
 * Обработка события таймера контроля ответа прибора.
 *
 * Возвращает:
 *
 * 0 - таймаута нет, можно продолжать обычную обработку
 *     текущего состояния LOGGER;
 *
 * 1 - таймаут был обработан. Обработчик установил следующее
 *     состояние LOGGER, поэтому текущий проход
 *     LoggerStateMachine_Process() должен завершиться.
 */
static uint8_t LoggerStateMachine_ProcessDeviceTimeout(LoggerStateMachine *logger)
{
    /*
     * Если TIM3 не срабатывал, обрабатывать нечего.
     */
    if (device_timeout == 0U)
    {
        return 0U;
    }

    /*
     * Принимаем событие таймаута и останавливаем TIM3.
     */
    device_timeout = 0U;
    (void)HAL_TIM_Base_Stop_IT(&htim3);

    /*
     * ============================================================
     * GPS
     * ============================================================
     */
    if (logger->dev_type == DEV_GPS)
    {
        /*
         * Прекращаем приём GPS и очищаем его данные.
         */
        GPS_StopReceive();
        GPS_Clear();

        /*
         * MODE_TEST: сообщаем о неисправности GPS
         * и возвращаемся к ожиданию команд.
         */
        if (logger->req_mode == MODE_TEST)
        {
            (void)RadioLine_SendPacket(15U, NULL, 0U);
            logger->dev_type = DEV_NONE;
            logger->state = COMMAND_INFO;
            return 1U;
        }

        /*
         * MODE_EXPECT и MODE_RETURN:
         * передаём ошибку обработчику GPS_ERROR_PROC.
         */
        if ((logger->req_mode == MODE_EXPECT) || (logger->req_mode == MODE_RETURN))
        {
            logger->dev_type = DEV_NONE;
            logger->state = GPS_ERROR_PROC;
            return 1U;
        }

        /*
         * MODE_WORK:
         * передаём ошибку обработчику GPS_ERROR_PROC.
         */
        if (logger->req_mode == MODE_WORK)
        {
            logger->dev_type = DEV_NONE;
            logger->state = GPS_ERROR_PROC;
            return 1U;
        }

        /*
         * START_SPECTRE / MODE_SPECTRE:
         * сохраняем существующую обработку GPS.
         */
        if ((logger->work_mode == START_SPECTRE) || (logger->work_mode == MODE_SPECTRE))
        {
            logger->dev_type = DEV_NONE;
            logger->state = GPS_ERROR_PROC;
            return 1U;
        }

        /*
         * Для остальных сочетаний режимов
         * дополнительное поведение не определено.
         */
        return 1U;
    }

    /*
     * ============================================================
     * LIDAR
     * ============================================================
     */
    if (logger->dev_type == DEV_LIDAR)
    {
        /*
         * Время ожидания ответа истекло.
         * Останавливаем DMA-приём LIDAR и сбрасываем
         * событие получения данных.
         */
        LIDAR_StopReceive();
        lidar_info = 0U;

        /*
         * MODE_TEST.
         *
         * ID12 — LIDAR неисправен (нет передачи данных).
         * Используем стандартный Payload.
         *
         * В LIDAR_ERROR_PROC не переходим,
         * счётчик ошибок не увеличиваем.
         */
        if (logger->req_mode == MODE_TEST)
        {
            (void)RadioLine_SendPacket(12U, NULL, 0U);
            logger->dev_type = DEV_NONE;
            logger->state = COMMAND_INFO;
            return 1U;
        }

        /*
         * MODE_WORK.
         *
         * Передаём управление LIDAR_ERROR_PROC.
         * Счётчик последовательных ошибок будет
         * увеличен непосредственно в этом состоянии.
         */
        if (logger->req_mode == MODE_WORK)
        {
            logger->dev_type = DEV_NONE;
            logger->state = LIDAR_ERROR_PROC;
            return 1U;
        }

        /*
         * В остальных режимах LIDAR не должен
         * опрашиваться. Дополнительных действий нет.
         */
        logger->dev_type = DEV_NONE;
        return 1U;
    }

    /* BDBG: истечение TIM3 обрабатывается без работы с UART в ISR. */
    if (logger->dev_type == DEV_BDBG)
    {
        BDBG_StopReceive();
        logger->dev_type = DEV_NONE;
        if (logger->req_mode == MODE_TEST)
        {
            (void)RadioLine_SendPacket(8U, NULL, 0U);
            logger->state = COMMAND_INFO;
        }
        else if (logger->req_mode == MODE_WORK)
        {
            logger->state = BDBG_ERROR_PROC;
        }
        else logger->state = COMMAND_INFO;
        return 1U;
    }

    /*
     * ============================================================
     * Остальные приборы
     * ============================================================
     *
     * Их обработку добавим на следующих этапах.
     */
    return 1U;
}

/* ============================================================================
 * Вычисление расстояния между двумя GPS-координатами.
 *
 * Используется формула Haversine, учитывающая кривизну Земли.
 *
 * Параметры:
 *   coord1 - первая GPS-координата (широта и долгота в градусах);
 *   coord2 - вторая GPS-координата (широта и долгота в градусах).
 *
 * Возвращает:
 *   расстояние между координатами в метрах.
 *
 * Особенности:
 *   - динамическая память не используется;
 *   - данные LOGGER не изменяются;
 *   - функция не зависит от периферии STM32;
 *   - вычисления выполняются с использованием float.
 * ========================================================================== */
static float LoggerStateMachine_CalculateDistance(
    Coord coord1,
    Coord coord2)
{
    /*
     * Средний радиус Земли в метрах.
     */
    const float earth_radius_m = 6371000.0f;

    /*
     * Коэффициент преобразования градусов в радианы.
     */
    const float deg_to_rad = 0.017453292519943295f;

    /*
     * Переводим широты обеих точек в радианы.
     */
    const float lat1 = coord1.lat * deg_to_rad;
    const float lat2 = coord2.lat * deg_to_rad;

    /*
     * Вычисляем разности широт и долгот в радианах.
     */
    const float delta_lat =
        (coord2.lat - coord1.lat) * deg_to_rad;

    const float delta_lon =
        (coord2.lon - coord1.lon) * deg_to_rad;

    /*
     * Вычисляем синусы половин углов.
     */
    const float sin_half_lat = sinf(delta_lat * 0.5f);
    const float sin_half_lon = sinf(delta_lon * 0.5f);

    /*
     * Промежуточное значение формулы Haversine.
     */
    float a =
        sin_half_lat * sin_half_lat +
        cosf(lat1) * cosf(lat2) *
        sin_half_lon * sin_half_lon;

    /*
     * Из-за погрешностей вычислений float значение a
     * теоретически может немного выйти за диапазон [0; 1].
     *
     * Ограничиваем его, чтобы избежать некорректного
     * аргумента функции sqrtf().
     */
    if (a < 0.0f)
    {
        a = 0.0f;
    }
    else if (a > 1.0f)
    {
        a = 1.0f;
    }

    /*
     * Центральный угол между точками в радианах.
     */
    const float central_angle =
        2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));

    /*
     * Длина дуги по поверхности Земли в метрах.
     */
    return earth_radius_m * central_angle;
}




/* ============================================================================
 * Проверка принадлежности GPS-координаты области разведки.
 *
 * Область задаётся выпуклым многоугольником из 3–20 вершин.
 * Вершины должны перечисляться последовательно вдоль его границы
 * (по часовой стрелке или против часовой стрелки).
 *
 * Параметры:
 *   point  - проверяемая GPS-координата;
 *   config - конфигурация области разведки.
 *
 * Возвращает:
 *   1 - координата находится внутри области или на её границе;
 *   0 - координата находится вне области либо конфигурация некорректна.
 *
 * Ограничения:
 *   - область локальная, размером до нескольких десятков километров;
 *   - многоугольник выпуклый;
 *   - пересечение 180-го меридиана исключено.
 *
 * Динамическая память не используется.
 * Состояния LOGGER и конфигурация не изменяются.
 * ========================================================================== */
static uint8_t LoggerStateMachine_IsPointInsideArea(
    Coord point,
    const FlightConfig *config)
{
    /*
     * Направление первого ненулевого векторного произведения.
     *
     *  0 - направление пока не определено;
     *  1 - положительное;
     * -1 - отрицательное.
     */
    int8_t direction = 0;

    /*
     * Проверяем указатель на конфигурацию.
     */
    if (config == NULL)
    {
        return 0U;
    }

    /*
     * Проверяем допустимое количество вершин.
     *
     * Если конфигурация ещё не получена, pointCount = 0.
     */
    if ((config->pointCount < LOGGER_MIN_AREA_POINTS) ||
        (config->pointCount > LOGGER_MAX_AREA_POINTS))
    {
        return 0U;
    }

    /*
     * Последовательно проверяем каждую сторону многоугольника.
     *
     * Последняя вершина соединяется с первой.
     */
    for (uint8_t i = 0U; i < config->pointCount; i++)
    {
        /*
         * Индекс следующей вершины.
         */
        uint8_t next = (uint8_t)(i + 1U);

        if (next == config->pointCount)
        {
            next = 0U;
        }

        /*
         * Начальная и конечная вершины текущей стороны.
         *
         * X соответствует долготе, Y — широте.
         */
        const Coord *a = &config->points[i];
        const Coord *b = &config->points[next];

        /*
         * Вектор стороны AB.
         */
        const double edge_x =
            (double)b->lon - (double)a->lon;

        const double edge_y =
            (double)b->lat - (double)a->lat;

        /*
         * Вектор от вершины A к проверяемой точке P.
         */
        const double point_x =
            (double)point.lon - (double)a->lon;

        const double point_y =
            (double)point.lat - (double)a->lat;

        /*
         * Векторное произведение AB × AP.
         *
         * Положительное значение означает, что точка
         * находится слева от направленной стороны AB.
         *
         * Отрицательное — справа.
         *
         * Нулевое — точка лежит на прямой стороны.
         */
        const double cross =
            edge_x * point_y - edge_y * point_x;

        /*
         * Если точка находится на прямой текущей стороны,
         * эта сторона не ограничивает принадлежность.
         *
         * Точки на границе считаются находящимися внутри.
         */
        if (cross == 0.0)
        {
            continue;
        }

        /*
         * Запоминаем направление первого ненулевого
         * векторного произведения.
         */
        if (direction == 0)
        {
            direction = (cross > 0.0) ? 1 : -1;
        }
        else
        {
            /*
             * Если знак отличается от ранее установленного,
             * точка находится вне выпуклого многоугольника.
             */
            if (((cross > 0.0) && (direction < 0)) ||
                ((cross < 0.0) && (direction > 0)))
            {
                return 0U;
            }
        }
    }

    /*
     * Все стороны проверены.
     *
     * Точка находится внутри области или на её границе.
     */
    return 1U;
}




/**
 * @brief Вычисляет абсолютную разницу между двумя датами и временем.
 *
 * @param time1 Первая дата и время.
 * @param time2 Вторая дата и время.
 *
 * @return Разница в секундах.
 *
 * @note Используется григорианский календарь.
 * @note Предполагается, что входные даты корректны.
 * @note Динамическая память не используется.
 */
static uint64_t LoggerStateMachine_CalculateTimeDifference(
    const GPS_TimeStamp *time1,
    const GPS_TimeStamp *time2)
{
    /*
     * Для каждой даты вычисляем количество суток,
     * прошедших с начала условной календарной шкалы.
     *
     * Алгоритм учитывает високосные годы:
     * - каждый четвёртый год високосный;
     * - каждый сотый год не високосный;
     * - каждый четырёхсотый год високосный.
     */
    const GPS_TimeStamp *times[2] = {time1, time2};
    uint64_t seconds[2] = {0ULL, 0ULL};

    for (uint8_t i = 0U; i < 2U; i++)
    {
        const GPS_TimeStamp *t = times[i];

        /*
         * Количество полных лет, предшествующих текущему году.
         */
        uint32_t years = (uint32_t)t->year - 1U;

        /*
         * Количество суток за полные годы.
         */
        uint32_t days =
            years * 365U +
            years / 4U -
            years / 100U +
            years / 400U;

        /*
         * Количество дней в месяцах обычного года.
         */
        static const uint8_t days_in_month[12] =
        {
            31U, 28U, 31U, 30U,
            31U, 30U, 31U, 31U,
            30U, 31U, 30U, 31U
        };

        /*
         * Добавляем полные месяцы текущего года.
         */
        for (uint8_t month = 1U; month < t->month; month++)
        {
            days += days_in_month[month - 1U];

            /*
             * Добавляем 29 февраля, если текущий год
             * високосный и обрабатываем февраль.
             */
            if ((month == 2U) &&
                ((t->year % 4U == 0U &&
                  t->year % 100U != 0U) ||
                 (t->year % 400U == 0U)))
            {
                days++;
            }
        }

        /*
         * Добавляем дни текущего месяца.
         * Первый день месяца имеет смещение 0.
         */
        days += (uint32_t)t->day - 1U;

        /*
         * Переводим полную дату и время в секунды.
         */
        seconds[i] =
            (uint64_t)days * 86400ULL +
            (uint64_t)t->hour * 3600ULL +
            (uint64_t)t->minute * 60ULL +
            (uint64_t)t->second;
    }

    /*
     * Возвращаем абсолютную разницу.
     * Порядок аргументов значения не имеет.
     */
    if (seconds[0] >= seconds[1])
    {
        return seconds[0] - seconds[1];
    }

    return seconds[1] - seconds[0];
}



/**
 * @brief Проверяет скачок GPS-координат более допустимого расстояния.
 *
 * @param gps_data Указатель на структуру GPS-данных.
 *
 * @return 1 - обнаружен недопустимый скачок координат.
 * @return 0 - скачок не обнаружен или отсутствует
 *             предыдущая достоверная координата.
 *
 * @note Функция не изменяет GPS-данные, счётчики
 *       ошибок и состояние LOGGER.
 */
static uint8_t LoggerStateMachine_IsGpsJumpDetected(
    const GPS_Data *gps_data)
{
    /*
     * Защита от некорректного указателя.
     */
    if (gps_data == NULL)
    {
        return 0U;
    }

    /*
     * Если предыдущей достоверной координаты ещё нет,
     * сравнивать текущую координату не с чем.
     *
     * Проверку скачка пропускаем.
     * Проверка времени GPS-RTC выполняется отдельно.
     */
    if (gps_data->has_last_valid_coord == 0U)
    {
        return 0U;
    }

    /*
     * Вычисляем расстояние между последней достоверной
     * и текущей полученной GPS-координатами.
     *
     * Результат получаем в метрах.
     */
    float distance_m = LoggerStateMachine_CalculateDistance(
        gps_data->last_valid_coord,
        gps_data->current_coord
    );

    /*
     * Сравниваем расстояние с допустимым пределом.
     *
     * LOGGER_GPS_MAX_JUMP_M = 50000 метров.
     *
     * Превышение предела означает подозрение
     * на GPS-спуфинг.
     */
    if (distance_m > LOGGER_GPS_MAX_JUMP_M)
    {
        return 1U;
    }

    /*
     * Недопустимый скачок не обнаружен.
     */
    return 0U;
}



/**
 * @brief Проверяет превышение допустимой скорости перемещения GPS.
 *
 * @param gps_data Указатель на структуру GPS-данных.
 *
 * @return 1 - скорость превышает допустимый предел.
 * @return 0 - превышение не обнаружено или проверка
 *             невозможна из-за отсутствия исходных данных.
 *
 * @note Скорость рассчитывается в метрах в минуту.
 * @note Для вычисления времени используется RTC, а не GPS.
 * @note Функция не изменяет состояние LOGGER и GPS-данные.
 */
static uint8_t LoggerStateMachine_IsGpsSpeedExceeded(
    const GPS_Data *gps_data)
{
    /*
     * Проверяем корректность указателя.
     */
    if (gps_data == NULL)
    {
        return 0U;
    }

    /*
     * Если предыдущей достоверной координаты нет,
     * вычислить скорость перемещения невозможно.
     */
    if (gps_data->has_last_valid_coord == 0U)
    {
        return 0U;
    }

    /*
     * Вычисляем абсолютную разницу времени между
     * последней достоверной координатой и текущей.
     *
     * Используем показания внутреннего RTC STM32.
     *
     * Результат - секунды.
     */
    uint64_t delta_time_s =
        LoggerStateMachine_CalculateTimeDifference(
            &gps_data->last_valid_rtc_time,
            &gps_data->current_rtc_time
        );

    /*
     * Защита от деления на ноль.
     *
     * В нормальном цикле LOGGER интервал опроса
     * составляет не менее 20 секунд.
     *
     * Однако если разница времени равна нулю,
     * проверку скорости пропускаем.
     */
    if (delta_time_s == 0ULL)
    {
        return 0U;
    }

    /*
     * Вычисляем расстояние между последней достоверной
     * и текущей GPS-координатами.
     *
     * Результат - метры.
     */
    float distance_m =
        LoggerStateMachine_CalculateDistance(
            gps_data->last_valid_coord,
            gps_data->current_coord
        );

    /*
     * Вычисляем скорость в метрах в минуту.
     *
     * distance_m - расстояние в метрах.
     * delta_time_s - время в секундах.
     *
     * Умножаем на 60 для перевода в минуты.
     */
    double speed_m_per_min =
        ((double)distance_m * 60.0) / (double)delta_time_s;

    /*
     * Сравниваем рассчитанную скорость
     * с максимально допустимой.
     *
     * LOGGER_GPS_MAX_SPEED_M_PER_MIN = 2000 м/мин.
     */
    if (speed_m_per_min > LOGGER_GPS_MAX_SPEED_M_PER_MIN)
    {
        return 1U;
    }

    /*
     * Скорость находится в допустимых пределах.
     */
    return 0U;
}


/**
 * @brief Проверяет расхождение времени GPS и внутреннего RTC.
 *
 * @param gps_data Указатель на структуру GPS-данных.
 *
 * @return 1 - расхождение времени превышает допустимый предел.
 * @return 0 - расхождение находится в допустимых пределах
 *             или проверка невозможна.
 *
 * @note Проверка не требует предыдущей достоверной координаты.
 * @note Функция не изменяет RTC, GPS-данные, счётчики ошибок
 *       и состояние LOGGER.
 */
static uint8_t LoggerStateMachine_IsGpsTimeMismatch(
    const GPS_Data *gps_data)
{
    /*
     * Защита от некорректного указателя.
     */
    if (gps_data == NULL)
    {
        return 0U;
    }

    /*
     * Вычисляем абсолютную разницу между:
     *
     * 1. Временем, полученным от GPS.
     * 2. Временем внутренних часов RTC.
     *
     * Учитывается полная дата:
     * год, месяц, день, часы, минуты и секунды.
     *
     * Результат получаем в секундах.
     */
    uint64_t time_diff_s =
        LoggerStateMachine_CalculateTimeDifference(
            &gps_data->current_gps_time,
            &gps_data->current_rtc_time
        );

    /*
     * Сравниваем разницу с допустимым пределом.
     *
     * LOGGER_GPS_MAX_TIME_DIFF_S = 1800 секунд.
     *
     * Если разница больше 30 минут,
     * обнаружено подозрение на GPS-спуфинг.
     */
    if (time_diff_s > LOGGER_GPS_MAX_TIME_DIFF_S)
    {
        return 1U;
    }

    /*
     * Расхождение времени не превышает
     * установленный предел.
     */
    return 0U;
}



/**
 * @brief Проверяет текущий GPS-пакет на признаки спуфинга.
 *
 * @param gps_data Указатель на текущие GPS-данные.
 *
 * @return 1 - обнаружен хотя бы один признак спуфинга.
 * @return 0 - признаки спуфинга не обнаружены.
 *
 * @note Функция не изменяет счётчики, координаты и состояния LOGGER.
 */
static uint8_t LoggerStateMachine_IsGpsSpoofingDetected(const GPS_Data *gps_data)
{
    if (gps_data == NULL)
    {
        return 0U;
    }

    /*
     * Проверяем все три признака:
     * 1. Скачок координат более 50 км.
     * 2. Скорость перемещения более 2000 м/мин.
     * 3. Расхождение времени GPS и RTC более 30 минут.
     *
     * Возвращаем один общий результат.
     * Поэтому один GPS-пакет может увеличить счётчик
     * спуфинга не более чем на единицу.
     */
    if ((LoggerStateMachine_IsGpsJumpDetected(gps_data) != 0U) ||
        (LoggerStateMachine_IsGpsSpeedExceeded(gps_data) != 0U) ||
        (LoggerStateMachine_IsGpsTimeMismatch(gps_data) != 0U))
    {
        return 1U;
    }

    return 0U;
}



/* ============================================================================
 * Управление программным интервалом LOGGER.
 *
 * Эти функции пока не подключены к переходам стейт-машины:
 * точки запуска и остановки будут согласованы при реализации
 * режимов MODE_EXPECT, MODE_WORK и MODE_RETURN.
 *
 * RTC и его глобальный секундный счётчик продолжают работать
 * независимо от состояния программного интервала.
 * ========================================================================== */

/*
 * Запуск нового интервала: фиксируем текущую секунду RTC.
 * Повторный вызов начинает отсчёт заново, поэтому вызывать
 * функцию нужно только при реальном начале нового интервала.
 */
void LoggerStateMachine_StartInterval(LoggerStateMachine *logger)
{
    if (logger == NULL) return;

    logger->interval_start_seconds = rtc_seconds;
    logger->interval_control_active = 1U;
}

/*
 * Остановка контроля интервала.
 * Глобальный счётчик RTC не меняется.
 */
void LoggerStateMachine_StopInterval(LoggerStateMachine *logger)
{
    if (logger == NULL) return;

    logger->interval_control_active = 0U;
}




/* ============================================================================
 * Обработка программных интервалов RTC.
 *
 * Вызывается из основного цикла LOGGER, а не из прерывания RTC.
 *
 * Возвращает:
 * 0 - переход состояния не выполнен;
 * 1 - интервал истёк, выполнен переход в REQ_GPS.
 *
 * RTC продолжает работать независимо от программного интервала.
 * ========================================================================== */
static uint8_t LoggerStateMachine_ProcessRtcInterval(
    LoggerStateMachine *logger)
{
    uint32_t required_interval_s;

    /* Защита от некорректного указателя. */
    if (logger == NULL)
    {
        return 0U;
    }

    /*
     * Интервалы ожидания контролируются только в COMMAND_INFO.
     *
     * Во время опроса GPS и остальных приборов
     * новый цикл по таймеру не запускается.
     */
    if (logger->state != COMMAND_INFO)
    {
        return 0U;
    }

    /*
     * Определяем необходимый интервал
     * в зависимости от режима LOGGER.
     */
    switch (logger->req_mode)
    {
        case MODE_EXPECT:
        case MODE_RETURN:
        {
            /*
             * Периодическая трассировка маршрута.
             * Запрашивается только GPS.
             */
            required_interval_s = LOGGER_ROUTE_GPS_INTERVAL_S;
            break;
        }

        case MODE_WORK:
        {
            switch (logger->work_mode)
            {
                case MODE_TIME:
                {
                    /*
                     * Интервал между полными циклами
                     * измерений задаётся оператором.
                     */
                    required_interval_s =
                        logger->flight_config.saveTime;
                    break;
                }

                case MODE_DISTANCE:
                {
                    /*
                     * Периодический запрос GPS.
                     *
                     * Проверка расстояния до последней
                     * сохранённой точки выполняется
                     * позднее, в обработке GPS.
                     */
                    required_interval_s =
                        LOGGER_DISTANCE_GPS_INTERVAL_S;
                    break;
                }

                default:
                {
                    /*
                     * MODE_OPERATOR, START_SPECTRE,
                     * MODE_SPECTRE и остальные режимы
                     * здесь не обрабатываются.
                     */
                    LoggerStateMachine_StopInterval(logger);
                    return 0U;
                }
            }

            break;
        }

        default:
        {
            /*
             * MODE_TEST и остальные режимы
             * не используют этот обработчик.
             */
            LoggerStateMachine_StopInterval(logger);
            return 0U;
        }
    }

    /*
     * Если интервал ещё не запущен,
     * начинаем новый отсчёт от текущей секунды RTC.
     *
     * Повторные вызовы не изменяют точку отсчёта.
     */
    if (logger->interval_control_active == 0U)
    {
        LoggerStateMachine_StartInterval(logger);
        return 0U;
    }

    /*
     * Проверяем продолжительность ожидания.
     *
     * Беззнаковое вычитание uint32_t позволяет
     * корректно обрабатывать переполнение rtc_seconds.
     */
    if ((uint32_t)(rtc_seconds -
                   logger->interval_start_seconds) <
        required_interval_s)
    {
        return 0U;
    }

    /*
     * Интервал истёк.
     *
     * Сначала останавливаем контроль времени,
     * затем устанавливаем новое состояние.
     *
     * Время работы GPS и других приборов
     * в следующий интервал не включается.
     */
    LoggerStateMachine_StopInterval(logger);

    logger->state = REQ_GPS;

    return 1U;
}















/* ============================================================================
 * Инициализация стейт-машины LOGGER.
 *
 * Вызывается один раз после инициализации периферии STM32.
 *
 * Согласно принятой логике LOGGER после включения:
 *
 * 1. Система находится в режиме предполетной проверки MODE_TEST.
 * 2. Основная стейт-машина находится в COMMAND_INFO.
 * 3. Активного ожидания ответа от приборов нет.
 * 4. БДБГ подготовлен к обычному запросу ПАЕД.
 * 5. Параметры области разведки ещё не получены.
 * 6. Опорные GPS-координаты ещё не установлены.
 *
 * Все данные размещаются в памяти структуры LoggerStateMachine.
 * Динамическое выделение памяти не используется.
 * ========================================================================== */
void LoggerStateMachine_Init(LoggerStateMachine *logger)
{
    uint8_t i;

    /*
     * Защита от нулевого указателя.
     */
    if (logger == 0)
    {
        return;
    }

    /* ========================================================================
     * 1. Начальные режимы и состояния LOGGER.
     * ====================================================================== */

    /*
     * После включения выполняется предполетная проверка.
     */
    logger->req_mode = MODE_TEST;

    /*
     * До получения параметров разведки используется
     * операторский режим измерений.
     */
    logger->work_mode = MODE_OPERATOR;

    /*
     * Предыдущий режим первоначально также операторский.
     */
    logger->old_work_mode = MODE_OPERATOR;

    /*
     * После запуска LOGGER ожидает команду с земли.
     */
    logger->state = COMMAND_INFO;

    /*
     * Активного ожидания ответа прибора пока нет.
     */
    logger->dev_type = DEV_NONE;
		
		/*
		 * При включении LOGGER контроль временных интервалов отключён.
		 *
		 * Начальный режим MODE_TEST не требует периодического
		 * запуска измерений по RTC.
		 *
		 * Отсчёт будет включён позже при переходе
		 * в соответствующий рабочий режим.
		 */
		logger->interval_control_active = 0U;

		/*
		 * Начальное значение секундного счётчика интервала.
		 *
		 * Пока контроль отключён, значение не используется.
		 * При запуске интервала сюда будет записано
		 * текущее значение глобального rtc_seconds.
		 */
		logger->interval_start_seconds = 0U;
		
		
		
        /*
         * Координаты последнего сохранённого полного измерения
         * инициализируются ниже вместе с остальными GPS-данными.
         * Заполнение этих полей после успешной записи полного
         * измерения на SD-карту пока остаётся заглушкой.
         */

		
		

    /*
     * Начальный тип запроса к БДБГ-15.
     */
    logger->bdbg_request = PAED_REQ;


    /* ========================================================================
     * 2. Счётчики последовательных ошибок приборов.
     *
     * Все счётчики начинаются с нуля.
     * ====================================================================== */

    logger->bdbg_error_counter = 0U;
    logger->gps_error_counter = 0U;
    logger->lidar_error_counter = 0U;
    logger->batt_error_counter = 0U;
    logger->rtc_error_counter = 0U;

    /*
     * Отдельный счётчик последовательных обнаружений спуфинга.
     *
     * Не связан со счётчиком ошибок GPS.
     */
    logger->gps_spoofing_counter = 0U;


    /* ========================================================================
     * 3. Начальные параметры радиационной разведки.
     *
     * До получения команды ID04 действующая область разведки
     * отсутствует.
     * ====================================================================== */

    /*
     * Нулевое количество точек означает отсутствие
     * принятой конфигурации области разведки.
     */
    logger->flight_config.pointCount = 0U;

    /*
     * Очищаем весь статический массив координат.
     *
     * Используем только фиксированный массив из 20 элементов.
     */
    for (i = 0U; i < LOGGER_MAX_AREA_POINTS; i++)
    {
        logger->flight_config.points[i].lat = 0.0f;
        logger->flight_config.points[i].lon = 0.0f;
    }

    /*
     * Начальный режим измерений — операторский.
     */
    logger->flight_config.mode = (uint8_t)MODE_OPERATOR;

    /*
     * Интервал сохранения измерений пока не задан.
     */
    logger->flight_config.saveTime = 0U;

    /*
     * Минимальная дистанция между точками сохранения
     * пока не задана.
     */
    logger->flight_config.saveDistance = 0U;

    /*
     * Начальный коэффициент дублирования радиопакетов.
     */
    logger->flight_config.duplCoeff = LOGGER_DEFAULT_REPEAT_FACTOR;

    /*
     * Начальное время набора спектра — 60 секунд.
     */
    logger->flight_config.saveSpectre = LOGGER_DEFAULT_SPECTRUM_TIME;


    /* ========================================================================
     * 4. Начальные данные GPS.
     *
     * После включения ни одной валидной координаты ещё нет.
     *
     * Все координаты, временные метки и расстояния
     * инициализируются нулевыми значениями.
     * ====================================================================== */

    /*
     * Последняя валидная координата.
     */
    logger->gps_data.last_valid_coord.lat = 0.0f;
    logger->gps_data.last_valid_coord.lon = 0.0f;
    logger->gps_data.has_last_valid_coord = 0U;

    /*
     * Временная метка RTC последней валидной координаты.
     */
    logger->gps_data.last_valid_rtc_time =
        (GPS_TimeStamp){0};

    /*
     * Текущая GPS-координата.
     */
    logger->gps_data.current_coord.lat = 0.0f;
    logger->gps_data.current_coord.lon = 0.0f;

    /*
     * Время, полученное от GPS.
     */
    logger->gps_data.current_gps_time =
        (GPS_TimeStamp){0};

    /*
     * Время RTC текущего GPS-измерения.
     */
    logger->gps_data.current_rtc_time =
        (GPS_TimeStamp){0};

    /*
     * Последняя координата сохранённого измерения.
     */
    logger->gps_data.last_saved_coord.lat = 0.0f;
    logger->gps_data.last_saved_coord.lon = 0.0f;
    logger->gps_data.has_last_saved_coord = 0U;

    /*
     * Время RTC последнего сохранённого измерения.
     */
    logger->gps_data.last_saved_rtc_time =
        (GPS_TimeStamp){0};

    /*
     * Расстояния первоначально равны нулю.
     */
    logger->gps_data.last_move_distance_m = 0.0f;
    logger->gps_data.distance_from_saved_m = 0.0f;


    /* ========================================================================
     * 5. Данные RadioLine.
     *
     * После включения завершённых пакетов ещё нет.
     * ====================================================================== */

    logger->radioline_data = 0;
    logger->radioline_length = 0U;
}






























/* ============================================================================
 * Передача завершённого burst RadioLine в стейт-машину.
 *
 * Функция вызывается из main(), когда TIM6 уже определил окончание
 * приёма RadioLine по паузе 100 мс.
 *
 * Здесь данные НЕ разбираются и НЕ копируются.
 * Стейт-машина получает только адрес буфера и количество принятых байтов.
 * ========================================================================== */
void LoggerStateMachine_SetRadioLineData(LoggerStateMachine *logger,
                                         const uint8_t *data,
                                         uint16_t length)
{
    /*
     * Проверяем корректность переданных параметров.
     */
    if ((logger == 0) || (data == 0) || (length == 0U))
    {
        return;
    }

    /*
     * Запоминаем адрес накопительного буфера RadioLine.
     * Сам массив остаётся в main.c.
     */
    logger->radioline_data = data;

    /*
     * Запоминаем фактическое количество принятых байтов.
     */
    logger->radioline_length = length;
}


uint16_t calculateCrc16(const uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}



/*
 * Дескриптор USART2, созданный в main.c.
 *
 * USART2 используется для обмена с наземной станцией
 * по каналу RadioLine.
 */
extern UART_HandleTypeDef huart2;



/*
 * Формирование и передача пакета RadioLine.
 *
 * packet_id      - идентификатор сообщения.
 * payload        - указатель на полезные данные.
 * payload_length - длина полезных данных в байтах.
 *
 * Формат пакета:
 *   START   - 4 байта
 *   ID      - 1 байт
 *   LENGTH  - 2 байта
 *   PAYLOAD - payload_length байт
 *   CRC16   - 2 байта
 *   END     - 4 байта
 *   PAD     - 2 байта
 *
 * Полная длина = payload_length + 15 байт.
 *
 * Используется статический буфер radioline_tx_buffer[4200].
 * Динамическое выделение памяти не применяется.
 *
 * Каждая копия передаётся отдельным вызовом
 * HAL_UART_Transmit() с задержкой 50 мс.
 */
static HAL_StatusTypeDef RadioLine_SendPacket(
    uint8_t packet_id,
    const uint8_t *payload,
    uint16_t payload_length)
{
    /* Стандартные данные для сообщений без Payload. */
    const uint8_t default_payload = 0x01U;

    /* Полная длина одного сформированного пакета. */
    uint32_t packet_length;

    /* Контрольная сумма пакета. */
    uint16_t crc;

    /* Счётчик передаваемых копий. */
    uint8_t repeat;

    /* Результат операции передачи USART2. */
    HAL_StatusTypeDef status;

    /*
     * Если полезные данные отсутствуют,
     * используем один байт 0x01.
     */
    if ((payload == NULL) || (payload_length == 0U))
    {
        payload = &default_payload;
        payload_length = 1U;
    }

    /*
     * Проверяем допустимое количество повторов.
     */
    if ((repeat_factor < LOGGER_MIN_REPEAT_FACTOR) ||
        (repeat_factor > LOGGER_MAX_REPEAT_FACTOR))
    {
        return HAL_ERROR;
    }

    /*
     * Рассчитываем длину одного пакета
     * с учётом всех служебных байтов.
     */
    packet_length = 15U + (uint32_t)payload_length;

    /*
     * Проверяем, что один пакет помещается
     * в статический буфер.
     */
    if (packet_length > sizeof(radioline_tx_buffer))
    {
        return HAL_ERROR;
    }

    /*
     * Проверяем ограничение длины HAL_UART_Transmit().
     */
    if (packet_length > 65535U)
    {
        return HAL_ERROR;
    }

    /* Начальный маркер пакета. */
    radioline_tx_buffer[0] = 0xAAU;
    radioline_tx_buffer[1] = 0x55U;
    radioline_tx_buffer[2] = 0xAAU;
    radioline_tx_buffer[3] = 0x55U;

    /* Идентификатор сообщения. */
    radioline_tx_buffer[4] = packet_id;

    /*
     * Длина Payload в формате Little Endian.
     */
    radioline_tx_buffer[5] =
        (uint8_t)(payload_length & 0xFFU);

    radioline_tx_buffer[6] =
        (uint8_t)((payload_length >> 8U) & 0xFFU);

    /*
     * Копируем Payload в буфер,
     * начиная с позиции 7.
     */
    memcpy(
        &radioline_tx_buffer[7],
        payload,
        payload_length
    );

    /*
     * Рассчитываем CRC16 по полям:
     * ID + LENGTH + PAYLOAD.
     */
    crc = calculateCrc16(
        &radioline_tx_buffer[4],
        (uint16_t)(3U + payload_length)
    );

    /* Младший байт CRC16. */
    radioline_tx_buffer[7U + payload_length] =
        (uint8_t)(crc & 0xFFU);

    /* Старший байт CRC16. */
    radioline_tx_buffer[8U + payload_length] =
        (uint8_t)((crc >> 8U) & 0xFFU);

    /* Конечный маркер пакета. */
    radioline_tx_buffer[9U + payload_length]  = 0x0DU;
    radioline_tx_buffer[10U + payload_length] = 0x0AU;
    radioline_tx_buffer[11U + payload_length] = 0x0DU;
    radioline_tx_buffer[12U + payload_length] = 0x0AU;

    /*
     * Два дополнительных байта после маркера.
     */
    radioline_tx_buffer[13U + payload_length] = 0x00U;
    radioline_tx_buffer[14U + payload_length] = 0x00U;

    /*
     * Передаём сформированный пакет repeat_factor раз.
     *
     * Все копии используют один и тот же буфер.
     * Повторное формирование пакета и расчёт CRC
     * не требуются.
     */
    for (repeat = 0U; repeat < repeat_factor; repeat++)
    {
        /*
         * Блокирующая передача одной копии пакета.
         */
        status = HAL_UART_Transmit(
            &huart2,
            radioline_tx_buffer,
            (uint16_t)packet_length,
            1000U
        );

        /*
         * При ошибке прекращаем передачу
         * оставшихся копий.
         */
        if (status != HAL_OK)
        {
            return status;
        }

        /*
				* Задержка между отправками
				* без задержки работа нестабильна
        */
      
        HAL_Delay(50U);
       
    }

    /* Все копии успешно переданы. */
    return HAL_OK;
}



/* ============================================================================
 * Выполнение одного шага основной стейт-машины LOGGER.
 *
 * Функция вызывается из главного цикла программы и выполняет обработку
 * ТОЛЬКО текущего состояния.
 *
 * Здесь не должно быть бесконечных циклов ожидания.
 * После обработки текущего состояния функция возвращает управление в main().
 * ========================================================================== */
void LoggerStateMachine_Process(LoggerStateMachine *logger)
{
    /*
     * Защита от передачи нулевого указателя.
     */
    if (logger == 0)
    {
        return;
    }


		#ifdef DEBUG_MODE
		/*
		 * Запоминаем состояние, для которого
		 * уже отправлено DEBUG-сообщение.
		 */
		static DroneState debug_previous_state = (DroneState)-1;

		/*
		 * При переходе в другое состояние разрешаем
		 * отправку нового DEBUG-сообщения.
		 */
		if (debug_previous_state != logger->state)
		{
				debug_previous_state = logger->state;

				if (logger->state == COMMAND_INFO)
				{
						/*
						 * Сообщение о входе в COMMAND_INFO.
						 */
						(void)RadioLine_SendPacket(70U, NULL, 0U);						
				}
				else if (logger->state == GPS_PROC)
				{
						/*
						 * Сообщение о входе в GPS_PROC.
						 */
					  (void)RadioLine_SendPacket(81U, NULL, 0U);
				}
				else if (logger->state == GPS_ERROR_PROC)
				{
						/*
						 * Сообщение о входе в GPS_ERROR_PROC.
						 */
						(void)RadioLine_SendPacket(82U, NULL, 0U);
				}
				else if (logger->state == REQ_GPS)
				{
						/*
						 * Сообщение о входе в REQ_GPS.
						 */
						(void)RadioLine_SendPacket(80U, NULL, 0U);
				}
				HAL_Delay(50U);			
				
		}
		#endif
		
		/*
		 * Сначала проверяем асинхронное событие таймера контроля
		 * ответа прибора.
		 *
		 * Если таймаут произошёл, его обработчик сам выполняет
		 * необходимые действия и устанавливает следующее состояние.
		 *
		 * Поэтому обычную обработку текущего состояния в этом
		 * проходе уже не выполняем.
		 */
		if (LoggerStateMachine_ProcessDeviceTimeout(logger) != 0U)
		{
				return;
		}

		/*
		 * Проверяем программный интервал RTC только после
		 * обработки возможного тайм-аута прибора.
		 *
		 * Если интервал истёк, состояние уже изменено на REQ_GPS.
		 * Сам запрос будет выполнен на следующем проходе
		 * главного цикла, чтобы сохранить правило «один шаг за вызов».
		 */
		if (LoggerStateMachine_ProcessRtcInterval(logger) != 0U)
		{
				return;
		}

    /*
     * Основной диспетчер состояний LOGGER.
     */
    switch (logger->state)
    {
        /*
         * ============================================================
         * COMMAND_INFO
         *
         * Обработка накопленного массива RadioLine.
         *
         * Наземная станция отправляет дублирующиеся пакеты
         * одной передачей.
         *
         * Ищем первый корректный пакет.
         *
         * Если пакет найден:
         *   - отправляем ответ 01 "Пакет принят";
         *   - прекращаем поиск;
         *   - отбрасываем остальные дубликаты.
         *
         * Если корректный пакет не найден:
         *   - отправляем ответ 02 "Пакет не принят".
         *
         * Земля подтверждений не отправляет.
         * ============================================================
         */ 
			
				case COMMAND_INFO:
				{
						/*
						 * Ожидаем завершения приёма RadioLine.
						 *
						 * TIM6 устанавливает com_info после паузы 100 мс.
						 */
						if (com_info == 0U)
						{
								break;
						}

						com_info = 0U;

						/*
						 * Минимальный размер радиопакета с одним байтом Payload.
						 */
						if (radioline_rx_length < 14U)
						{
								radioline_rx_length = 0U;

								(void)RadioLine_SendPacket(0x02U, NULL, 0U);

								break;
						}

						uint8_t start_found = 0U;
						uint8_t packet_found = 0U;

						uint16_t start_pos = 0U;
						uint16_t end_pos = 0U;

						uint8_t received_command = 0U;

						/*
						 * Запоминаем положение Payload и его длину.
						 *
						 * Эти значения понадобятся при обработке ID04.
						 */
						uint16_t received_payload_pos = 0U;
						uint16_t received_payload_length = 0U;

						/*
						 * Поиск первого корректного пакета в принятом burst.
						 *
						 * Остальные дубликаты игнорируются.
						 */
						for (uint16_t i = 0U;
								 (uint32_t)i + 3U < radioline_rx_length;
								 i++)
						{
								/*
								 * Поиск маркера начала AA 55 AA 55.
								 */
								if ((radioline_rx_buffer[i]      == 0xAAU) &&
										(radioline_rx_buffer[i + 1U] == 0x55U) &&
										(radioline_rx_buffer[i + 2U] == 0xAAU) &&
										(radioline_rx_buffer[i + 3U] == 0x55U))
								{
										start_pos = i;
										start_found = 1U;

										i += 3U;

										continue;
								}

								if (start_found == 0U)
								{
										continue;
								}

								/*
								 * Поиск маркера окончания 0D 0A 0D 0A.
								 */
								if ((radioline_rx_buffer[i]      == 0x0DU) &&
										(radioline_rx_buffer[i + 1U] == 0x0AU) &&
										(radioline_rx_buffer[i + 2U] == 0x0DU) &&
										(radioline_rx_buffer[i + 3U] == 0x0AU))
								{
										end_pos = i;

										/*
										 * Проверяем минимальную длину найденного фрагмента.
										 */
										if (((uint32_t)end_pos - start_pos) < 10U)
										{
												start_found = 0U;

												i += 3U;

												continue;
										}

										/*
										 * Извлекаем длину Payload (Little Endian).
										 */
										uint16_t payload_length =
												(uint16_t)radioline_rx_buffer[start_pos + 5U] |
												((uint16_t)radioline_rx_buffer[start_pos + 6U] << 8U);

										/*
										 * Вычисляем фактическую длину Payload.
										 */
										uint16_t actual_payload_length =
												(uint16_t)((uint32_t)end_pos -
																	 (uint32_t)start_pos - 9U);

										if ((payload_length == 0U) ||
												(payload_length != actual_payload_length))
										{
												start_found = 0U;

												i += 3U;

												continue;
										}

										/*
										 * Извлекаем принятую CRC16 (Little Endian).
										 */
										uint16_t received_crc =
												(uint16_t)radioline_rx_buffer[end_pos - 2U] |
												((uint16_t)radioline_rx_buffer[end_pos - 1U] << 8U);

										/*
										 * Вычисляем CRC16 по Command ID, Length и Payload.
										 */
										uint16_t calculated_crc =
												calculateCrc16(
														&radioline_rx_buffer[start_pos + 4U],
														(uint16_t)(3U + payload_length)
												);

										if (received_crc != calculated_crc)
										{
												start_found = 0U;

												i += 3U;

												continue;
										}

										/*
										 * Первый корректный пакет найден.
										 *
										 * Сохраняем его идентификатор и расположение Payload.
										 */
										received_command = radioline_rx_buffer[start_pos + 4U];

										received_payload_pos = (uint16_t)(start_pos + 7U);
										received_payload_length = payload_length;

										packet_found = 1U;

										break;
								}
						}

						/*
						 * Освобождаем накопительный буфер.
						 *
						 * Его содержимое остаётся доступным до следующего приёма.
						 */
						radioline_rx_length = 0U;

						if (packet_found == 0U)
						{
								/*
								 * Корректный пакет не найден.
								 */
								(void)RadioLine_SendPacket(0x02U, NULL, 0U);

								break;
						}

						/*
						 * Пакет успешно прошёл проверку структуры и CRC16.
						 *
						 * ID01 подтверждает именно приём радиопакета,
						 * а не успешное применение его параметров.
						 */
						(void)RadioLine_SendPacket(0x01U, NULL, 0U);

						/*
						 * ================================================================
						 * Команда ID01 — предполётное тестирование.
						 *
						 * Сохраняем существующий алгоритм без изменений.
						 * ================================================================
						 */
						if ((logger->req_mode == MODE_TEST) &&
								(received_command == 0x01U))
						{
								logger->state = REQ_GPS;
						}

						/*
						 * ================================================================
						 * Команда ID04 — параметры радиационной разведки.
						 *
						 * Разбираем Payload только после проверки CRC16.
						 *
						 * Формат:
						 *
						 * pointCount    uint8_t
						 * points[]      pointCount * (float lat + float lon)
						 * mode          uint8_t
						 * saveTime      uint16_t
						 * saveDistance  uint16_t
						 * duplCoeff     uint8_t
						 * saveSpectre   uint16_t
						 *
						 * Все многобайтовые значения передаются Little Endian.
						 * ================================================================
						 */
						if (received_command == 0x04U)
						{
								/*
								 * Минимальная длина Payload при трёх вершинах:
								 *
								 * 1 + 3 * 8 + 1 + 2 + 2 + 1 + 2 = 33 байта.
								 */
								if (received_payload_length >=
										(uint16_t)(9U + 8U * LOGGER_MIN_AREA_POINTS))
								{
										const uint8_t *payload =
												&radioline_rx_buffer[received_payload_pos];

										uint8_t point_count = payload[0];

										/*
										 * Проверяем допустимое количество вершин.
										 */
										if ((point_count >= LOGGER_MIN_AREA_POINTS) &&
												(point_count <= LOGGER_MAX_AREA_POINTS))
										{
												/*
												 * Проверяем точную длину Payload.
												 *
												 * Это исключает чтение за пределами пакета.
												 */
												uint16_t expected_length =
														(uint16_t)(9U + 8U * point_count);

												if (received_payload_length == expected_length)
												{
														/*
														 * Временная структура фиксированного размера.
														 *
														 * До завершения проверки действующая
														 * конфигурация LOGGER не изменяется.
														 */
														FlightConfig new_config = {0};

														uint16_t offset = 1U;

														new_config.pointCount = point_count;

														/*
														 * Копируем координаты по 4 байта.
														 *
														 * memcpy исключает нарушение требований
														 * выравнивания при чтении float.
														 *
														 * Формат float на STM32 — IEEE-754,
														 * порядок байтов Little Endian.
														 */
														for (uint8_t p = 0U; p < point_count; p++)
														{
																memcpy(&new_config.points[p].lat,
																			 &payload[offset],
																			 sizeof(float));

																offset += (uint16_t)sizeof(float);

																memcpy(&new_config.points[p].lon,
																			 &payload[offset],
																			 sizeof(float));

																offset += (uint16_t)sizeof(float);
														}

														/*
														 * Режим разведки.
														 */
														new_config.mode = payload[offset];
														offset++;

														/*
														 * Интервал сохранения, секунды.
														 */
														new_config.saveTime =
																(uint16_t)payload[offset] |
																((uint16_t)payload[offset + 1U] << 8U);

														offset += 2U;

														/*
														 * Минимальная дистанция сохранения, метры.
														 */
														new_config.saveDistance =
																(uint16_t)payload[offset] |
																((uint16_t)payload[offset + 1U] << 8U);

														offset += 2U;

														/*
														 * Коэффициент дублирования радиопакетов.
														 */
														new_config.duplCoeff = payload[offset];
														offset++;

														/*
														 * Время набора спектра, секунды.
														 */
														new_config.saveSpectre =
																(uint16_t)payload[offset] |
																((uint16_t)payload[offset + 1U] << 8U);

														/*
														 * Проверяем допустимые значения.
														 *
														 * Режимы ID04:
														 * 0 — операторский;
														 * 1 — по времени;
														 * 2 — по расстоянию;
														 * 3 — спектр.
														 */
														if ((new_config.mode <= 3U) && (new_config.duplCoeff >=  LOGGER_MIN_REPEAT_FACTOR) && (new_config.duplCoeff <= LOGGER_MAX_REPEAT_FACTOR))
														{
																/*
																 * Все проверки завершены.
																 *
																 * Заменяем действующую конфигурацию
																 * одним присваиванием структуры.
																 *
																 * Массив точек копируется вместе
																 * с остальными полями.
																 */
																logger->flight_config = new_config;

																/*
																 * Режим работы и переходы состояний
																 * здесь не изменяем.
																 *
																 * Применение параметров будет выполнено
																 * в соответствующих состояниях
																 * утверждённой стейт-машины.
																 */
														}
												}
										}
								}
						}

						break;
				}
								 

        /*
         * ============================================================
         * Остальные состояния.
         *
         * Их реализацию добавим последовательно.
         * ============================================================
         */
        /* Запуск GPS: тайм-аут начинается после отладочной задержки. */
       case REQ_GPS:
				{
						/*
						 * Очищаем данные предыдущего сеанса приёма GPS.
						 */
						GPS_Clear();

						/*
						 * Сбрасываем событие GPS перед началом нового ожидания.
						 */
						gps_info = 0U;

						/*
						 * Указываем, что TIM3 сейчас контролирует ответ GPS.
						 */
						logger->dev_type = DEV_GPS;

						/*
						 * Сбрасываем возможное старое событие таймаута.
						 */
						device_timeout = 0U;

						/*
						 * Запускаем приём GPS через USART3 + DMA.
						 */
						GPS_StartReceive();

						/*
						 * Запускаем TIM3 на 5 секунд.
						 *
						 * TIM3 уже настроен в CubeMX:
						 * 1 тик = 1 мс;
						 * ARR = 4999 -> 5000 мс.
						 */
						__HAL_TIM_SET_COUNTER(&htim3, 0U);

						if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
						{
								Error_Handler();
						}

						/*
						 * GPS запущен.
						 * Дальнейшее ожидание и обработка NMEA выполняются
						 * неблокирующе в GPS_PROC.
						 */
						logger->state = GPS_PROC;

						break;
				}

       /* Неблокирующая обработка поступивших NMEA-предложений. */
       case GPS_PROC:
			 {
					GPS_Fix fix;

					/*
					 * Ждём событие приёма данных от GPS.
					 * gps_info устанавливается в HAL_UARTEx_RxEventCallback().
					 */
					if (gps_info == 0U)
					{
							break;
					}

					/*
					 * Сбрасываем событие.
					 * Следующая принятая порция данных снова установит gps_info.
					 */
					gps_info = 0U;

					/*
					 * Пытаемся извлечь корректный $GNRMC.
					 *
					 * Если полного корректного пакета пока нет,
					 * остаёмся в GPS_PROC и ждём следующие данные.
					 */
					if (GPS_Process(&fix) == 0U)
					{
							break;
					}

					/*
					 * Получены корректные координаты и время.
					 * Останавливаем таймер контроля неответа GPS.
					 */
					HAL_TIM_Base_Stop_IT(&htim3);

					/*
					 * Останавливаем приём GPS и очищаем его приёмные данные.
					 */
					GPS_StopReceive();
					GPS_Clear();

					device_timeout = 0U;
					gps_info = 0U;

					/*
					 * Корректный ответ GPS получен —
					 * последовательность ошибок GPS прервана.
					 */
					logger->gps_error_counter = 0U;
					
					/*
					 * Сохраняем координаты текущего корректного GPS-пакета.
					 *
					 * Пока не считаем их достоверными для проверки спуфинга:
					 * здесь только переносим данные в рабочую структуру.
					 */
					logger->gps_data.current_coord.lat = fix.latitude;
					logger->gps_data.current_coord.lon = fix.longitude;

					/*
					 * Сохраняем полную дату и время UTC из GPS.
					 *
					 * В GPS_Fix год представлен двумя цифрами (0–99).
					 * В GPS_TimeStamp используем полный год (2000–2099).
					 */
					logger->gps_data.current_gps_time.year =
							(uint16_t)(2000U + fix.year);

					logger->gps_data.current_gps_time.month  = fix.month;
					logger->gps_data.current_gps_time.day    = fix.day;
					logger->gps_data.current_gps_time.hour   = fix.hour;
					logger->gps_data.current_gps_time.minute = fix.minute;
					logger->gps_data.current_gps_time.second = fix.second;				

					/*
					 * MODE_TEST.
					 *
					 * Первая корректная координата принимается без проверки
					 * spoofing и используется как начальная.
					 */
					if (logger->req_mode == MODE_TEST)
					{
							uint8_t payload[11];

							/*
							 * Подготавливаем дату и время UTC для нового модуля RTC.
							 *
							 * GPS_Fix содержит двузначный год (например, 26).
							 * GPS_TimeStamp использует полный год (например, 2026).
							 */
							GPS_TimeStamp rtc_timestamp = {0};

							rtc_timestamp.year   = (uint16_t)(2000U + fix.year);
							rtc_timestamp.month  = fix.month;
							rtc_timestamp.day    = fix.day;

							rtc_timestamp.hour   = fix.hour;
							rtc_timestamp.minute = fix.minute;
							rtc_timestamp.second = fix.second;

							/*
							 * Синхронизируем аппаратный RTC по времени GPS.
							 *
							 * Если синхронизация завершилась ошибкой,
							 * предполётная проверка GPS считается неуспешной.
							 */
							if (RTC_Service_SetTime(&rtc_timestamp) != HAL_OK)
							{
									/*
									 * Отправляем оператору сообщение:
									 * ID 26 — "Ошибка синхронизации RTC".
									 *
									 * RadioLine_SendPacket() автоматически использует
									 * стандартный Payload 0x01.
									 */
									(void)RadioLine_SendPacket(26U, NULL, 0U);

									/*
									 * Не сохраняем полученную координату как достоверную.
									 * Не переходим к проверке LIDAR.
									 *
									 * Возвращаемся к ожиданию команд оператора.
									 */
									logger->dev_type = DEV_NONE;
									logger->state = COMMAND_INFO;

									break;
							}
														
							
							/*
							 * Первая корректная GPS-координата в MODE_TEST
							 * принимается без проверки спуфинга.
							 *
							 * Сохраняем её как начальную достоверную координату
							 * для последующего контроля перемещения.
							 */
							logger->gps_data.last_valid_coord = logger->gps_data.current_coord;

							/*
							 * Отмечаем наличие достоверной координаты.
							 */
							logger->gps_data.has_last_valid_coord = 1U;

							/*
							 * Сохраняем время первой достоверной координаты.
							 *
							 * В MODE_TEST RTC синхронизируется по GPS,
							 * поэтому используем соответствующую временную метку.
							 */
							logger->gps_data.last_valid_rtc_time = rtc_timestamp;

							/*
							 * Пакет 19:
							 * latitude  - float, 4 байта
							 * longitude - float, 4 байта
							 * время     - HH, MM, SS
							 */
							memcpy(&payload[0], &fix.latitude, sizeof(float));
							memcpy(&payload[4], &fix.longitude, sizeof(float));

							payload[8]  = fix.hour;
							payload[9]  = fix.minute;
							payload[10] = fix.second;

							/*
							 * MODE_TEST:
							 * "Координата, время, вне области".
							 */
							(void)RadioLine_SendPacket(19U,
																				 payload,
																				 sizeof(payload));

							/*
							 * GPS успешно проверен.
							 * Переходим к LIDAR.
							 */
							logger->state = REQ_LIDAR;

							break;
					}

						/*
						* MODE_EXPECT, MODE_WORK и MODE_RETURN
						* добавим строго по соответствующим веткам стейт-машины.
						*/
						
						/*
						* В рабочих режимах получаем текущее время
						* от аппаратных часов RTC STM32.
						*
						* RTC был синхронизирован с GPS в MODE_TEST.
						* Повторную синхронизацию здесь не выполняем,
						* чтобы сохранить независимый источник времени
						* для последующей проверки GPS-спуфинга.
						*/
						if ((logger->req_mode == MODE_EXPECT) || (logger->req_mode == MODE_WORK) || (logger->req_mode == MODE_RETURN))
						{
							
						
							 /*
							 * Читаем текущие дату и время аппаратного RTC.
							 *
							 * При ошибке чтения увеличиваем отдельный счётчик
							 * последовательных ошибок RTC.
							 */
							if (RTC_Service_GetTime(&logger->gps_data.current_rtc_time) != HAL_OK)
							{
									/*
									 * Увеличиваем счётчик без переполнения uint8_t.
									 */
									if (logger->rtc_error_counter < UINT8_MAX)
									{
											logger->rtc_error_counter++;
									}

									/*
									 * Если число ошибок превысило допустимый максимум,
									 * дальнейшая радиационная разведка невозможна.
									 *
									 * LOGGER_DEVICE_ERROR_LIMIT = 3.
									 * Переход в MODE_ERROR произойдёт на четвёртой
									 * последовательной ошибке чтения RTC.
									 */
									if (logger->rtc_error_counter >= LOGGER_DEVICE_ERROR_LIMIT)
									{
											/*
											 * ID 26 — ошибка синхронизации RTC.
											 * Стандартный Payload: 0x01.
											 */
											(void)RadioLine_SendPacket(26U, NULL, 0U);

											logger->req_mode = MODE_ERROR;
									}

									/*
									 * Не принимаем текущую GPS-координату
									 * как достоверную.
									 *
									 * Возвращаемся к обработке команд.
									 */
									logger->dev_type = DEV_NONE;
									logger->state = COMMAND_INFO;

									break;
							 }

								/*
								 * RTC успешно прочитан.
								 * Последовательность ошибок прервана.
								 */
								logger->rtc_error_counter = 0U;
							 
								 /*
								 * Проверяем GPS-пакет на признаки спуфинга:
								 * скачок координат, превышение скорости, расхождение GPS/RTC.
								 */
								if (LoggerStateMachine_IsGpsSpoofingDetected(&logger->gps_data) != 0U)
								{
										/*
										 * Один подозрительный GPS-пакет увеличивает счётчик на единицу,
										 * независимо от количества обнаруженных признаков спуфинга.
										 */
										if (logger->gps_spoofing_counter < UINT8_MAX)
										{
												logger->gps_spoofing_counter++;
										}

										/*
										 * Три последовательных обнаружения спуфинга означают,
										 * что дальнейшая радиационная разведка невозможна.
										 */
										if (logger->gps_spoofing_counter >= LOGGER_GPS_SPOOFING_LIMIT)
										{
												/*
												 * ID 27 — "Обнаружен GPS-спуфинг".
												 * Стандартный Payload: 0x01.
												 */
												(void)RadioLine_SendPacket(27U, NULL, 0U);

												logger->req_mode = MODE_ERROR;
										}

										/*
										 * Подозрительную координату не принимаем как достоверную.
										 * Возвращаемся к обработке команд.
										 */
										logger->dev_type = DEV_NONE;
										logger->state = COMMAND_INFO;
										break;
								}

								/*
								 * GPS-пакет прошёл все проверки.
								 * Сбрасываем счётчик последовательных обнаружений спуфинга.
								 */
								logger->gps_spoofing_counter = 0U;

								/*
								 * Сохраняем координату и время RTC последнего
								 * достоверного GPS-измерения.
								 *
								 * Эти данные используются при проверке следующего GPS-пакета.
								 */
								logger->gps_data.last_valid_coord = logger->gps_data.current_coord;
								logger->gps_data.last_valid_rtc_time = logger->gps_data.current_rtc_time;
								logger->gps_data.has_last_valid_coord = 1U;													 
						}
												
					
												
						/*
						 * ================================================================
						 * Обработка достоверной GPS-координаты по режимам LOGGER.
						 *
						 * Все проверки GPS и RTC уже успешно пройдены.
						 * ================================================================
						 */

						/*
						 * Формируем единый формат координат и времени:
						 *
						 * 0–3   — latitude, float
						 * 4–7   — longitude, float
						 * 8     — часы
						 * 9     — минуты
						 * 10    — секунды
						 *
						 * Всего 11 байт.
						 */
						uint8_t gps_payload[11];

						memcpy(&gps_payload[0], &fix.latitude, sizeof(float));
						memcpy(&gps_payload[4], &fix.longitude, sizeof(float));

						gps_payload[8]  = fix.hour;
						gps_payload[9]  = fix.minute;
						gps_payload[10] = fix.second;


						/*
						 * ================================================================
						 * MODE_EXPECT — полёт к области разведки.
						 * ================================================================
						 */
						if (logger->req_mode == MODE_EXPECT)
						{
								/*
								 * Проверяем принадлежность текущей координаты
								 * заданной области разведки.
								 */
								if (LoggerStateMachine_IsPointInsideArea(
												logger->gps_data.current_coord,
												&logger->flight_config) != 0U)
								{
										/*
										 * Дрон вошёл в область разведки.
										 *
										 * ID 20 — координата и время, в области.
										 */
										(void)RadioLine_SendPacket(
												20U,
												gps_payload,
												sizeof(gps_payload));

										/*
										 * Здесь должна выполняться запись
										 * пакета № 6 на SD-карту.
										 *
										 * Запись подключим к реализации WORK_SD.
										 */

										/*
										 * Переходим в рабочий режим.
										 */
										logger->req_mode = MODE_WORK;

										/*
										 * Первый полный цикл выполняется немедленно,
										 * независимо от MODE_TIME или MODE_DISTANCE.
										 *
										 * Запускаем цикл заново, начиная с GPS.
										 */
										LoggerStateMachine_StopInterval(logger);

										logger->dev_type = DEV_NONE;
										logger->state = REQ_GPS;
								}
								else
								{
										/*
										 * Дрон пока находится вне области разведки.
										 *
										 * ID 19 — координата и время, вне области.
										 */
										(void)RadioLine_SendPacket(
												19U,
												gps_payload,
												sizeof(gps_payload));

										/*
										 * Здесь должна выполняться запись
										 * пакета № 5 на SD-карту.
										 */

										/*
										 * Возвращаемся к ожиданию следующего
										 * программного интервала GPS.
										 */
										logger->dev_type = DEV_NONE;
										logger->state = COMMAND_INFO;
								}

								break;
						}


						/*
						 * ================================================================
						 * MODE_RETURN — возвращение на базу.
						 * ================================================================
						 */
						if (logger->req_mode == MODE_RETURN)
						{
								/*
								 * В режиме возвращения интересует только маршрут.
								 *
								 * Проверку принадлежности области не выполняем.
								 * Остальные приборы не опрашиваем.
								 *
								 * ID 19 — координата и время.
								 */
								(void)RadioLine_SendPacket(
										19U,
										gps_payload,
										sizeof(gps_payload));

								/*
								 * Здесь должна выполняться запись
								 * пакета № 5 на SD-карту.
								 */

								logger->dev_type = DEV_NONE;
								logger->state = COMMAND_INFO;

								break;
						}


						/*
						 * ================================================================
						 * MODE_WORK — радиационная разведка.
						 * ================================================================
						 */
						if (logger->req_mode == MODE_WORK)
						{
								/*
								 * Первый полный цикл после входа в область
								 * выполняется без проверки времени или расстояния.
								 *
								 * Переход к следующему прибору — LIDAR.
								 */
								if (logger->gps_data.has_last_saved_coord == 0U)
								{
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;

										break;
								}

								/*
								 * ------------------------------------------------------------
								 * MODE_TIME.
								 *
								 * Временной интервал уже проверен обработчиком RTC.
								 * Если GPS запрошен, продолжаем полный цикл.
								 * ------------------------------------------------------------
								 */
								if (logger->work_mode == MODE_TIME)
								{
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;

										break;
								}

								/*
								 * ------------------------------------------------------------
								 * MODE_DISTANCE.
								 *
								 * После получения достоверных GPS-координат
								 * всегда переходим к опросу LIDAR.
								 *
								 * Проверка расстояния от последней сохранённой точки
								 * и изменения высоты будет выполняться в LIDAR_PROC.
								 *
								 * Это позволяет учитывать оба условия:
								 * - достижение порога расстояния;
								 * - достижение порога изменения высоты.
								 *
								 * Достаточно выполнения хотя бы одного условия.
								 * ------------------------------------------------------------
								 */
								if (logger->work_mode == MODE_DISTANCE)
								{
										/*
										 * GPS-данные уже проверены.
										 * Переходим к получению текущей высоты.
										 */
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;

										break;
								}
								
								
								/*
								 * MODE_OPERATOR.
								 *
								 * Запрос GPS был инициирован оператором в составе
								 * полного цикла измерений.
								 *
								 * После получения достоверных координат
								 * переходим к следующему прибору — LIDAR.
								 */
								if (logger->work_mode == MODE_OPERATOR)
								{
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;
										break;
								}
																
						}						

					  break;   
			  } 				
				
				case GPS_ERROR_PROC:
				{
						/*
						 * В MODE_TEST ошибка GPS обрабатывается
						 * непосредственно обработчиком TIM3.
						 */
						if (logger->req_mode == MODE_TEST)
						{
								logger->dev_type = DEV_NONE;
								logger->state = COMMAND_INFO;
								break;
						}

						/*
						 * В MODE_EXPECT, MODE_WORK и MODE_RETURN
						 * учитываем последовательные ошибки GPS.
						 */
						if ((logger->req_mode == MODE_EXPECT) || (logger->req_mode == MODE_WORK) || (logger->req_mode == MODE_RETURN))
						{
								/*
								 * Защита счётчика от переполнения.
								 */
								if (logger->gps_error_counter < UINT8_MAX)
								{
										logger->gps_error_counter++;
								}

								/*
								 * Пока установленный предел не достигнут,
								 * возвращаемся к ожиданию следующего запроса.
								 */
								if (logger->gps_error_counter < LOGGER_DEVICE_ERROR_LIMIT)
								{
										logger->dev_type = DEV_NONE;
										logger->state = COMMAND_INFO;
										break;
								}

								/*
								 * Достигнут предел последовательных ошибок GPS.
								 *
								 * ID14 — GPS вышел из строя,
								 * радиационная разведка невозможна.
								 */
								(void)RadioLine_SendPacket(14U, NULL, 0U);

								logger->req_mode = MODE_ERROR;
								logger->dev_type = DEV_NONE;
								logger->state = COMMAND_INFO;

								break;
						}

						break;
				}			
				
				
				

        /*
			 * ============================================================
			 * REQ_LIDAR
			 *
			 * Запуск нового сеанса приёма данных LIDAR.
			 *
			 * LIDAR опрашивается:
			 * - при предполётном тестировании MODE_TEST;
			 * - при радиационной разведке MODE_WORK.
			 *
			 * В MODE_EXPECT и MODE_RETURN опрос не выполняется.
			 * ============================================================
			 */
			case REQ_LIDAR:
			{
					/*
					 * Защита от запуска LIDAR в режимах,
					 * для которых опрос прибора не предусмотрен.
					 */
					if ((logger->req_mode != MODE_TEST) && (logger->req_mode != MODE_WORK))
					{
							logger->dev_type = DEV_NONE;
							logger->state = COMMAND_INFO;

							break;
					}

					/*
					 * Сбрасываем событие предыдущего приёма
					 * и возможное старое событие таймаута.
					 */
					lidar_info = 0U;
					device_timeout = 0U;

					/*
					 * Указываем, что TIM3 контролирует ответ LIDAR.
					 */
					logger->dev_type = DEV_LIDAR;

					/*
					 * Запускаем приём 45 байтов через LPUART1 + DMA.
					 *
					 * LIDAR_StartReceive() самостоятельно очищает
					 * приёмный буфер и состояние предыдущего опроса.
					 */
					if (LIDAR_StartReceive() != HAL_OK)
					{
							/*
							 * DMA-приём не удалось запустить.
							 * Переходим к обработке ошибки LIDAR.
							 *
							 * TIM3 в этом случае не запускаем.
							 */
							logger->dev_type = DEV_NONE;
							logger->state = LIDAR_ERROR_PROC;

							break;
					}

					/*
					 * Запускаем TIM3 для контроля времени ответа LIDAR.
					 *
					 * Настройка TIM3:
					 * 1 тик = 1 мс;
					 * ARR = 4999;
					 * время ожидания = 5 секунд.
					 */
					__HAL_TIM_SET_COUNTER(&htim3, 0U);

					if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
					{
							/*
							 * Если таймер не запустился,
							 * прекращаем DMA-приём и передаём
							 * управление обработчику ошибки.
							 */
							LIDAR_StopReceive();

							logger->dev_type = DEV_NONE;
							logger->state = LIDAR_ERROR_PROC;

							break;
					}

					/*
					 * Приём успешно запущен.
					 *
					 * Ожидание завершения DMA и обработка
					 * полученных данных будут выполняться
					 * неблокирующе в LIDAR_PROC.
					 */
					logger->state = LIDAR_PROC;

					break;
			}		
				
				
        /* ============================================================
         * LIDAR_PROC — обработка завершённого DMA-приёма LIDAR.
         * Ожидание неблокирующее; таймаут обрабатывается отдельно.
         * ============================================================ */
        case LIDAR_PROC:
        {
            LIDAR_Data measurement;

            /* Пока 45 байтов не приняты, остаёмся в текущем состоянии. */
            if (lidar_info == 0U) { break; }

            /* Приём завершён: отключаем контрольный таймер и приёмник. */
            (void)HAL_TIM_Base_Stop_IT(&htim3);
            device_timeout = 0U;
            logger->dev_type = DEV_NONE;

            /* Разбираем принятые данные до очистки состояния приёмника. */
            uint8_t valid_measurement = LIDAR_Process(&measurement);
            LIDAR_StopReceive();
            if (valid_measurement == 0U)
            {
                lidar_info = 0U;
                if (logger->req_mode == MODE_TEST)
                {
                    /* ID12 — LIDAR неисправен; тестирование прерывается. */
                    (void)RadioLine_SendPacket(12U, NULL, 0U);
                    logger->state = COMMAND_INFO;
                }
                else if (logger->req_mode == MODE_WORK)
                {
                    logger->state = LIDAR_ERROR_PROC;
                }
                else
                {
                    logger->state = COMMAND_INFO;
                }
                break;
            }

            /* Корректный ответ прерывает серию последовательных ошибок. */
            lidar_info = 0U;
            logger->lidar_error_counter = 0U;

            if (logger->req_mode == MODE_TEST)
            {
                /* ID11 — LIDAR протестирован. Далее тестируем БДБГ-15. */
                (void)RadioLine_SendPacket(11U, NULL, 0U);
                logger->state = REQ_BDBG;
                break;
            }

            /* Вне рабочего режима другие действия не выполняем. */
            if (logger->req_mode != MODE_WORK)
            {
                logger->state = COMMAND_INFO;
                break;
            }

            /* MODE_TIME: интервал уже проверен, измерение необходимо. */
            if (logger->work_mode == MODE_TIME)
            {
                logger->bdbg_request = PAED_REQ;
                logger->state = REQ_BDBG;
                break;
            }

            /* START_SPECTRE: запускаем спектр без проверки расстояния/высоты. */
            if (logger->work_mode == START_SPECTRE)
            {
                logger->bdbg_request = START_SPECTRE_REQ;
                logger->state = REQ_BDBG;
                break;
            }

            if ((logger->work_mode == MODE_OPERATOR) || (logger->work_mode == MODE_DISTANCE))
            {
                /* Первое измерение выполняется без сравнения с сохранённой точкой. */
                if ((logger->gps_data.has_last_saved_coord == 0U) ||
                    (logger->has_last_saved_lidar_height == 0U))
                {
                    logger->bdbg_request = PAED_REQ;
                    logger->state = REQ_BDBG;
                    break;
                }

                /* Горизонтальное расстояние между текущей и сохранённой GPS-точками. */
                logger->gps_data.distance_from_saved_m = LoggerStateMachine_CalculateDistance(
                    logger->gps_data.current_coord, logger->gps_data.last_saved_coord);

                /* Абсолютная разность высот, см; вычитание без переполнения. */
                uint16_t height_delta_cm = (measurement.distance_cm >= logger->last_saved_lidar_height_cm)
                    ? (uint16_t)(measurement.distance_cm - logger->last_saved_lidar_height_cm)
                    : (uint16_t)(logger->last_saved_lidar_height_cm - measurement.distance_cm);

                /* Достаточно одного условия: расстояние ИЛИ изменение высоты. */
                if ((logger->gps_data.distance_from_saved_m >= (float)logger->flight_config.saveDistance) ||
                    (height_delta_cm >= LOGGER_MIN_HEIGHT_CHANGE_CM))
                {
                    logger->bdbg_request = PAED_REQ;
                    logger->state = REQ_BDBG;
                }
                else
                {
                    /* В операторском режиме сообщаем, что точка уже измерена.
                     * ID21 — «Измерения для данной точки уже проведены».
                     * NULL, 0U — стандартный Payload по умолчанию.
                     * В MODE_DISTANCE сообщение не отправляем. */
                    if (logger->work_mode == MODE_OPERATOR)
                    {
                        (void)RadioLine_SendPacket(21U, NULL, 0U);
                    }
                    logger->state = COMMAND_INFO;
                }
                break;
            }

            /* Защита от неизвестного рабочего подрежима. */
            logger->state = COMMAND_INFO;
            break;
        }

        case LIDAR_ERROR_PROC:
				{
						/*
						 * Прекращаем ожидание ответа LIDAR и останавливаем приём.
						 * Это безопасно и при ошибке запуска DMA, когда TIM3 не запускался.
						 */
						(void)HAL_TIM_Base_Stop_IT(&htim3);
						LIDAR_StopReceive();

						/* Очищаем признаки завершения приёма и таймаута. */
						lidar_info = 0U;
						device_timeout = 0U;
						logger->dev_type = DEV_NONE;

						/*
						 * MODE_TEST: ошибка предполётной проверки LIDAR.
						 * ID12 — "LIDAR неисправен (нет передачи данных)".
						 * Счётчик ошибок в режиме тестирования не увеличиваем.
						 */
						if (logger->req_mode == MODE_TEST)
						{
								(void)RadioLine_SendPacket(12U, NULL, 0U);
								logger->state = COMMAND_INFO;
								break;
						}

						/*
						 * MODE_WORK: учитываем последовательные ошибки LIDAR.
						 */
						if (logger->req_mode == MODE_WORK)
						{
								/* Защита счётчика от переполнения uint8_t. */
								if (logger->lidar_error_counter < UINT8_MAX)
								{
										logger->lidar_error_counter++;
								}

								/*
								 * Пока предел ошибок не достигнут,
								 * возвращаемся к обработке команд.
								 */
								if (logger->lidar_error_counter < LOGGER_DEVICE_ERROR_LIMIT)
								{
										logger->state = COMMAND_INFO;
										break;
								}

								/*
								 * Достигнут предел последовательных ошибок.
								 * ID13 — "LIDAR вышел из строя, разведка невозможна".
								 * Используется стандартный Payload.
								 */
								(void)RadioLine_SendPacket(13U, NULL, 0U);

								logger->req_mode = MODE_ERROR;
								logger->state = COMMAND_INFO;
								break;
						}

						/* Защита от неожиданного режима работы. */
						logger->state = COMMAND_INFO;
						break;
				}

        case REQ_BDBG:
        {
            /* Приборы не должны одновременно занимать приёмные интерфейсы. */
            GPS_StopReceive();
            LIDAR_StopReceive();
            device_timeout = 0U;
            logger->dev_type = DEV_BDBG;
            if (logger->req_mode == MODE_TEST) logger->bdbg_request = PAED_REQ;
            /* TIM3: 64 MHz / (63999+1) = 1000 Гц; 2000 отсчётов = 2 с. */
            (void)HAL_TIM_Base_Stop_IT(&htim3);
            __HAL_TIM_SET_AUTORELOAD(&htim3, BDBG_RESPONSE_TIMEOUT_MS - 1U);
            __HAL_TIM_SET_COUNTER(&htim3, 0U);
            if (BDBG_StartRequest(&huart4, logger->bdbg_request) == 0U)
            {
                logger->dev_type = DEV_NONE;
                logger->state = (logger->req_mode == MODE_TEST) ? COMMAND_INFO : BDBG_ERROR_PROC;
                if (logger->req_mode == MODE_TEST) (void)RadioLine_SendPacket(8U, NULL, 0U);
                break;
            }
            (void)HAL_TIM_Base_Start_IT(&htim3);
            logger->state = BDBG_PROC;
            break;
        }

        case BDBG_PROC:
        {
            BDBG_RxStatus rx = BDBG_GetRxStatus();
            if (rx == BDBG_RX_WAIT) break;
            (void)HAL_TIM_Base_Stop_IT(&htim3);
            device_timeout = 0U;
            logger->dev_type = DEV_NONE;
            uint8_t ok = (rx == BDBG_RX_READY) &&
                         BDBG_ParseResponse(logger->bdbg_request, &logger->bdbg_data);
            BDBG_StopReceive();
            if (!ok)
            {
                if (logger->req_mode == MODE_TEST)
                {
                    (void)RadioLine_SendPacket(7U, NULL, 0U);
                    logger->state = COMMAND_INFO;
                }
                else logger->state = BDBG_ERROR_PROC;
                break;
            }
            logger->bdbg_error_counter = 0U;
            if (logger->req_mode == MODE_TEST)
            {
                (void)RadioLine_SendPacket(9U, NULL, 0U);
                logger->state = REQ_BATTERY;
                break;
            }
            if (logger->req_mode != MODE_WORK) { logger->state = COMMAND_INFO; break; }
            switch (logger->bdbg_request)
            {
                case PAED_REQ:
                    HAL_Delay(50U);
                    logger->bdbg_request = TEMP_REQ;
                    logger->state = REQ_BDBG;
                    break;
                case TEMP_REQ:
                    logger->state = REQ_BATTERY;
                    break;
                case START_SPECTRE_REQ:
                    memset(logger->spectre_arr, 0, sizeof(logger->spectre_arr));
                    logger->bdbg_spectrum_active = 1U;
                    logger->bdbg_spectrum_started_s = rtc_seconds;
                    logger->bdbg_last_poll_s = rtc_seconds;
                    logger->work_mode = MODE_SPECTRE;
                    logger->bdbg_request = GET_SPECTRE_REQ;
                    logger->state = COMMAND_INFO;
                    break;
                case GET_SPECTRE_REQ:
                {
                    uint8_t near_limit = 0U;
                    for (uint16_t i = 0U; i < BDBG_SPECTRUM_CHANNELS; ++i)
                    {
                        uint32_t sum = (uint32_t)logger->spectre_arr[i] +
                                       (uint32_t)logger->bdbg_data.spectrum[i];
                        /* Saturation avoids uint16_t wraparound on unexpected bursts. */
                        if (sum > UINT16_MAX) sum = UINT16_MAX;
                        logger->spectre_arr[i] = (uint16_t)sum;
                        if (sum >= 55705U) near_limit = 1U;
                    }
                    logger->bdbg_last_poll_s = rtc_seconds;
                    /* Завершение и отправка спектра пока НЕ реализованы:
                     * требуется отдельная интеграция с протоколом RadioLine/SD. */
                    (void)near_limit;
                    logger->state = COMMAND_INFO;
                    break;
                }
                default: logger->state = COMMAND_INFO; break;
            }
            break;
        }

        case BDBG_ERROR_PROC:
        {
            (void)HAL_TIM_Base_Stop_IT(&htim3);
            BDBG_StopReceive();
            device_timeout = 0U;
            logger->dev_type = DEV_NONE;
            if (logger->req_mode == MODE_WORK)
            {
                if (logger->bdbg_error_counter < UINT8_MAX) logger->bdbg_error_counter++;
                if (logger->bdbg_error_counter >= LOGGER_DEVICE_ERROR_LIMIT)
                {
                    (void)RadioLine_SendPacket(10U, NULL, 0U);
                    logger->req_mode = MODE_ERROR;
                }
            }
            logger->state = COMMAND_INFO;
            break;
        }

        case REQ_BATTERY:
        case BATTERY_PROC:
        case BATTERY_ERROR_PROC:

        case WORK_SD:
        {
            break;
        }

        /*
         * Защита от некорректного состояния.
         */
        default:
        {
            logger->state = COMMAND_INFO;

            break;
        }
    }
}