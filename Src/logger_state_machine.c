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
#include "sd_logger.h"

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


static HAL_StatusTypeDef RadioLine_SendPacket(uint8_t packet_id, const uint8_t *payload, uint16_t payload_length);


/*
 * ============================================================
 * ОБМЕН ДАННЫМИ С ADC1
 * ============================================================
 *
 * Переменные определены в main.c.
 *
 * Их значения изменяются обработчиками прерываний ADC1
 * и используются основной стейт-машиной LOGGER.
 */
extern ADC_HandleTypeDef hadc1;

extern volatile uint16_t battery_adc_raw;
extern volatile uint8_t battery_adc_ready;
extern volatile uint8_t battery_adc_error;
extern volatile uint8_t battery_adc_active;


		
		
/*
 * Размер полезных данных радиопакета ID23:
 *
 * 1024 канала по 2 байта = 2048 байт;
 * время набора             =    2 байта;
 * ПАЕД                     =    4 байта;
 * состояние БДБГ           =    1 байт.
 *
 * Итого: 2055 байт.
 */
#define RADIOLINE_SPECTRUM_PAYLOAD_SIZE 2055U

/*
 * Статический буфер для очередной порции спектра.
 *
 * Не размещаем массив размером 2055 байт на стеке.
 * Динамическую память не используем.
 */
static uint8_t radioline_spectrum_payload[ RADIOLINE_SPECTRUM_PAYLOAD_SIZE ];


/*
 * Размер полезной нагрузки радиопакета ID27.
 *
 * Дата и время:        10 байт ASCII.
 * Номер спектра:        2 байта uint16_t.
 * Широта:               4 байта float.
 * Долгота:              4 байта float.
 * Высота LIDAR:         2 байта uint16_t.
 *
 * Итого: 22 байта.
 */
#define RADIOLINE_SPECTRUM_START_PAYLOAD_SIZE 22U



/*
 * Размер полезной нагрузки итогового спектра ID23.
 *
 * 10 байт — дата и время ASCII;
 *  2 байта — номер спектра;
 *  4 байта — широта GPS;
 *  4 байта — долгота GPS;
 *  2 байта — высота LIDAR;
 * 2048 байт — 1024 канала спектра.
 *
 * Всего: 2070 байт.
 */
#define SD_SPECTRUM_PAYLOAD_SIZE 2070U

/*
 * Статический буфер полного накопленного спектра.
 *
 * Не размещаем массив размером 2070 байт на стеке.
 * Динамическая память не используется.
 */
static uint8_t sd_spectrum_payload[SD_SPECTRUM_PAYLOAD_SIZE];




/*
 * Формирует полезную нагрузку ID22 (данные разведки).
 *
 * Размер: 31 байт.
 * Все многобайтовые значения записываются в Little Endian.
 *
 * Функция не отправляет пакет и не записывает SD-карту.
 * Она только заполняет переданный буфер.
 */
#define ID22_PAYLOAD_SIZE 31U

static void Logger_BuildID22Payload(
    const LoggerStateMachine *logger,
    uint8_t payload[ID22_PAYLOAD_SIZE])
{
    uint16_t value16;

    /* 0–3: ПАЕД, float. */
    memcpy(&payload[0],
           &logger->bdbg_data.paed_usv_h,
           sizeof(float));

    /* 4–5: температура БДБГ, uint16_t. */
    value16 = logger->bdbg_data.temperature_raw;
    payload[4] = (uint8_t)(value16 & 0xFFU);
    payload[5] = (uint8_t)((value16 >> 8U) & 0xFFU);

    /* 6: состояние датчиков и валидность данных БДБГ. */
    payload[6] = logger->bdbg_data.status;

    /* 7–8: высота LIDAR, uint16_t. */
    value16 = logger->lidar_data.distance_cm;
    payload[7] = (uint8_t)(value16 & 0xFFU);
    payload[8] = (uint8_t)((value16 >> 8U) & 0xFFU);

    /* 9–10: мощность сигнала LIDAR, uint16_t. */
    value16 = logger->lidar_data.strength;
    payload[9] = (uint8_t)(value16 & 0xFFU);
    payload[10] = (uint8_t)((value16 >> 8U) & 0xFFU);

    /* 11–14: широта GPS, float. */
    memcpy(&payload[11],
           &logger->gps_data.current_coord.lat,
           sizeof(float));

    /* 15–18: долгота GPS, float. */
    memcpy(&payload[15],
           &logger->gps_data.current_coord.lon,
           sizeof(float));

    /*
     * 19–22: высота GPS над уровнем моря.
     * Временно 0.0f до реализации обработки $GNGGA.
     */
    {
        const float gps_altitude = 0.0f;
        memcpy(&payload[19], &gps_altitude, sizeof(float));
    }

    /*
     * 23: количество спутников GPS.
     * Временно 0 до реализации обработки $GNGGA.
     */
    payload[23] = 0U;

    /*
     * 24–27: напряжение батареи, float.
     * Пока поле logger->battery_voltage содержит
     * согласованное отладочное значение 1.0f.
     */
    memcpy(&payload[24],
           &logger->battery_voltage,
           sizeof(float));

    /*
     * 28–30: время текущего GPS-измерения.
     * Три отдельных байта: HH, MM, SS.
     */
    payload[28] = logger->gps_data.current_gps_time.hour;
    payload[29] = logger->gps_data.current_gps_time.minute;
    payload[30] = logger->gps_data.current_gps_time.second;
}




















/*
 * Формирует и записывает итоговый накопленный
 * спектр в открытый CSV-файл SD-карты.
 *
 * Формат полезной нагрузки полностью соответствует
 * согласованному формату ID23 для SD.
 *
 * Функция не передаёт данные по RadioLine.
 * Функция не изменяет состояние стейт-машины.
 */
static SD_LoggerResult SD_WriteAccumulatedSpectrum(
    const LoggerStateMachine *logger)
{
    /*
     * Проверяем указатель на структуру LOGGER.
     */
    if (logger == NULL)
    {
        return SD_LOGGER_INVALID_ARGUMENT;
    }

    /*
     * Проверяем наличие зафиксированных
     * координат GPS и высоты LIDAR.
     */
    if (logger->spectrum_start_data_valid == 0U)
    {
        return SD_LOGGER_INVALID_ARGUMENT;
    }

    /*
     * Проверяем разрешение записи на SD-карту.
     */
    if (SD_Logger_IsEnabled() == 0U)
    {
        return SD_LOGGER_DISABLED;
    }

    /*
     * Проверяем, что файл полёта открыт.
     */
    if (SD_Logger_IsFileOpen() == 0U)
    {
        return SD_LOGGER_NO_FILE;
    }

    /*
     * Получаем зафиксированную дату и время
     * начала набора спектра.
     */
    const GPS_TimeStamp *time =
        &logger->spectrum_start_time;

    /*
     * Подготавливаем дату и время:
     *
     * YYMMDDHHMM
     *
     * Каждое поле представлено двумя
     * ASCII-символами.
     */
    const uint8_t values[5] =
    {
        (uint8_t)(time->year % 100U),
        time->month,
        time->day,
        time->hour,
        time->minute
    };

    /*
     * Заполняем байты 0–9.
     */
    for (uint8_t i = 0U; i < 5U; ++i)
    {
        sd_spectrum_payload[2U * i] =
            (uint8_t)('0' + values[i] / 10U);

        sd_spectrum_payload[2U * i + 1U] =
            (uint8_t)('0' + values[i] % 10U);
    }

    /*
     * Байты 10–11:
     * порядковый номер спектра.
     *
     * uint16_t, Little Endian.
     */
    const uint16_t sequence =
        logger->spectrum_sequence_number;

    sd_spectrum_payload[10] =
        (uint8_t)(sequence & 0xFFU);

    sd_spectrum_payload[11] =
        (uint8_t)((sequence >> 8U) & 0xFFU);

    /*
     * Байты 12–15:
     * широта GPS, float IEEE-754.
     */
    memcpy(
        &sd_spectrum_payload[12],
        &logger->spectrum_start_coord.lat,
        sizeof(float)
    );

    /*
     * Байты 16–19:
     * долгота GPS, float IEEE-754.
     */
    memcpy(
        &sd_spectrum_payload[16],
        &logger->spectrum_start_coord.lon,
        sizeof(float)
    );

    /*
     * Байты 20–21:
     * высота LIDAR в сантиметрах.
     *
     * uint16_t, Little Endian.
     */
    const uint16_t height =
        logger->spectrum_start_height_cm;

    sd_spectrum_payload[20] =
        (uint8_t)(height & 0xFFU);

    sd_spectrum_payload[21] =
        (uint8_t)((height >> 8U) & 0xFFU);

    /*
     * Байты 22–2069:
     * накопленный спектр LOGGER.
     *
     * 1024 канала по 2 байта.
     *
     * Каждый канал передаётся в Little Endian:
     * младший байт, затем старший.
     */
    for (uint16_t channel = 0U;
         channel < BDBG_SPECTRUM_CHANNELS;
         ++channel)
    {
        const uint16_t value =
            logger->spectre_arr[channel];

        const uint16_t offset =
            (uint16_t)(22U + 2U * channel);

        sd_spectrum_payload[offset] =
            (uint8_t)(value & 0xFFU);

        sd_spectrum_payload[offset + 1U] =
            (uint8_t)((value >> 8U) & 0xFFU);
    }

    /*
     * Записываем итоговый спектр на SD.
     *
     * ID23 передаётся десятичным числом 23.
     * SD_Logger_WritePacket() преобразует его
     * в HEX "17".
     *
     * Эта же функция преобразует каждый байт
     * полезной нагрузки в HEX, добавляет запятые
     * и выполняет f_sync().
     *
     * Результат возвращаем вызывающему коду
     * для последующей обработки.
     */
    return SD_Logger_WritePacket(
        23U,
        sd_spectrum_payload,
        SD_SPECTRUM_PAYLOAD_SIZE
    );
}



/*
 * ============================================================
 * Открытие CSV-файла для нового FlightConfig.
 *
 * Вызывается только после успешной проверки ID04.
 *
 * Время берём из RTC. При ошибке чтения RTC
 * используем последнее сохранённое время GPS/RTC.
 *
 * Ошибки SD не меняют состояние LOGGER.
 * ============================================================
 */
static void SD_OpenFlightFile(LoggerStateMachine *logger)
{
    GPS_TimeStamp flight_time = {0};

    /*
     * Проверяем указатель.
     */
    if (logger == NULL)
    {
        return;
    }

    /*
     * Если SD отключена, не пытаемся
     * создавать новый файл.
     */
    if (SD_Logger_IsEnabled() == 0U)
    {
        return;
    }

    /*
     * Начальное значение — последнее время,
     * сохранённое в структуре LOGGER.
     */
    flight_time = logger->gps_data.current_rtc_time;

    /*
     * Пытаемся получить актуальное время RTC.
     */
    GPS_TimeStamp current_time = {0};

    if (RTC_Service_GetTime(&current_time) == HAL_OK)
    {
        flight_time = current_time;

        /*
         * Обновляем сохранённое время,
         * чтобы оно было доступно при
         * последующей ошибке чтения RTC.
         */
        logger->gps_data.current_rtc_time = current_time;
    }

    /*
     * Создаём файл:
     *
     * 0:/YYMMDD/HHMMSS.CSV
     *
     * SD_Logger_OpenFlight():
     * - закрывает предыдущий открытый файл;
     * - создаёт каталог даты;
     * - создаёт новый файл без перезаписи;
     * - регистрирует ошибки FatFs.
     *
     * Год GPS_TimeStamp хранится полностью
     * (например, 2026), поэтому берём
     * последние две цифры.
     */
    SD_LoggerResult result = SD_Logger_OpenFlight(
        (uint8_t)(flight_time.year % 100U),
        flight_time.month,
        flight_time.day,
        flight_time.hour,
        flight_time.minute,
        flight_time.second
    );

    /*
     * Ошибка SD не должна прерывать
     * выполнение FlightConfig.
     *
     * Состояние стейт-машины не изменяем.
     * Ошибки FatFs обрабатывает sd_logger.
     */
    (void)result;
}













 

/*
 * Формирование и передача радиопакета ID27.
 *
 * Пакет содержит данные начала набора спектра.
 * Вызывается после успешного подтверждения
 * команды START_SPECTRE_REQ прибором БДБГ-15.
 *
 * Формат полезной нагрузки (22 байта):
 *
 *  0–1   Год (YY), ASCII
 *  2–3   Месяц (MM), ASCII
 *  4–5   День (DD), ASCII
 *  6–7   Часы (HH), ASCII
 *  8–9   Минуты (MM), ASCII
 * 10–11  Порядковый номер спектра, uint16_t
 * 12–15  Широта GPS, float
 * 16–19  Долгота GPS, float
 * 20–21  Высота LIDAR, uint16_t (сантиметры)
 *
 * Все бинарные числовые поля передаются
 * в формате Little Endian.
 *
 * Функция не изменяет состояние стейт-машины
 * и не записывает данные на SD-карту.
 */
static HAL_StatusTypeDef RadioLine_SendSpectrumStart(
    const LoggerStateMachine *logger)
{
    /*
     * Локальный буфер полезной нагрузки.
     *
     * Размер определён константой:
     * RADIOLINE_SPECTRUM_START_PAYLOAD_SIZE = 22.
     */
    uint8_t payload[RADIOLINE_SPECTRUM_START_PAYLOAD_SIZE];

    /*
     * Проверяем указатель на структуру LOGGER
     * и наличие зафиксированных GPS/LIDAR данных.
     */
    if ((logger == NULL) ||
        (logger->spectrum_start_data_valid == 0U))
    {
        return HAL_ERROR;
    }

    /*
     * Получаем дату и время начала набора спектра.
     *
     * Время должно быть предварительно сохранено
     * из RTC в spectrum_start_time.
     */
    const GPS_TimeStamp *time =
        &logger->spectrum_start_time;

    /*
     * Подготавливаем пять числовых полей даты
     * и времени для преобразования в ASCII:
     *
     * YY, MM, DD, HH, MM.
     *
     * Полный год (например, 2026)
     * преобразуется в последние две цифры (26).
     */
    const uint8_t values[5] =
    {
        (uint8_t)(time->year % 100U),
        time->month,
        time->day,
        time->hour,
        time->minute
    };

    /*
     * Преобразуем каждое значение в два
     * ASCII-символа с ведущим нулём.
     *
     * Например:
     *  3 -> '0', '3'
     * 26 -> '2', '6'
     */
    for (uint8_t i = 0U; i < 5U; ++i)
    {
        payload[2U * i] =
            (uint8_t)('0' + values[i] / 10U);

        payload[2U * i + 1U] =
            (uint8_t)('0' + values[i] % 10U);
    }

    /*
     * Байты 10–11:
     * порядковый номер спектра.
     *
     * uint16_t, Little Endian:
     * сначала младший байт, затем старший.
     */
    const uint16_t sequence =
        logger->spectrum_sequence_number;

    payload[10] =
        (uint8_t)(sequence & 0xFFU);

    payload[11] =
        (uint8_t)((sequence >> 8U) & 0xFFU);

    /*
     * Байты 12–15:
     * широта GPS в десятичных градусах.
     *
     * Тип float, IEEE-754, Little Endian.
     *
     * В структуре Coord поле широты называется lat.
     */
    memcpy(
        &payload[12],
        &logger->spectrum_start_coord.lat,
        sizeof(float)
    );

    /*
     * Байты 16–19:
     * долгота GPS в десятичных градусах.
     *
     * Тип float, IEEE-754, Little Endian.
     *
     * В структуре Coord поле долготы называется lon.
     */
    memcpy(
        &payload[16],
        &logger->spectrum_start_coord.lon,
        sizeof(float)
    );

    /*
     * Байты 20–21:
     * высота LIDAR в сантиметрах.
     *
     * Тип uint16_t, Little Endian.
     */
    const uint16_t height =
        logger->spectrum_start_height_cm;

    payload[20] =
        (uint8_t)(height & 0xFFU);

    payload[21] =
        (uint8_t)((height >> 8U) & 0xFFU);

    /*
     * Передаём сформированный пакет ID27
     * через существующий механизм RadioLine.
     *
     * Обрамление и контрольная сумма
     * формируются функцией RadioLine_SendPacket().
     */
    return RadioLine_SendPacket(
        27U,
        payload,
        RADIOLINE_SPECTRUM_START_PAYLOAD_SIZE
    );
}


/*
 * Формирует и передаёт радиопакет ID23.
 *
 * Передаётся именно очередная порция спектра БДБГ,
 * а не накопленный массив spectre_arr[].
 *
 * Функция вызывается после успешной проверки ответа
 * GET_SPECTRE_REQ.
 */
static HAL_StatusTypeDef RadioLine_SendSpectrum(const BDBG_Data *data)
{
    if (data == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * Записываем 1024 канала спектра.
     *
     * Каждый канал — uint16_t.
     * Порядок байтов: LSB, затем MSB.
     */
    for (uint16_t i = 0U;
         i < BDBG_SPECTRUM_CHANNELS;
         ++i)
    {
        uint16_t value = data->spectrum[i];

        radioline_spectrum_payload[2U * i] =
            (uint8_t)(value & 0xFFU);

        radioline_spectrum_payload[2U * i + 1U] =
            (uint8_t)((value >> 8U) & 0xFFU);
    }

    /*
     * Время набора из ответа БДБГ-15.
     *
     * Исходные байты ответа: 2054–2055.
     * Смещение в Payload ID23: 2048–2049.
     */
    radioline_spectrum_payload[2048] =
        (uint8_t)(data->acquisition_time & 0xFFU);

    radioline_spectrum_payload[2049] =
        (uint8_t)((data->acquisition_time >> 8U) & 0xFFU);

    /*
     * ПАЕД — 4 байта float IEEE-754.
     *
     * STM32L4 использует Little Endian.
     * memcpy позволяет избежать небезопасного
     * преобразования указателей разных типов.
     */
    memcpy(
        &radioline_spectrum_payload[2050],
        &data->paed_usv_h,
        sizeof(float)
    );

    /*
     * Состояние низко- и высокочувствительного
     * датчиков и валидность данных БДБГ.
     *
     * Передаём исходный байт status без изменения
     * его битового представления.
     */
    radioline_spectrum_payload[2054] = data->status;

    /*
     * ID23 — очередная порция спектра.
     */
    return RadioLine_SendPacket( 23U,
        radioline_spectrum_payload,
        RADIOLINE_SPECTRUM_PAYLOAD_SIZE
    );
}			
		
		

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
     * АККУМУЛЯТОР — ТАЙМАУТ ИЗМЕРЕНИЙ ADC1
     * ============================================================
     *
     * Общий таймер TIM3 контролирует продолжительность
     * полного цикла измерений аккумулятора.
     *
     * За отведённое время должны успешно завершиться
     * все 10 преобразований ADC.
     *
     * Если TIM3 сработал, считаем текущий цикл
     * измерения неуспешным.
     */
    if (logger->dev_type == DEV_BATTERY)
    {
        /*
         * Сбрасываем признак активного прибора.
         *
         * TIM3 уже остановлен, а device_timeout
         * уже сброшен в начале этой функции.
         */
        logger->dev_type = DEV_NONE;

        /*
         * Отключаем приём результатов рабочего
         * измерения ADC.
         *
         * Это необходимо, чтобы возможное запоздавшее
         * прерывание не записало результат после
         * завершения текущего цикла.
         *
         * Сам ADC будет остановлен при обработке
         * состояния BATTERY_ERROR_PROC.
         */
         battery_adc_active = 0U;

        /*
         * Переходим в состояние обработки ошибки ADC.
         *
         * Здесь не увеличиваем batt_error_counter:
         * увеличение выполняется исключительно
         * в BATTERY_ERROR_PROC, один раз за ошибочный
         * цикл измерений.
         *
         * Это правило действует как в MODE_TEST,
         * так и в MODE_WORK.
         */
        logger->state = BATTERY_ERROR_PROC;

        /*
         * Таймаут полностью обработан.
         *
         * Основной цикл завершит текущий вызов
         * LoggerStateMachine_Process().
         *
         * На следующем проходе будет выполнено
         * состояние BATTERY_ERROR_PROC.
         */
        return 1U;
    }

    /*
     * ============================================================
     * ОСТАЛЬНЫЕ ПРИБОРЫ
     * ============================================================
     *
     * Обработка неизвестного dev_type пока
     * остаётся без изменений.
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
		
		/*
		 * После включения LOGGER спектральный режим неактивен.
		 * Процедура остановки спектра также не выполняется.
		 */
		logger->bdbg_spectrum_active = 0U;
		logger->bdbg_spectrum_stopping = 0U;		
		logger->bdbg_spectrum_error_shutdown = 0U;
		logger->bdbg_spectrum_started_s = 0U;
		logger->bdbg_last_poll_s = 0U;

		/*
		 * Обнуляем накопленный спектр.
		 * Новый набор также будет очищать этот массив
		 * после успешного START_SPECTRE_REQ.
		 */
		memset(logger->spectre_arr, 0, sizeof(logger->spectre_arr));


    
		
    /*
     * Инициализация счётчиков последовательных
     * ошибок приборов.
     *
     * После включения LOGGER ошибок ещё нет.
     */
    logger->bdbg_error_counter = 0U;
    logger->gps_error_counter = 0U;
    logger->lidar_error_counter = 0U;

    /*
     * ============================================================
     * ПОВТОРНЫЙ ЗАПУСК НАБОРА СПЕКТРА БДБГ
     * ============================================================
     *
     * После включения LOGGER повторный запуск спектра
     * не запланирован.
     *
     * Признак ожидания сброшен.
     */
    logger->bdbg_start_retry_pending = 0U;

    /*
     * Время последней ошибки START_SPECTRE_REQ
     * первоначально равно нулю.
     *
     * Это значение используется только при установленном
     * признаке bdbg_start_retry_pending.
     */
    logger->bdbg_start_retry_tick = 0U;


		/*
		 * Ошибки выполнения измерений ADC аккумулятора.
		 */
		logger->batt_error_counter = 0U;

		/*
		 * Последовательные успешные измерения,
		 * показавшие критически низкое напряжение.
		 *
		 * Этот счётчик независим от ошибок ADC.
		 */
		logger->batt_low_voltage_counter = 0U;

		/*
		 * Ошибки чтения RTC.
		 */
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

		
    /*
     * ============================================================
     * ОЖИДАЮЩАЯ КОНФИГУРАЦИЯ НОВОЙ ТЕРРИТОРИИ
     * ============================================================
     *
     * При включении LOGGER новая конфигурация ID04
     * ещё не получена.
     *
     * Полностью очищаем структуру, включая массив
     * координат вершин многоугольника.
     */
		memset(&logger->pending_flight_config,
				 0,
				 sizeof(logger->pending_flight_config));

		/*
		* Смена территории первоначально не выполняется.
		*
		* Флаг будет установлен только после успешной
		* проверки новой команды ID04.
		*/
		logger->new_flight_config_pending = 0U;	
		


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
				
		/*
		* Начальные значения GPS-координат и высоты LIDAR
		* для набора спектра.
		*
		* После запуска LOGGER данные отсутствуют.
		*/
    logger->spectrum_start_coord = (Coord){0};
    logger->spectrum_start_height_cm = 0U;
    logger->spectrum_start_data_valid = 0U;		

		/*
		 * Порядковый номер спектра.
		 * При запуске LOGGER счётчик обнуляется.
		 */
		logger->spectrum_sequence_number = 0U;

		/*
		 * Дата и время начала набора спектра.
		 * Будут заполнены после подтверждения START_SPECTRE_REQ.
		 */
		logger->spectrum_start_time = (GPS_TimeStamp){0};

		
		/*
		 * ============================================================
		 * НАЧАЛЬНЫЕ ДАННЫЕ АККУМУЛЯТОРА
		 * ============================================================
		 */

		/*
		 * Пока оставляем существующее начальное значение
		 * напряжения аккумулятора.
		 *
		 * После первого успешного рабочего измерения
		 * оно будет заменено фактическим напряжением.
		 *
		 * В дальнейшем отдельно проверим, как отличать
		 * отсутствие измерения от достоверного значения.
		 */
		logger->battery_voltage = 1.0f;

		/*
		 * При включении LOGGER результаты рабочих
		 * измерений ADC ещё не накоплены.
		 */
		logger->battery_adc_sum = 0U;

		/*
		 * Ни одного рабочего преобразования ADC
		 * пока не выполнено.
		 */
		logger->battery_adc_count = 0U;		
		
		
		/* При запуске подготовка спектра не ожидает повтора. */
		logger->spectrum_prepare_retry = 0U;

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
				 * Рассчитываем минимальное время передачи пакета
				 * исходя из текущей скорости USART2.
				 *
				 * Для формата 8N1 один байт занимает 10 бит:
				 * стартовый бит + 8 бит данных + стоповый бит.
				 *
				 * Дополнительно предусматриваем запас 500 мс
				 * на выполнение передачи.
				 *
				 * Расчёт выполняется в uint64_t, чтобы исключить
				 * переполнение при умножении длины на 10000.
				 */
				uint32_t tx_timeout_ms = 1000U;

				if (huart2.Init.BaudRate != 0U)
				{
						uint64_t required_ms =
								(((uint64_t)packet_length * 10000ULL) +
								 (uint64_t)huart2.Init.BaudRate - 1ULL) /
								(uint64_t)huart2.Init.BaudRate;

						required_ms += 500ULL;

						if (required_ms > UINT32_MAX)
						{
								return HAL_ERROR;
						}

						tx_timeout_ms = (uint32_t)required_ms;
				}

				/*
				 * Рассчитываем минимальное время передачи пакета
				 * исходя из текущей скорости USART2.
				 *
				 * Для формата 8N1 один байт занимает 10 бит:
				 * стартовый бит + 8 бит данных + стоповый бит.
				 *
				 * Дополнительно предусматриваем запас 500 мс
				 * на выполнение передачи.
				 *
				 * Расчёт выполняется в uint64_t, чтобы исключить
				 * переполнение при умножении длины на 10000.
				 */
				
				if (huart2.Init.BaudRate != 0U)
				{
						uint64_t required_ms =
								(((uint64_t)packet_length * 10000ULL) +
								 (uint64_t)huart2.Init.BaudRate - 1ULL) /
								(uint64_t)huart2.Init.BaudRate;

						required_ms += 500ULL;

						if (required_ms > UINT32_MAX)
						{
								return HAL_ERROR;
						}

						tx_timeout_ms = (uint32_t)required_ms;
				}

				/*
				 * Передаём одну копию пакета.
				 *
				 * Таймаут теперь зависит от размера пакета
				 * и скорости USART2.
				 */
				status = HAL_UART_Transmit(
						&huart2,
						radioline_tx_buffer,
						(uint16_t)packet_length,
						tx_timeout_ms
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
		
   
    /*
     * ============================================================
     * Закрытие CSV-файла при завершении радиационной разведки.
     * ============================================================
     *
     * MODE_RETURN:
     *   Разведка завершена, аппарат возвращается на базу.
     *   Новые данные на SD-карту больше не записываем.
     *   ID19 продолжает передаваться только по RadioLine.
     *
     * MODE_ERROR:
     *   Разведка прекращена из-за аварийной ситуации.
     *   Запись на SD-карту необходимо завершить
     *   независимо от причины аварии.
     *
     * Исключение для MODE_ERROR:
     *   Если стейт-машина находится в WORK_SD,
     *   закрывать файл пока нельзя.
     *
     *   WORK_SD должен сначала сохранить последний
     *   накопленный спектр ID23.
     *
     *   После завершения WORK_SD состояние изменится
     *   на COMMAND_INFO, и файл будет закрыт
     *   при следующем вызове этой функции.
     */
    if ((logger->req_mode == MODE_RETURN) ||
        ((logger->req_mode == MODE_ERROR) &&
         (logger->state != WORK_SD)))
    {
       
        /*
         * Закрываем CSV-файл только при выполнении
         * двух условий:
         *
         * 1. Файл действительно открыт.
         *
         * 2. Работа с SD-картой не была отключена
         *    вследствие предыдущей ошибки.
         *
         * Если SD-карта уже отключена,
         * SD_Logger_Close() возвращает
         * SD_LOGGER_DISABLED, не выполняя f_close().
         *
         * Дополнительная проверка предотвращает
         * бесполезные повторные вызовы этой функции
         * при каждом проходе стейт-машины.
         */
        if ((SD_Logger_IsFileOpen() != 0U) &&
            (SD_Logger_IsEnabled() != 0U))

        {
            /*
             * Закрываем CSV-файл.
             *
             * SD_Logger_Close() выполняет f_close().
             *
             * При ошибке FatFs модуль SD:
             *   - регистрирует ошибку;
             *   - запрещает дальнейшую запись;
             *   - формирует событие для ID16.
             *
             * Ошибка закрытия не должна блокировать
             * дальнейшую работу стейт-машины.
             */
            (void)SD_Logger_Close();
        }
    }
		
		
    /*
     * ============================================================
     * Контроль ошибок SD-карты.
     * ============================================================
     *
     * SD_Logger_TakeErrorEvent() возвращает:
     *
     * 0 — нового события ошибки нет;
     * 1 — обнаружена ошибка SD-карты.
     *
     * После чтения событие сбрасывается.
     * Поэтому повторно отправлять ID16 из-за
     * одного и того же события не будем.
     *
     * Проверка выполняется независимо от текущего
     * состояния и режима работы LOGGER.
     */
			if (SD_Logger_TakeErrorEvent() != 0U)
			{
					/*
					 * ID16 — ошибка SD-карты.
					 *
					 * Согласно протоколу RadioLine,
					 * даже если дополнительных данных нет,
					 * Payload должен содержать один байт 0x01.
					 *
					 * Поле Length содержит значение 1
					 * и занимает два байта в радиопакете.
					 */
					const uint8_t payload = 1U;

					/*
					 * Отправляем сообщение об ошибке SD-карты
					 * на наземную станцию.
					 *
					 * ID     = 16
					 * Length = 1
					 * Payload = 0x01
					 *
					 * Текущее состояние стейт-машины не изменяем.
					 */
					(void)RadioLine_SendPacket(
							16U,
							&payload,
							1U
					);
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
				
				
				else if (logger->state == FLIGHT_CHANGE)
				{
						/*
						 * DEBUG ID84.
						 *
						 * Сообщение о входе в состояние смены
						 * территории радиационной разведки.
						 *
						 * Передаётся только при изменении состояния,
						 * поскольку выше выполняется сравнение
						 * с debug_previous_state.
						 *
						 * Используем существующий механизм
						 * формирования пакетов RadioLine.
						 */
						(void)RadioLine_SendPacket(84U, NULL, 0U);
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
																
								/*
								 * ============================================================
								 * ПОВТОР GPS/LIDAR ПРИ ПОДГОТОВКЕ СПЕКТРА
								 * ============================================================
								 *
								 * После первой или второй ошибки прибора
								 * обработчик возвращает управление в COMMAND_INFO.
								 *
								 * Здесь продолжаем подготовку спектра, повторяя
								 * только тот прибор, на котором возникла ошибка.
								 *
								 * Команды RadioLine имеют приоритет:
								 * при com_info != 0U этот участок не выполняется.
								 */
								if (logger->spectrum_prepare_retry != 0U)
								{
										/*
										 * Повтор разрешён исключительно при подготовке
										 * запуска спектра в рабочем режиме.
										 *
										 * После перехода в MODE_ERROR повтор запрещён.
										 */
										if ((logger->req_mode != MODE_WORK) ||
												(logger->work_mode != START_SPECTRE))
										{
												logger->spectrum_prepare_retry = 0U;
												break;
										}

										/*
										 * Сохраняем требуемый прибор и сразу очищаем
										 * признак, чтобы повтор не запускался дважды.
										 */
										const uint8_t retry_device =
												logger->spectrum_prepare_retry;

										logger->spectrum_prepare_retry = 0U;

										/*
										 * Повтор GPS после:
										 * - ошибки связи с GPS;
										 * - ошибки чтения RTC;
										 * - обнаружения GPS-спуфинга.
										 */
										if (retry_device == 1U)
										{
												logger->dev_type = DEV_NONE;
												logger->state = REQ_GPS;
												break;
										}

										/*
										 * Повтор LIDAR после:
										 * - ошибки запуска приёма;
										 * - таймаута;
										 * - некорректного ответа.
										 */
										if (retry_device == 2U)
										{
												logger->dev_type = DEV_NONE;
												logger->state = REQ_LIDAR;
												break;
										}

										/*
										 * Неизвестное значение уже сброшено.
										 * Остаёмся в COMMAND_INFO.
										 */
										break;
								}
							
								/*
								 * ============================================================
								 * ОЖИДАНИЕ ПОВТОРНОГО START_SPECTRE_REQ
								 * ============================================================
								 *
								 * Проверка выполняется только при отсутствии
								 * завершённого пакета RadioLine.
								 *
								 * Если команда с земли получена, она будет
								 * обработана существующим кодом COMMAND_INFO.
								 */
								if (logger->bdbg_start_retry_pending != 0U)
								{
										/*
										 * Проверяем, сохраняются ли условия,
										 * при которых разрешён повторный запуск спектра.
										 *
										 * Если режим изменился, ожидание отменяется.
										 */
										if ((logger->req_mode != MODE_WORK) ||
												(logger->work_mode != START_SPECTRE) ||
												(logger->bdbg_request != START_SPECTRE_REQ))
										{
												/*
												 * Повторный запуск больше не требуется.
												 */
												logger->bdbg_start_retry_pending = 0U;
												logger->bdbg_start_retry_tick = 0U;
										}
										else
										{
												/*
												 * Определяем время, прошедшее после
												 * последней неудачной попытки.
												 *
												 * Беззнаковое вычитание позволяет корректно
												 * работать при переполнении HAL_GetTick().
												 */
												const uint32_t elapsed_ms =
														(uint32_t)(
																HAL_GetTick() -
																logger->bdbg_start_retry_tick
														);

												/*
												 * Проверяем истечение интервала ожидания.
												 */
												if (elapsed_ms >=
														LOGGER_BDBG_START_RETRY_INTERVAL_MS)
												{
														/*
														 * Интервал истёк.
														 *
														 * Сбрасываем признак ожидания до перехода
														 * в REQ_BDBG, чтобы исключить повторное
														 * срабатывание этого механизма.
														 */
														logger->bdbg_start_retry_pending = 0U;
														logger->bdbg_start_retry_tick = 0U;

														/*
														 * Тип запроса остаётся START_SPECTRE_REQ.
														 *
														 * Следующий вызов стейт-машины выполнит
														 * повторную отправку команды БДБГ.
														 */
														logger->state = REQ_BDBG;
												}

												/*
												 * Пока ожидается повторный запуск,
												 * остальные автоматические переходы
												 * из COMMAND_INFO не выполняются.
												 *
												 * Ожидание неблокирующее.
												 */
												break;
										}
								}

								/*
								 * ============================================================
								 * ШТАТНАЯ ОБРАБОТКА СПЕКТРАЛЬНОГО РЕЖИМА
								 * ============================================================
								 *
								 * Ниже полностью сохранён существующий алгоритм
								 * периодического получения спектра.
								 *
								 * Он работает только после успешного запуска
								 * набора спектра.
								 */
								if ((logger->req_mode == MODE_WORK) &&
										(logger->work_mode == MODE_SPECTRE) &&
										(logger->bdbg_spectrum_active != 0U) &&
										(logger->bdbg_spectrum_stopping == 0U))
								{
										/*
										 * Сначала проверяем ограничение
										 * длительности набора спектра.
										 *
										 * saveSpectre == 0 означает
										 * неограниченное время набора.
										 */
										if ((logger->flight_config.saveSpectre != 0U) &&
												((uint32_t)(
														rtc_seconds -
														logger->bdbg_spectrum_started_s
												) >= logger->flight_config.saveSpectre))
										{
												/*
												 * Время набора истекло.
												 *
												 * Запрещаем дальнейшие GET_SPECTRE_REQ
												 * и подготавливаем PAED_REQ для
												 * штатной остановки прибора.
												 */
												logger->bdbg_spectrum_stopping = 1U;
												logger->bdbg_request = PAED_REQ;
												logger->state = REQ_BDBG;

												break;
										}

										/*
										 * Если время набора ещё не истекло,
										 * проверяем пятитисекундный интервал
										 * получения следующей порции спектра.
										 */
										if ((uint32_t)(
												rtc_seconds -
												logger->bdbg_last_poll_s
										) >= 5U)
										{
												/*
												 * Запрашиваем следующую порцию спектра.
												 */
												logger->bdbg_request = GET_SPECTRE_REQ;
												logger->state = REQ_BDBG;

												break;
										}
								}

								/*
								 * Нет завершённой команды RadioLine
								 * и нет необходимости переходить
								 * в другое состояние.
								 */
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
														 */
														if ((new_config.mode <= 2U) && (new_config.saveTime > 0U) && (new_config.saveDistance > 0U) && (new_config.duplCoeff >= LOGGER_MIN_REPEAT_FACTOR) && (new_config.duplCoeff <= LOGGER_MAX_REPEAT_FACTOR))
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


                        
												/*
												 * ============================================================
												 * ID09 — изменение режима радиационной разведки.
												 *
												 * Payload:
												 * 0 — операторский режим;
												 * 1 — режим по времени;
												 * 2 — режим по расстоянию;
												 * 3 — запуск набора спектра.
												 * ============================================================
												 */
												if ((received_command == 0x09U) &&
														(received_payload_length == 1U) &&
														(logger->req_mode == MODE_WORK))
												{
														const uint8_t requested_mode =
																radioline_rx_buffer[received_payload_pos];

														/*
														 * Не прерываем активный или уже запускаемый спектр.
														 */
														if ((logger->bdbg_spectrum_active == 0U) &&
																(logger->bdbg_spectrum_stopping == 0U) &&
																(logger->work_mode != START_SPECTRE) &&
																(logger->work_mode != MODE_SPECTRE))
														{
																if (requested_mode <= 2U)
																{
																		/*
																		 * Обычный режим разведки.
																		 *
																		 * Устанавливаем новый режим и начинаем
																		 * новый интервал измерений.
																		 */
																		logger->work_mode =
																				(DroneWorkMode)requested_mode;

																		LoggerStateMachine_StopInterval(logger);
																}
																else if (requested_mode == 3U)
																{
																		/*
																		 * Запрошено начало набора спектра.
																		 *
																		 * Сохраняем текущий рабочий режим,
																		 * чтобы восстановить его после
																		 * завершения спектрального измерения.
																		 */
																		logger->old_work_mode = logger->work_mode;

																		/*
																		 * Останавливаем текущий интервал измерений.
																		 */
																		LoggerStateMachine_StopInterval(logger);

																		/*
																		 * Переходим в промежуточный режим
																		 * подготовки запуска спектра.
																		 */
																		logger->work_mode = START_SPECTRE;

																		/*
																		 * Новый набор ещё не запущен.
																		 */
																		logger->bdbg_spectrum_active = 0U;
																		logger->bdbg_spectrum_stopping = 0U;
																		logger->bdbg_spectrum_error_shutdown = 0U;

																		/*
																		 * Сбрасываем признак достоверности
																		 * координат и высоты нового спектра.
																		 *
																		 * Он будет установлен в LIDAR_PROC
																		 * после успешного получения данных.
																		 */
																		logger->spectrum_start_data_valid = 0U;

																		/*
																		 * Подготавливаем будущий запрос БДБГ.
																		 *
																		 * Сейчас его НЕ отправляем.
																		 */
																		logger->bdbg_request = START_SPECTRE_REQ;
																		
																		/*
																		 * Начинается новая подготовка спектра.
																		 *
																		 * Сбрасываем признак повторного опроса GPS/LIDAR,
																		 * чтобы не использовать состояние предыдущей попытки.
																		 */
																		logger->spectrum_prepare_retry = 0U;


																		/*
																		 * Сначала запрашиваем GPS.
																		 *
																		 * После успешного GPS:
																		 * REQ_LIDAR -> LIDAR_PROC.
																		 *
																		 * В LIDAR_PROC будут сохранены
																		 * координаты и высота начала спектра.
																		 *
																		 * Только затем будет выполнен
																		 * переход в REQ_BDBG.
																		 */
																		logger->state = REQ_GPS;
																}
														}
												}


                        /*
                         * ID12: досрочная остановка набора спектра.
                         * Payload должен содержать ровно один байт 0x01.
                         */
                        if ((received_command == 0x0CU) &&
                            (received_payload_length == 1U) &&
                            (radioline_rx_buffer[received_payload_pos] == 0x01U) &&
                            (logger->req_mode == MODE_WORK) &&
                            (logger->work_mode == MODE_SPECTRE) &&
                            (logger->bdbg_spectrum_active != 0U) &&
                            (logger->bdbg_spectrum_stopping == 0U))
                        {
                            /* Останавливаем прибор штатной командой PAED_REQ. */
                            logger->bdbg_spectrum_stopping = 1U;
                            logger->bdbg_request = PAED_REQ;
                            logger->state = REQ_BDBG;
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
						 * Настраиваем TIM3 на 5000 мс для GPS.
						 *
						 * TIM3 используется несколькими приборами,
						 * поэтому ARR необходимо устанавливать
						 * перед каждым новым ожиданием.
						 *
						 * Частота счёта TIM3 — 1000 Гц (1 мс).
						 * ARR = 4999 соответствует 5000 мс.
						 */
						(void)HAL_TIM_Base_Stop_IT(&htim3);

						__HAL_TIM_SET_AUTORELOAD(&htim3, 5000U - 1U);

						/*
						 * Начинаем отсчёт с нуля и удаляем
						 * возможный флаг предыдущего таймаута.
						 */
						__HAL_TIM_SET_COUNTER(&htim3, 0U);
						__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);


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
									 * Если ошибка RTC не привела к MODE_ERROR,
									 * повторяем GPS при подготовке спектра.
									 *
									 * Повтор GPS необходим, чтобы получить новый
									 * пакет координат и повторно проверить его
									 * относительно времени RTC.
									 *
									 * Счётчик ошибок RTC и ID26 не изменяем.
									 */
									if ((logger->req_mode == MODE_WORK) &&
											(logger->work_mode == START_SPECTRE))
									{
											logger->spectrum_prepare_retry = 1U;
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
												 * ID24 — зафиксирован GPS-спуфинг.
												 * Дальнейшая радиационная разведка невозможна.
												 * Стандартный Payload: 0x01.
												 */
												(void)RadioLine_SendPacket(24U, NULL, 0U);

												logger->req_mode = MODE_ERROR;
										}
										
	
										/*
										 * Подозрительную координату не используем.
										 *
										 * Если лимит спуфинга ещё не достигнут,
										 * повторяем GPS при подготовке спектра.
										 *
										 * Существующий счётчик спуфинга,
										 * отправку ID24 и MODE_ERROR не изменяем.
										 */
										if ((logger->req_mode == MODE_WORK) &&
												(logger->work_mode == START_SPECTRE))
										{
												logger->spectrum_prepare_retry = 1U;
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
										 * ========================================================
										 * Дрон вошёл в область радиационной разведки.
										 *
										 * ID20 — координаты GPS и время входа в область.
										 *
										 * Этот участок выполняется в MODE_EXPECT
										 * при первом обнаружении координаты внутри области.
										 * ========================================================
										 */

										/*
										 * Передаём пакет ID20 на землю по RadioLine.
										 *
										 * Формат полезной нагрузки:
										 *
										 * Байты 0–3  — latitude, float, Little Endian.
										 * Байты 4–7  — longitude, float, Little Endian.
										 * Байт  8    — часы.
										 * Байт  9    — минуты.
										 * Байт  10   — секунды.
										 *
										 * Всего 11 байт.
										 */
										(void)RadioLine_SendPacket(
												20U,
												gps_payload,
												sizeof(gps_payload)
										);

										/*
										 * ========================================================
										 * Сохранение ID20 на SD-карту.
										 * ========================================================
										 *
										 * Записываем тот же пакет, который отправили
										 * на землю по RadioLine.
										 *
										 * Формат строки CSV:
										 *
										 * 14,XX,XX,XX,...,XX
										 *
										 * 14 — идентификатор ID20 в HEX.
										 * Далее 11 байт полезной нагрузки.
										 *
										 * SD_Logger_WritePacket() самостоятельно:
										 * - проверяет доступность SD-карты;
										 * - проверяет наличие открытого CSV-файла;
										 * - преобразует байты в HEX;
										 * - разделяет байты запятыми;
										 * - записывает строку;
										 * - выполняет f_sync();
										 * - регистрирует ошибки FatFs.
										 *
										 * Ошибка записи не должна препятствовать
										 * переходу LOGGER в MODE_WORK.
										 */
										(void)SD_Logger_WritePacket(
												20U,
												gps_payload,
												(uint16_t)sizeof(gps_payload)
										);


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
										 * Дрон находится вне области разведки.
										 *
										 * ID19 — координаты GPS и время.
										 *
										 * Передаём пакет на землю независимо
										 * от состояния SD-карты.
										 */
										(void)RadioLine_SendPacket(
												19U,
												gps_payload,
												sizeof(gps_payload)
										);

										/*
										 * Сохраняем тот же пакет ID19 на SD-карту.
										 *
										 * Выполняется только в MODE_EXPECT,
										 * поскольку данный участок находится
										 * внутри соответствующей ветки.
										 *
										 * Формат CSV:
										 *
										 * 13,XX,XX,...,XX
										 *
										 * 13 — это ID19 в шестнадцатеричном виде.
										 * Далее 11 байт GPS и времени.
										 *
										 * SD_Logger_WritePacket():
										 * - проверяет разрешение записи;
										 * - проверяет наличие открытого файла;
										 * - преобразует байты в HEX;
										 * - записывает строку CSV;
										 * - выполняет f_sync();
										 * - регистрирует ошибки FatFs.
										 *
										 * Ошибка SD не должна прерывать
										 * работу LOGGER или передачу RadioLine.
										 */
										(void)SD_Logger_WritePacket(
												19U,
												gps_payload,
												(uint16_t)sizeof(gps_payload)
										);

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

								logger->dev_type = DEV_NONE;
								logger->state = COMMAND_INFO;

								break;
						}

						
						/*
						 * ================================================================
						 * MODE_WORK — радиационная разведка.
						 * ================================================================
						 *
						 * GPS успешно обработан.
						 * Определяем дальнейший переход по режиму разведки.
						 */
						if (logger->req_mode == MODE_WORK)
						{
								/*
								 * START_SPECTRE — подготовка набора спектра.
								 *
								 * После получения достоверных GPS-данных
								 * необходимо измерить высоту LIDAR.
								 *
								 * Наличие ранее сохранённых координат
								 * на этот переход не влияет.
								 */
								if (logger->work_mode == START_SPECTRE)
								{
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;
										break;
								}

								/*
								 * MODE_TIME — разведка по времени.
								 *
								 * Запрос GPS уже инициирован программным
								 * интервалом RTC. Продолжаем цикл измерений.
								 */
								if (logger->work_mode == MODE_TIME)
								{
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;
										break;
								}

								/*
								 * MODE_DISTANCE — разведка по расстоянию.
								 *
								 * После GPS получаем высоту LIDAR.
								 * Проверка расстояния и изменения высоты
								 * выполняется в LIDAR_PROC.
								 */
								if (logger->work_mode == MODE_DISTANCE)
								{
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;
										break;
								}

								/*
								 * MODE_OPERATOR — операторский режим.
								 *
								 * GPS запрошен в составе полного цикла
								 * измерений, инициированного оператором.
								 */
								if (logger->work_mode == MODE_OPERATOR)
								{
										logger->dev_type = DEV_NONE;
										logger->state = REQ_LIDAR;
										break;
								}

								/*
								 * ============================================================
								 * НЕОПРЕДЕЛЁННЫЙ РЕЖИМ РАЗВЕДКИ
								 * ============================================================
								 *
								 * Значение work_mode не соответствует
								 * ни одному режиму, для которого разрешено
								 * продолжение цикла после GPS.
								 *
								 * Не запускаем LIDAR и БДБГ.
								 */

								/*
								 * Освобождаем идентификатор прибора.
								 */
								logger->dev_type = DEV_NONE;

								/*
								 * Сбрасываем возможный признак повторного
								 * опроса GPS/LIDAR при подготовке спектра.
								 */
								logger->spectrum_prepare_retry = 0U;

								/*
								 * Останавливаем программный интервал
								 * запуска измерений.
								 *
								 * Глобальный счётчик RTC продолжает работать.
								 */
								LoggerStateMachine_StopInterval(logger);

								/*
								 * ID29 — «Прошу выбрать режим разведки».
								 *
								 * При NULL и длине 0 функция
								 * RadioLine_SendPacket() использует
								 * стандартный Payload 0x01.
								 */
								(void)RadioLine_SendPacket(29U, NULL, 0U);

								/*
								 * Переходим к обработке команд с земли.
								 *
								 * Не устанавливаем MODE_ERROR,
								 * поскольку отказ прибора не обнаружен.
								 *
								 * Повторная отправка ID29 здесь не произойдёт,
								 * пока не будет нового входа в GPS_PROC.
								 */
								logger->state = COMMAND_INFO;
								break;
						}
			

					  break;   
			  } 				
				
				
				case GPS_ERROR_PROC:
				{
						/*
						 * MODE_TEST — предполётное тестирование GPS.
						 *
						 * Ошибка GPS в этом режиме обрабатывается
						 * непосредственно обработчиком TIM3.
						 *
						 * Здесь только освобождаем прибор
						 * и возвращаемся к обработке команд.
						 */
						if (logger->req_mode == MODE_TEST)
						{
								logger->dev_type = DEV_NONE;
								logger->state = COMMAND_INFO;
								break;
						}

						/*
						 * Ошибки GPS учитываем в трёх режимах:
						 *
						 * MODE_EXPECT — ожидание начала разведки;
						 * MODE_WORK   — радиационная разведка;
						 * MODE_RETURN — возвращение.
						 */
						if ((logger->req_mode == MODE_EXPECT) ||
								(logger->req_mode == MODE_WORK)   ||
								(logger->req_mode == MODE_RETURN))
						{
								/*
								 * Увеличиваем счётчик последовательных
								 * ошибок GPS.
								 *
								 * Защита от переполнения uint8_t.
								 */
								if (logger->gps_error_counter < UINT8_MAX)
								{
										logger->gps_error_counter++;
								}

								/*
								 * Пока предел последовательных ошибок
								 * не достигнут, продолжаем работу.
								 */
								
								if (logger->gps_error_counter < LOGGER_DEVICE_ERROR_LIMIT)
								{
										/*
										 * Первая или вторая последовательная ошибка GPS.
										 *
										 * Если GPS опрашивается при подготовке спектра,
										 * необходимо повторить его запрос.
										 *
										 * Существующий переход через COMMAND_INFO сохраняем.
										 */
										if ((logger->req_mode == MODE_WORK) &&
												(logger->work_mode == START_SPECTRE))
										{
												logger->spectrum_prepare_retry = 1U;
										}

										logger->dev_type = DEV_NONE;
										logger->state = COMMAND_INFO;
										break;
								}


								/*
								 * Достигнут предел последовательных
								 * ошибок GPS.
								 *
								 * ID25 — дальнейшее выполнение задания
								 * невозможно из-за отказа прибора.
								 *
								 * Полезная нагрузка:
								 * 0x02 — GPS.
								 *
								 * Размер — 1 байт.
								 */
								const uint8_t failed_device = 2U;

								/*
								 * Передаём ID25 на наземную станцию.
								 *
								 * Прежний ID14 здесь больше
								 * не используется.
								 */
								(void)RadioLine_SendPacket(
										25U,
										&failed_device,
										1U
								);

								/*
								 * Записываем тот же ID25 на SD-карту.
								 *
								 * Если файл не открыт или SD недоступна,
								 * модуль регистрации обработает ситуацию
								 * согласно своей логике.
								 *
								 * Ошибка записи не препятствует
								 * переходу в MODE_ERROR.
								 */
								(void)SD_Logger_WritePacket(
										25U,
										&failed_device,
										1U
								);

								/*
								 * Переводим систему в аварийный режим.
								 */
								logger->req_mode = MODE_ERROR;

								/*
								 * Освобождаем прибор и возвращаемся
								 * к обработке команд.
								 */
								logger->dev_type = DEV_NONE;
								logger->state = COMMAND_INFO;

								break;
						}

						/*
						 * Для остальных режимов сохраняем
						 * существующее поведение.
						 */
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
					 * Настраиваем TIM3 на 5000 мс для LIDAR.
					 *
					 * ARR устанавливаем при каждом новом опросе,
					 * независимо от предыдущего использования TIM3
					 * другими приборами.
					 *
					 * Частота счёта TIM3 — 1000 Гц (1 мс).
					 * ARR = 4999 соответствует 5000 мс.
					 */
					(void)HAL_TIM_Base_Stop_IT(&htim3);

					__HAL_TIM_SET_AUTORELOAD(&htim3, 5000U - 1U);

					/*
					 * Сбрасываем счётчик и старый флаг обновления,
					 * чтобы новый таймаут отсчитывался с нуля.
					 */
					__HAL_TIM_SET_COUNTER(&htim3, 0U);
					__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);


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
						
						/*
						 * Сохраняем последнее корректное измерение LIDAR.
						 *
						 * Значения distance_cm и strength понадобятся
						 * при формировании пакета ID22 в состоянии WORK_SD.
						 *
						 * Данные сохраняются только после успешной
						 * проверки ответа функцией LIDAR_Process().
						 */
						logger->lidar_data = measurement;
						
						

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

             /*
             * START_SPECTRE:
             *
             * Фиксируем GPS-координаты и высоту LIDAR,
             * полученные непосредственно перед запуском спектра.
             *
             * Эти значения будут использоваться при последующем
             * сохранении накопленного спектра на SD-карту.
             *
             * Повторное измерение GPS и LIDAR после подтверждения
             * START_SPECTRE_REQ не требуется.
             */
            if (logger->work_mode == START_SPECTRE)
            {
                /*
                 * Сохраняем текущие GPS-координаты.
                 *
                 * Проверка достоверности GPS выполняется
                 * ранее в последовательности состояний
                 * утверждённой стейт-машины.
                 */
                logger->spectrum_start_coord = logger->gps_data.current_coord;

                /*
                 * Сохраняем высоту из только что обработанного
                 * корректного измерения LIDAR.
                 */
                logger->spectrum_start_height_cm = measurement.distance_cm;

                /*
                 * Обе величины зафиксированы.
                 */
                logger->spectrum_start_data_valid = 1U;

                /*
                 * Существующий переход не изменяем.
                 */
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
						 * Прекращаем ожидание ответа LIDAR
						 * и останавливаем приём данных.
						 *
						 * Это безопасно также при ошибке запуска DMA,
						 * когда таймер TIM3 ещё не был запущен.
						 */
						(void)HAL_TIM_Base_Stop_IT(&htim3);
						LIDAR_StopReceive();

						/*
						 * Очищаем признаки завершения приёма,
						 * таймаута и активного прибора.
						 */
						lidar_info = 0U;
						device_timeout = 0U;
						logger->dev_type = DEV_NONE;

						/*
						 * MODE_TEST — предполётная проверка LIDAR.
						 *
						 * ID12 сообщает об отсутствии передачи
						 * данных от LIDAR.
						 *
						 * Счётчик последовательных ошибок
						 * в режиме тестирования не увеличиваем.
						 */
						if (logger->req_mode == MODE_TEST)
						{
								(void)RadioLine_SendPacket(12U, NULL, 0U);

								logger->state = COMMAND_INFO;
								break;
						}

						/*
						 * MODE_WORK — радиационная разведка.
						 *
						 * Учитываем последовательные ошибки LIDAR.
						 */
						if (logger->req_mode == MODE_WORK)
						{
								/*
								 * Увеличиваем счётчик с защитой
								 * от переполнения uint8_t.
								 */
								if (logger->lidar_error_counter < UINT8_MAX)
								{
										logger->lidar_error_counter++;
								}

								/*
								 * Если предел последовательных ошибок
								 * ещё не достигнут, продолжаем работу.
								 */
								
								if (logger->lidar_error_counter < LOGGER_DEVICE_ERROR_LIMIT)
								{
										/*
										 * Первая или вторая последовательная ошибка LIDAR.
										 *
										 * GPS уже успешно обработан, поэтому повторяем
										 * только LIDAR.
										 *
										 * Существующий переход через COMMAND_INFO сохраняем.
										 */
										if ((logger->req_mode == MODE_WORK) &&
												(logger->work_mode == START_SPECTRE))
										{
												logger->spectrum_prepare_retry = 2U;
										}

										logger->state = COMMAND_INFO;
										break;
								}


								/*
								 * Достигнут предел последовательных ошибок.
								 *
								 * ID25 — дальнейшая радиационная
								 * разведка невозможна.
								 *
								 * Код неисправного прибора:
								 * 0x01 — LIDAR.
								 */
								const uint8_t failed_device = 1U;

								/*
								 * Отправляем ID25 на наземную станцию.
								 *
								 * Ранее здесь использовался ID13.
								 * Теперь ID13 в MODE_WORK не отправляем.
								 */
								(void)RadioLine_SendPacket(
										25U,
										&failed_device,
										1U
								);

								/*
								 * Сохраняем тот же пакет ID25
								 * в открытом CSV-файле SD-карты.
								 *
								 * При ошибке SD модуль регистрации
								 * самостоятельно отключает запись.
								 *
								 * Ошибка SD не должна препятствовать
								 * переходу в аварийный режим.
								 */
								(void)SD_Logger_WritePacket(
										25U,
										&failed_device,
										1U
								);

								/*
								 * Дальнейшая радиационная разведка
								 * невозможна из-за отказа LIDAR.
								 */
								logger->req_mode = MODE_ERROR;
								logger->state = COMMAND_INFO;

								break;
						}

						/*
						 * Защита от неожиданного режима работы.
						 */
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
            /* TIM3: 64 MHz / (63999+1) = 1000 Гц; 3000 отсчётов = 3 с. */
            (void)HAL_TIM_Base_Stop_IT(&htim3);
            __HAL_TIM_SET_AUTORELOAD(&htim3, BDBG_RESPONSE_TIMEOUT_MS - 1U);
            __HAL_TIM_SET_COUNTER(&htim3, 0U);
            /* Сбрасываем старый флаг обновления, чтобы таймаут отсчитывался с нуля. */
            __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
            if (BDBG_StartRequest(&huart4, logger->bdbg_request) == 0U)
            {
                logger->dev_type = DEV_NONE;
                logger->state = (logger->req_mode == MODE_TEST) ? COMMAND_INFO : BDBG_ERROR_PROC;
                if (logger->req_mode == MODE_TEST) (void)RadioLine_SendPacket(8U, NULL, 0U);
                break;
            }
            /*
             * Запускаем TIM3. При ошибке запуска контроль
             * времени ответа невозможен — обрабатываем так же,
             * как ошибку отправки запроса БДБГ.
             */
            if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
            {
                BDBG_StopReceive();
                logger->dev_type = DEV_NONE;
                logger->state = (logger->req_mode == MODE_TEST) ? COMMAND_INFO : BDBG_ERROR_PROC;
                if (logger->req_mode == MODE_TEST) (void)RadioLine_SendPacket(8U, NULL, 0U);
                break;
            }
            logger->state = BDBG_PROC;
            break;
        }
							

        case BDBG_PROC:
				{
						/*
						 * Получаем текущее состояние DMA-приёма БДБГ.
						 *
						 * Пока полный ответ не получен и ошибка UART
						 * не зарегистрирована, остаёмся в BDBG_PROC.
						 */
						BDBG_RxStatus rx = BDBG_GetRxStatus();

						if (rx == BDBG_RX_WAIT)
						{
								break;
						}

						/*
						 * Ответ получен либо зарегистрирована ошибка.
						 * Останавливаем контроль времени и приём UART4.
						 */
						(void)HAL_TIM_Base_Stop_IT(&htim3);
						device_timeout = 0U;
						logger->dev_type = DEV_NONE;

						/*
						 * Проверяем полноту ответа, формат и CRC.
						 */
						uint8_t ok =
								(rx == BDBG_RX_READY) &&
								BDBG_ParseResponse(
										logger->bdbg_request,
										&logger->bdbg_data
								);

						BDBG_StopReceive();

						/*
						 * Некорректный ответ или ошибка UART.
						 */
						if (ok == 0U)
						{
								if (logger->req_mode == MODE_TEST)
								{
										/*
										 * ID07 — некорректный ответ БДБГ
										 * во время предполётной проверки.
										 */
										(void)RadioLine_SendPacket(7U, NULL, 0U);
										logger->state = COMMAND_INFO;
								}
								else
								{
										/*
										 * В рабочем режиме ошибка учитывается
										 * общим механизмом BDBG_ERROR_PROC.
										 */
										logger->state = BDBG_ERROR_PROC;
								}

								break;
						}

						/*
						 * Успешный ответ прерывает последовательность ошибок.
						 */
						logger->bdbg_error_counter = 0U;

						/*
						 * Предполётная проверка БДБГ.
						 */
						if (logger->req_mode == MODE_TEST)
						{
								(void)RadioLine_SendPacket(9U, NULL, 0U);
								logger->state = REQ_BATTERY;
								break;
						}

						/*
						 * В остальных режимах, кроме MODE_WORK,
						 * обычная обработка БДБГ не выполняется.
						 */
						if (logger->req_mode != MODE_WORK)
						{
								logger->state = COMMAND_INFO;
								break;
						}

						switch (logger->bdbg_request)
						{	
		
								
								case PAED_REQ:
								{
										/*
										 * Успешный PAED_REQ во время завершения
										 * спектрального измерения.
										 *
										 * Независимо от причины остановки
										 * переходим к сохранению спектра.
										 */
										if (logger->bdbg_spectrum_stopping != 0U)
										{
												/*
												 * MODE_ERROR здесь НЕ устанавливаем.
												 *
												 * При аварийной остановке это сделает
												 * WORK_SD после попытки записи спектра
												 * и отправки сообщения ID25.
												 */
												logger->state = WORK_SD;
												break;
										}

										/*
										 * Обычный цикл измерений БДБГ:
										 * после PAED_REQ запрашиваем температуру.
										 */
										HAL_Delay(50U);

										logger->bdbg_request = TEMP_REQ;
										logger->state = REQ_BDBG;
										break;
								}


								case TEMP_REQ:
								{
										/*
										 * Температура уже сохранена
										 * функцией BDBG_ParseResponse().
										 */
										logger->state = REQ_BATTERY;
										break;
								}
								
								
								case START_SPECTRE_REQ:
								{
										/*
										 * БДБГ-15 успешно подтвердил команду
										 * начала набора спектра.
										 *
										 * С этого момента набор считается начатым.
										 */

										/*
										 * ========================================================
										 * ЗАВЕРШЕНИЕ МЕХАНИЗМА ПОВТОРНОГО ЗАПУСКА
										 * ========================================================
										 *
										 * Корректный ответ START_SPECTRE_REQ получен.
										 *
										 * Отменяем возможное ожидание повторного запроса
										 * и очищаем сохранённое время последней ошибки.
										 *
										 * Счётчик bdbg_error_counter уже обнулён
										 * общей обработкой успешного ответа БДБГ.
										 */
										logger->bdbg_start_retry_pending = 0U;
										logger->bdbg_start_retry_tick = 0U;

										/*
										 * Обнуляем накопленный спектр LOGGER.
										 *
										 * В массиве spectre_arr[] будет накапливаться
										 * сумма последующих порций спектра БДБГ-15.
										 */
										memset(
												logger->spectre_arr,
												0,
												sizeof(logger->spectre_arr)
										);

										/*
										 * Устанавливаем признаки активного набора.
										 *
										 * Процедура остановки набора сейчас
										 * не выполняется.
										 */
										logger->bdbg_spectrum_active = 1U;
										logger->bdbg_spectrum_stopping = 0U;

										/*
										 * Фиксируем время начала набора
										 * по секундному счётчику LOGGER.
										 *
										 * Этот счётчик используется для контроля
										 * длительности набора и интервалов опроса.
										 */
										logger->bdbg_spectrum_started_s = rtc_seconds;
										logger->bdbg_last_poll_s = rtc_seconds;

										/*
										 * Получаем дату и время начала набора.
										 *
										 * Сначала копируем последнее сохранённое
										 * время RTC в резервную переменную.
										 *
										 * Если новое чтение RTC завершится ошибкой,
										 * сохранённое время будет использовано
										 * для идентификатора спектра.
										 */
										GPS_TimeStamp spectrum_time =
												logger->gps_data.current_rtc_time;

										/*
										 * Пытаемся получить актуальное время RTC.
										 *
										 * При успешном чтении обновляем также
										 * последнее сохранённое время.
										 *
										 * При ошибке резервное значение spectrum_time
										 * остаётся без изменений.
										 *
										 * Ошибка RTC не отменяет набор спектра.
										 */
										GPS_TimeStamp current_time = {0};

										if (RTC_Service_GetTime(&current_time) == HAL_OK)
										{
												spectrum_time = current_time;

												logger->gps_data.current_rtc_time =
														current_time;
										}

										/*
										 * Сохраняем дату и время начала спектра.
										 *
										 * Эти данные будут использованы
										 * при формировании радиопакета ID27
										 * и последующей записи спектра на SD-карту.
										 */
										logger->spectrum_start_time = spectrum_time;

										/*
										 * Увеличиваем порядковый номер спектра.
										 *
										 * После включения LOGGER счётчик равен 0.
										 * Первый подтверждённый набор получает номер 1.
										 *
										 * При достижении UINT16_MAX счётчик
										 * возвращается к 1, не используя номер 0.
										 */
										if (logger->spectrum_sequence_number == UINT16_MAX)
										{
												logger->spectrum_sequence_number = 1U;
										}
										else
										{
												logger->spectrum_sequence_number++;
										}

										/*
										 * Передаём радиопакет ID27:
										 *
										 * - дата и время начала набора;
										 * - порядковый номер спектра;
										 * - GPS-координаты начала набора;
										 * - высота LIDAR начала набора.
										 *
										 * Координаты и высота были зафиксированы
										 * ранее, перед отправкой START_SPECTRE_REQ.
										 *
										 * Ошибка радиопередачи не отменяет
										 * уже запущенный набор спектра.
										 *
										 * Пакет ID27 на SD-карту не записывается.
										 */
										(void)RadioLine_SendSpectrumStart(logger);

										/*
										 * Переходим в режим набора спектра.
										 *
										 * Следующий запрос к БДБГ-15:
										 * GET_SPECTRE_REQ — получение порции спектра.
										 */
										logger->work_mode = MODE_SPECTRE;
										logger->bdbg_request = GET_SPECTRE_REQ;
										logger->state = COMMAND_INFO;

										break;
								}					
															
																
								case GET_SPECTRE_REQ:
								{
										/*
										 * Максимальное накопленное значение
										 * среди всех 1024 каналов спектра.
										 */
										uint16_t max_channel_value = 0U;

										/*
										 * Накапливаем очередную порцию спектра.
										 *
										 * Каждое значение из ответа БДБГ
										 * прибавляется к соответствующему каналу
										 * накопленного спектра LOGGER.
										 */
										for (uint16_t i = 0U;
												 i < BDBG_SPECTRUM_CHANNELS;
												 ++i)
										{
												/*
												 * Промежуточную сумму вычисляем в uint32_t,
												 * чтобы исключить переполнение при сложении.
												 */
												uint32_t sum =
														(uint32_t)logger->spectre_arr[i] +
														(uint32_t)logger->bdbg_data.spectrum[i];

												/*
												 * Ограничиваем накопленное значение
												 * максимальным значением uint16_t.
												 */
												if (sum > UINT16_MAX)
												{
														sum = UINT16_MAX;
												}

												logger->spectre_arr[i] = (uint16_t)sum;

												/*
												 * Определяем максимальное значение
												 * среди накопленных каналов.
												 */
												if (logger->spectre_arr[i] > max_channel_value)
												{
														max_channel_value = logger->spectre_arr[i];
												}
										}

										/*
										 * Фиксируем время обработки очередной порции
										 * для отсчёта интервала до следующего запроса.
										 */
										logger->bdbg_last_poll_s = rtc_seconds;

										/*
										 * Передаём очередную НОВУЮ порцию спектра
										 * по RadioLine сообщением ID23.
										 *
										 * RadioLine_SendSpectrum() самостоятельно:
										 * - формирует Payload из 1024 каналов;
										 * - добавляет время набора (2 байта);
										 * - добавляет ПАЕД (4 байта);
										 * - добавляет статус БДБГ (1 байт);
										 * - отправляет пакет длиной 2055 байт.
										 *
										 * Накопленный spectre_arr[] не передаётся.
										 *
										 * Ошибка радиопередачи не отменяет
										 * уже принятую и накопленную порцию спектра.
										 */
										(void)RadioLine_SendSpectrum(&logger->bdbg_data);

										/*
										 * Проверяем достижение 85% от UINT16_MAX.
										 *
										 * 65535 * 0.85 = 55704.75.
										 *
										 * Порог с округлением вверх: 55705.
										 */
										if (max_channel_value >= 55705U)
										{
												/*
												 * Запрещаем дальнейшее получение порций
												 * спектра и инициируем штатную остановку БДБГ.
												 */
												logger->bdbg_spectrum_stopping = 1U;
												logger->bdbg_request = PAED_REQ;
												logger->state = REQ_BDBG;
												break;
										}

										/*
										 * Порог накопления не достигнут.
										 * Возвращаемся в COMMAND_INFO.
										 */
										logger->state = COMMAND_INFO;
										break;
								}


								default:
								{
										logger->state = COMMAND_INFO;
										break;
								}
						}

						break;
				}		

								
				case BDBG_ERROR_PROC:
				{
						/*
						 * Завершаем незаконченный обмен с БДБГ.
						 */
						(void)HAL_TIM_Base_Stop_IT(&htim3);
						BDBG_StopReceive();

						device_timeout = 0U;
						logger->dev_type = DEV_NONE;

						/*
						 * Ошибки БДБГ учитываем только
						 * в рабочем режиме радиационной разведки.
						 */
						if (logger->req_mode == MODE_WORK)
						{
								/*
								 * Увеличиваем счётчик последовательных
								 * ошибок без переполнения uint8_t.
								 */
								if (logger->bdbg_error_counter < UINT8_MAX)
								{
										logger->bdbg_error_counter++;
								}

								/*
								 * ==========================================
								 * 1. ОШИБКА ЗАПУСКА СПЕКТРА
								 * ==========================================
								 *
								 * Существующий алгоритм:
								 * повтор START_SPECTRE_REQ через 1000 мс;
								 * после исчерпания попыток — ID28.
								 */
								if ((logger->work_mode == START_SPECTRE) &&
										(logger->bdbg_request == START_SPECTRE_REQ))
								{
										if (logger->bdbg_error_counter >=
												LOGGER_DEVICE_ERROR_LIMIT)
										{
												/*
												 * Отменяем запланированные повторы.
												 */
												logger->bdbg_start_retry_pending = 0U;
												logger->bdbg_start_retry_tick = 0U;

												/*
												 * ID28 — запуск спектра не удался.
												 */
												(void)RadioLine_SendPacket(
														28U,
														NULL,
														0U
												);

												/*
												 * Сохраняем ID28 на SD.
												 */
												{
														const uint8_t default_payload = 0x01U;

														(void)SD_Logger_WritePacket(
																28U,
																&default_payload,
																1U
														);
												}

												logger->req_mode = MODE_ERROR;
												logger->state = COMMAND_INFO;
												break;
										}

										/*
										 * Лимит ошибок не достигнут.
										 * Планируем неблокирующий повтор.
										 */
										logger->bdbg_start_retry_tick = HAL_GetTick();
										logger->bdbg_start_retry_pending = 1U;

										logger->state = COMMAND_INFO;
										break;
								}

								/*
								 * ==========================================
								 * 2. ТРИ ОШИБКИ GET_SPECTRE_REQ
								 * ==========================================
								 *
								 * Прекращаем получение порций спектра.
								 * Начинаем попытки остановки БДБГ.
								 */
								if ((logger->work_mode == MODE_SPECTRE) &&
										(logger->bdbg_spectrum_active != 0U) &&
										(logger->bdbg_spectrum_stopping == 0U) &&
										(logger->bdbg_request == GET_SPECTRE_REQ) &&
										(logger->bdbg_error_counter >=
										 LOGGER_DEVICE_ERROR_LIMIT))
								{
										/*
										 * Запоминаем аварийную причину
										 * завершения спектрального режима.
										 */
										logger->bdbg_spectrum_error_shutdown = 1U;

										/*
										 * Запрещаем дальнейшие GET_SPECTRE_REQ.
										 */
										logger->bdbg_spectrum_stopping = 1U;

										/*
										 * Для PAED_REQ начинаем новый
										 * подсчёт последовательных ошибок.
										 */
										logger->bdbg_error_counter = 0U;

										logger->bdbg_request = PAED_REQ;
										logger->state = REQ_BDBG;
										break;
								}

								/*
								 * ==========================================
								 * 3. ОШИБКА PAED_REQ ПРИ ОСТАНОВКЕ
								 * ==========================================
								 */
								if (logger->bdbg_spectrum_stopping != 0U)
								{
										if (logger->bdbg_error_counter >=
												LOGGER_DEVICE_ERROR_LIMIT)
										{
												/*
												 * Попытки остановки БДБГ исчерпаны.
												 *
												 * При аварии GET_SPECTRE_REQ пакет
												 * ID25 отправит WORK_SD после
												 * попытки сохранения спектра.
												 *
												 * Поэтому здесь ID25 не отправляем
												 * для аварийного завершения.
												 */
												if (logger->bdbg_spectrum_error_shutdown == 0U)
												{
														/*
														 * Штатная остановка спектра
														 * тоже завершилась отказом БДБГ.
														 *
														 * Сохраняем прежнее поведение.
														 */
														const uint8_t failed_device = 0U;

														(void)RadioLine_SendPacket(
																25U,
																&failed_device,
																1U
														);

														(void)SD_Logger_WritePacket(
																25U,
																&failed_device,
																1U
														);

														logger->req_mode = MODE_ERROR;
												}

												/*
												 * Сохраняем накопленный спектр
												 * даже без подтверждения PAED_REQ.
												 */
												logger->state = WORK_SD;
												break;
										}

										/*
										 * Лимит попыток остановки не достигнут.
										 * Повторяем PAED_REQ.
										 */
										logger->bdbg_request = PAED_REQ;
										logger->state = REQ_BDBG;
										break;
								}

								/*
								 * ==========================================
								 * 4. ОБЩАЯ ОБРАБОТКА ОШИБОК БДБГ
								 * ==========================================
								 */
								if (logger->bdbg_error_counter >=
										LOGGER_DEVICE_ERROR_LIMIT)
								{
										const uint8_t failed_device = 0U;

										/*
										 * ID25 — дальнейшая разведка
										 * невозможна из-за отказа БДБГ.
										 */
										(void)RadioLine_SendPacket(
												25U,
												&failed_device,
												1U
										);

										(void)SD_Logger_WritePacket(
												25U,
												&failed_device,
												1U
										);

										logger->req_mode = MODE_ERROR;
										logger->state = COMMAND_INFO;
										break;
								}
						}

						/*
						 * Лимит ошибок не достигнут
						 * либо режим отличается от MODE_WORK.
						 */
						logger->state = COMMAND_INFO;
						break;
				}

				
				case REQ_BATTERY:
				{
						/*
						 * ========================================================
						 * REQ_BATTERY — НАЧАЛО ИЗМЕРЕНИЯ АККУМУЛЯТОРА
						 * ========================================================
						 *
						 * В этом состоянии:
						 *
						 * 1. Отключаем приём других приборов.
						 * 2. Подготавливаем ADC1.
						 * 3. Калибруем ADC1.
						 * 4. Запускаем TIM3 на 3 секунды.
						 * 5. Запускаем первое преобразование ADC1.
						 * 6. Переходим в BATTERY_PROC.
						 *
						 * Ожидание результата ADC здесь не выполняется.
						 */

						/*
						 * Останавливаем приём данных GPS, LIDAR и BDBG.
						 *
						 * Во время измерения аккумулятора обмен
						 * с другими приборами не требуется.
						 *
						 * BDBG_StopReceive() безопасно вызывать,
						 * даже если приём BDBG уже остановлен.
						 *
						 * UART при этом не деинициализируется.
						 */
						GPS_StopReceive();
						LIDAR_StopReceive();
						BDBG_StopReceive();

						/*
						 * Отключаем обработку возможных запоздавших
						 * результатов предыдущего измерения ADC.
						 */
						battery_adc_active = 0U;

						/*
						 * Останавливаем TIM3, если он ранее использовался
						 * для контроля другого прибора.
						 */
						(void)HAL_TIM_Base_Stop_IT(&htim3);

						/*
						 * Сбрасываем флаг предыдущего таймаута.
						 */
						device_timeout = 0U;

						/*
						 * Указываем, что следующий таймаут TIM3
						 * относится к контролю аккумулятора.
						 */
						logger->dev_type = DEV_BATTERY;

						/*
						 * ========================================================
						 * ПОДГОТОВКА НОВОГО ЦИКЛА ИЗМЕРЕНИЙ ADC
						 * ========================================================
						 *
						 * Очищаем результаты предыдущего цикла.
						 *
						 * Эти переменные относятся только к текущим
						 * десяти преобразованиям ADC.
						 *
						 * Счётчики последовательных ошибок аккумулятора
						 * здесь не изменяем.
						 */

						/*
						 * Очищаем последний цифровой результат ADC.
						 */
						battery_adc_raw = 0U;

						/*
						 * Сбрасываем признак готовности результата.
						 */
						battery_adc_ready = 0U;

						/*
						 * Сбрасываем признак ошибки преобразования.
						 */
						battery_adc_error = 0U;

						/*
						 * Обнуляем накопленную сумму результатов ADC.
						 */
						logger->battery_adc_sum = 0U;

						/*
						 * Обнуляем количество успешно обработанных
						 * преобразований в текущем цикле.
						 */
						logger->battery_adc_count = 0U;

						/*
						 * Перед калибровкой останавливаем ADC.
						 *
						 * Если остановка не удалась, измерение
						 * считаем неуспешным.
						 */
						if (HAL_ADC_Stop(&hadc1) != HAL_OK)
						{
								logger->dev_type = DEV_NONE;
								logger->state = BATTERY_ERROR_PROC;
								break;
						}

						/*
						 * Калибруем ADC1 в режиме single-ended.
						 *
						 * Калибровка выполняется до запуска
						 * первого преобразования.
						 */
						if (HAL_ADCEx_Calibration_Start(
														&hadc1,
														ADC_SINGLE_ENDED) != HAL_OK)
						{
								logger->dev_type = DEV_NONE;
								logger->state = BATTERY_ERROR_PROC;
								break;
						}

						/*
						 * Настраиваем TIM3 на 3000 мс.
						 *
						 * TIM3 настроен на частоту счёта 1000 Гц:
						 * один отсчёт соответствует 1 мс.
						 *
						 * Значение периода берём из общей
						 * константы LOGGER_BATTERY_TIMEOUT_MS.
						 */
						__HAL_TIM_SET_AUTORELOAD(
										&htim3,
										LOGGER_BATTERY_TIMEOUT_MS - 1U
						);

						/*
						 * Начинаем отсчёт времени с нуля.
						 */
						__HAL_TIM_SET_COUNTER(&htim3, 0U);

						/*
						 * Сбрасываем возможный старый флаг обновления
						 * таймера перед новым запуском.
						 */
						__HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);

						/*
						 * Запускаем TIM3 с разрешением прерывания.
						 *
						 * Если запуск не удался, переходим
						 * в обработку ошибки аккумулятора.
						 */
						if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
						{
								logger->dev_type = DEV_NONE;
								logger->state = BATTERY_ERROR_PROC;
								break;
						}

						/*
						 * Разрешаем обработчику прерывания ADC1
						 * сохранять результаты рабочих измерений.
						 */
						battery_adc_active = 1U;

						/*
						 * Запускаем первое преобразование ADC1.
						 *
						 * HAL_ADC_Start_IT() не ожидает результата.
						 * После запуска управление возвращается
						 * основной программе.
						 *
						 * Результат будет получен через
						 * HAL_ADC_ConvCpltCallback().
						 */
						if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
						{
								/*
								 * Запуск ADC не удался.
								 *
								 * Запрещаем обработку результатов,
								 * останавливаем таймер и переводим
								 * стейт-машину в обработку ошибки.
								 */
								battery_adc_active = 0U;

								(void)HAL_TIM_Base_Stop_IT(&htim3);

								device_timeout = 0U;

								logger->dev_type = DEV_NONE;
								logger->state = BATTERY_ERROR_PROC;

								break;
						}

						/*
						 * Первое преобразование успешно запущено.
						 *
						 * Переходим к ожиданию и накоплению
						 * результатов в BATTERY_PROC.
						 */
						logger->state = BATTERY_PROC;

						break;
				}
				
				

				case BATTERY_PROC:
				{
						/*
						 * ========================================================
						 * BATTERY_PROC — ОБРАБОТКА ИЗМЕРЕНИЙ АККУМУЛЯТОРА
						 * ========================================================
						 *
						 * Выполняем 10 последовательных преобразований ADC1.
						 *
						 * Каждое преобразование запускается через прерывание.
						 * Блокирующее ожидание результата не используется.
						 *
						 * TIM3 контролирует время полного цикла измерений.
						 */

						/*
						 * ========================================================
						 * 1. ПРОВЕРКА ОШИБКИ ADC
						 * ========================================================
						 */
						if (battery_adc_error != 0U)
						{
								/*
								 * Запрещаем обработку новых результатов.
								 */
								battery_adc_active = 0U;

								/*
								 * Останавливаем ADC1 и TIM3.
								 */
								(void)HAL_ADC_Stop_IT(&hadc1);
								(void)HAL_TIM_Base_Stop_IT(&htim3);

								/*
								 * Очищаем флаги текущего измерения.
								 */
								battery_adc_ready = 0U;
								battery_adc_error = 0U;
								device_timeout = 0U;

								/*
								 * Передаём управление обработчику ошибок.
								 *
								 * Здесь счётчики ошибок не изменяем.
								 */
								logger->dev_type = DEV_NONE;
								logger->state = BATTERY_ERROR_PROC;

								break;
						}

						/*
						 * ========================================================
						 * 2. ОЖИДАНИЕ РЕЗУЛЬТАТА ADC
						 * ========================================================
						 *
						 * Если результат ещё не получен,
						 * возвращаем управление основному циклу.
						 */
						if (battery_adc_ready == 0U)
						{
								break;
						}

						/*
						 * Запрещаем обработку нового результата
						 * до запуска следующего преобразования.
						 */
						battery_adc_active = 0U;

						/*
						 * Завершаем текущее преобразование.
						 */
						if (HAL_ADC_Stop_IT(&hadc1) != HAL_OK)
						{
								(void)HAL_TIM_Base_Stop_IT(&htim3);

								battery_adc_ready = 0U;
								battery_adc_error = 0U;
								device_timeout = 0U;

								logger->dev_type = DEV_NONE;
								logger->state = BATTERY_ERROR_PROC;

								break;
						}

						/*
						 * ========================================================
						 * 3. НАКОПЛЕНИЕ РЕЗУЛЬТАТОВ
						 * ========================================================
						 */

						/*
						 * Добавляем результат ADC к накопленной сумме.
						 */
						logger->battery_adc_sum +=
								(uint32_t)battery_adc_raw;

						/*
						 * Увеличиваем количество успешных преобразований.
						 */
						logger->battery_adc_count++;

						/*
						 * Результат обработан.
						 */
						battery_adc_ready = 0U;

						/*
						 * ========================================================
						 * 4. ЗАВЕРШЕНИЕ ДЕСЯТИ ПРЕОБРАЗОВАНИЙ
						 * ========================================================
						 */
						if (logger->battery_adc_count >=
								LOGGER_BATTERY_ADC_SAMPLES)
						{
								float adc_average;
								float adc_voltage;

								/*
								 * Все результаты получены.
								 * Останавливаем контроль времени.
								 */
								(void)HAL_TIM_Base_Stop_IT(&htim3);

								/*
								 * Сбрасываем флаг таймаута.
								 */
								device_timeout = 0U;

								/*
								 * Активного измерения больше нет.
								 */
								logger->dev_type = DEV_NONE;

								/*
								 * Вычисляем среднее цифровое значение ADC.
								 */
								adc_average =
										(float)logger->battery_adc_sum /
										(float)LOGGER_BATTERY_ADC_SAMPLES;

								/*
								 * Пересчитываем результат в напряжение
								 * непосредственно на входе ADC1 (PC1).
								 *
								 * Uadc = ADCaverage * Vref / 4095
								 */
								adc_voltage =
										(adc_average *
										 LOGGER_BATTERY_ADC_REFERENCE_V) /
										LOGGER_BATTERY_ADC_MAX_VALUE;

								/*
								 * Вычисляем фактическое напряжение аккумулятора
								 * с учётом коэффициента делителя 18.182.
								 */
								logger->battery_voltage =
										adc_voltage *
										LOGGER_BATTERY_DIVIDER_RATIO;

								/*
								 * Все десять преобразований завершились успешно.
								 *
								 * Поэтому сбрасываем счётчик последовательных
								 * ошибок ADC независимо от уровня напряжения.
								 */
								logger->batt_error_counter = 0U;

								/*
								 * ====================================================
								 * 5. КОНТРОЛЬ КРИТИЧЕСКОГО НАПРЯЖЕНИЯ
								 * ====================================================
								 *
								 * Сравниваем напряжение именно на входе ADC1,
								 * а не напряжение аккумулятора после делителя.
								 */
								if (adc_voltage < LOGGER_BATTERY_CRITICAL_V)
								{
										/*
										 * Напряжение ниже критического.
										 *
										 * Увеличиваем счётчик последовательных
										 * измерений низкого напряжения.
										 *
										 * Защищаем счётчик от переполнения.
										 */
										if (logger->batt_low_voltage_counter < UINT8_MAX)
										{
												logger->batt_low_voltage_counter++;
										}
								}
								else
								{
										/*
										 * Напряжение нормальное.
										 *
										 * Последовательность измерений
										 * низкого напряжения прервана.
										 */
										logger->batt_low_voltage_counter = 0U;
								}

								/*
								 * ====================================================
								 * 6. ПРЕДПОЛЁТНАЯ ПРОВЕРКА — MODE_TEST
								 * ====================================================
								 *
								 * В MODE_TEST переход в MODE_ERROR запрещён.
								 *
								 * Даже при трёх и более измерениях
								 * низкого напряжения остаёмся в MODE_TEST.
								 */
								if (logger->req_mode == MODE_TEST)
								{
										if (adc_voltage < LOGGER_BATTERY_CRITICAL_V)
										{
												/*
												 * ID04 — аккумулятор разряжен.
												 *
												 * Передаём фактическое напряжение
												 * аккумулятора в формате float.
												 */
												(void)RadioLine_SendPacket(
														4U,
														(const uint8_t *)&logger->battery_voltage,
														sizeof(logger->battery_voltage)
												);

												/*
												 * Предполётная проверка прервана.
												 *
												 * Ожидаем дальнейших команд оператора.
												 * Режим MODE_TEST сохраняется.
												 */
												logger->state = COMMAND_INFO;
										}
										else
										{
												/*
												 * ID05 — аккумулятор заряжен.
												 *
												 * Передаём измеренное напряжение.
												 */
												(void)RadioLine_SendPacket(
														5U,
														(const uint8_t *)&logger->battery_voltage,
														sizeof(logger->battery_voltage)
												);

												/*
												 * Продолжаем предполётную проверку.
												 */
												logger->state = WORK_SD;
										}

										break;
								}

								/*
								 * ====================================================
								 * 7. РАБОЧИЙ РЕЖИМ — MODE_WORK
								 * ====================================================
								 */
								if (logger->req_mode == MODE_WORK)
								{
										/*
										 * Проверяем количество последовательных
										 * измерений критически низкого напряжения.
										 */
										if (logger->batt_low_voltage_counter >=
												LOGGER_DEVICE_ERROR_LIMIT)
										{
												/*
												 * ID25 — дальнейшая разведка невозможна.
												 *
												 * Тип неисправного прибора:
												 * 3 — аккумулятор.
												 */
												const uint8_t failed_device = 3U;

												(void)RadioLine_SendPacket(
														25U,
														&failed_device,
														1U
												);

												/*
												 * Записываем тот же ID25 на SD-карту.
												 * Ошибка записи не препятствует
												 * переходу в MODE_ERROR.
												 */
												(void)SD_Logger_WritePacket(
														25U,
														&failed_device,
														1U
												);

												/*
												 * Критическое напряжение подтверждено
												 * несколькими последовательными измерениями.
												 *
												 * Прекращаем рабочий режим.
												 */
												logger->req_mode = MODE_ERROR;
												logger->state = COMMAND_INFO;

												break;
										}

										/*
										 * Напряжение нормальное либо количество
										 * последовательных низких измерений
										 * ещё не достигло установленного предела.
										 *
										 * Продолжаем штатный рабочий цикл.
										 */
										logger->state = WORK_SD;

										break;
								}

								/*
								 * ====================================================
								 * 8. ЗАЩИТА ОТ НЕОЖИДАННОГО РЕЖИМА
								 * ====================================================
								 *
								 * Если аккумулятор был опрошен в режиме,
								 * отличном от MODE_TEST и MODE_WORK,
								 * возвращаемся к обработке команд.
								 */
								logger->state = COMMAND_INFO;

								break;
						}

						/*
						 * ========================================================
						 * 9. ЗАПУСК СЛЕДУЮЩЕГО ПРЕОБРАЗОВАНИЯ
						 * ========================================================
						 *
						 * Сюда попадаем, если успешно получено
						 * менее десяти результатов ADC.
						 */

						/*
						 * Разрешаем обработчику прерывания
						 * сохранить очередной результат.
						 */
						battery_adc_active = 1U;

						/*
						 * Запускаем следующее преобразование.
						 */
						if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
						{
								/*
								 * Ошибка запуска очередного преобразования.
								 *
								 * Завершаем текущий цикл и передаём
								 * управление BATTERY_ERROR_PROC.
								 */
								battery_adc_active = 0U;

								(void)HAL_ADC_Stop_IT(&hadc1);
								(void)HAL_TIM_Base_Stop_IT(&htim3);

								battery_adc_ready = 0U;
								battery_adc_error = 0U;
								device_timeout = 0U;

								logger->dev_type = DEV_NONE;
								logger->state = BATTERY_ERROR_PROC;

								break;
						}

						/*
						 * Следующее преобразование запущено.
						 *
						 * Ожидаем результат на следующем проходе
						 * основной стейт-машины.
						 */
						break;
				}
				
								
				case BATTERY_ERROR_PROC:
				{
						/*
						 * ========================================================
						 * BATTERY_ERROR_PROC — ОБРАБОТКА ОШИБКИ АККУМУЛЯТОРА
						 * ========================================================
						 *
						 * В это состояние переходим при:
						 *
						 * 1. Ошибке подготовки или калибровки ADC1.
						 * 2. Ошибке запуска преобразования ADC1.
						 * 3. Ошибке выполнения преобразования ADC1.
						 * 4. Ошибке завершения преобразования ADC1.
						 * 5. Срабатывании таймера TIM3 (3 секунды).
						 *
						 * Любое из этих событий считается одной
						 * неудачной попыткой измерения аккумулятора.
						 *
						 * Счётчик batt_error_counter увеличивается
						 * исключительно в этом состоянии.
						 */

						/*
						 * ========================================================
						 * 1. ЗАВЕРШЕНИЕ ТЕКУЩЕГО ЦИКЛА ИЗМЕРЕНИЙ
						 * ========================================================
						 */

						/*
						 * Запрещаем обработчикам прерываний ADC1
						 * сохранять новые результаты.
						 */
						battery_adc_active = 0U;

						/*
						 * Останавливаем ADC1.
						 *
						 * Даже если преобразование уже остановлено,
						 * выполняем попытку завершить его работу.
						 */
						(void)HAL_ADC_Stop_IT(&hadc1);

						/*
						 * Останавливаем TIM3.
						 *
						 * Таймер мог быть остановлен ранее
						 * при обнаружении ошибки или таймаута.
						 */
						(void)HAL_TIM_Base_Stop_IT(&htim3);

						/*
						 * Сбрасываем признаки текущего измерения.
						 */
						battery_adc_ready = 0U;
						battery_adc_error = 0U;
						battery_adc_raw = 0U;

						/*
						 * Сбрасываем флаг таймаута.
						 */
						device_timeout = 0U;

						/*
						 * Активного прибора больше нет.
						 */
						logger->dev_type = DEV_NONE;

						/*
						 * Очищаем промежуточные результаты
						 * неудачного цикла измерений.
						 *
						 * Последнее достоверное значение
						 * logger->battery_voltage НЕ изменяем.
						 */
						logger->battery_adc_sum = 0U;
						logger->battery_adc_count = 0U;

						/*
						 * ========================================================
						 * 2. РЕГИСТРАЦИЯ ОШИБКИ ADC
						 * ========================================================
						 *
						 * Увеличиваем счётчик последовательных
						 * неудачных циклов измерения.
						 *
						 * Защищаем uint8_t от переполнения.
						 */
						if (logger->batt_error_counter < UINT8_MAX)
						{
								logger->batt_error_counter++;
						}

						/*
						 * Счётчик batt_low_voltage_counter здесь
						 * НЕ изменяем.
						 *
						 * Ошибка ADC не является достоверным
						 * измерением напряжения аккумулятора.
						 */

						/*
						 * ========================================================
						 * 3. ПРЕДПОЛЁТНАЯ ПРОВЕРКА — MODE_TEST
						 * ========================================================
						 *
						 * При ошибке измерения сообщаем оператору
						 * о неисправности аккумулятора или ADC.
						 *
						 * Предполётную проверку прекращаем.
						 *
						 * Переход в MODE_ERROR запрещён независимо
						 * от количества накопленных ошибок.
						 */
						if (logger->req_mode == MODE_TEST)
						{
								/*
								 * ID06 — аккумулятор неисправен,
								 * разведка невозможна.
								 *
								 * Используется стандартный Payload.
								 */
								(void)RadioLine_SendPacket(
										6U,
										NULL,
										0U
								);

								/*
								 * Остаёмся в MODE_TEST.
								 *
								 * Ожидаем команду оператора.
								 */
								logger->state = COMMAND_INFO;

								break;
						}

						/*
						 * ========================================================
						 * 4. РАБОЧИЙ РЕЖИМ — MODE_WORK
						 * ========================================================
						 */
						if (logger->req_mode == MODE_WORK)
						{
								/*
								 * Проверяем количество последовательных
								 * ошибок измерения аккумулятора.
								 */
								if (logger->batt_error_counter >=
										LOGGER_DEVICE_ERROR_LIMIT)
								{
										/*
										 * Достигнут установленный предел
										 * последовательных ошибок ADC.
										 *
										 * ID25 — дальнейшее проведение
										 * разведки невозможно.
										 *
										 * Тип неисправного прибора:
										 * 3 — аккумулятор.
										 */
										const uint8_t failed_device = 3U;

										(void)RadioLine_SendPacket(
												25U,
												&failed_device,
												1U
										);

										/*
										 * Записываем тот же ID25 на SD-карту.
										 * Ошибка записи не препятствует
										 * переходу в MODE_ERROR.
										 */
										(void)SD_Logger_WritePacket(
												25U,
												&failed_device,
												1U
										);

										/*
										 * Прекращаем рабочий режим.
										 */
										logger->req_mode = MODE_ERROR;

										/*
										 * Переходим к обработке команд
										 * согласно согласованной логике.
										 */
										logger->state = COMMAND_INFO;

										break;
								}

								/*
								 * Количество последовательных ошибок
								 * пока меньше установленного предела.
								 *
								 * Последнее достоверное напряжение
								 * аккумулятора сохранено.
								 *
								 * Продолжаем рабочий цикл через WORK_SD.
								 */
								logger->state = WORK_SD;

								break;
						}

						/*
						 * ========================================================
						 * 5. ЗАЩИТА ОТ НЕОЖИДАННОГО РЕЖИМА
						 * ========================================================
						 *
						 * Если ошибка измерения возникла в режиме,
						 * отличном от MODE_TEST и MODE_WORK,
						 * возвращаемся к ожиданию команд.
						 *
						 * Самостоятельно режим не изменяем.
						 */
						logger->state = COMMAND_INFO;

						break;
				}

				
				case WORK_SD:
				{
						/*
						 * ========================================================
						 * WORK_SD — РАБОТА С SD-КАРТОЙ
						 * ========================================================
						 *
						 * MODE_TEST:
						 *   Проверка работоспособности SD-карты.
						 *
						 * MODE_WORK:
						 *   Сохранение ID22 и завершение спектрального режима.
						 */

						/*
						 * ========================================================
						 * 1. ПРЕДПОЛЁТНАЯ ПРОВЕРКА — MODE_TEST
						 * ========================================================
						 */
						if (logger->req_mode == MODE_TEST)
						{
								/*
								 * Монтирование SD-карты уже выполнено в main.c.
								 *
								 * Проверяем возможность записи через временный файл:
								 * создание -> запись -> f_sync -> закрытие -> удаление.
								 *
								 * Основной CSV-файл разведки не создаём.
								 */
								SD_LoggerResult sd_result = SD_Logger_TestWrite();

								/*
								 * SD-КАРТА НЕИСПРАВНА.
								 */
								if (sd_result != SD_LOGGER_OK)
								{
										/*
										 * Забираем ожидающее событие ошибки SD,
										 * чтобы не отправить ID16 повторно.
										 */
										(void)SD_Logger_TakeErrorEvent();

										/*
										 * ID16 — SD-карта неисправна.
										 */
										(void)RadioLine_SendPacket(
												16U,
												NULL,
												0U
										);

										/*
										 * MODE_TEST сохраняем.
										 * Ожидаем решения оператора.
										 */
										logger->state = COMMAND_INFO;

										break;
								}

								/*
								 * SD-КАРТА ИСПРАВНА.
								 *
								 * Тест временного файла успешно завершён.
								 * Основной CSV-файл здесь не создаём.
								 */

								/*
								 * ID18 — система радиационной разведки исправна.
								 */
								(void)RadioLine_SendPacket(
										18U,
										NULL,
										0U
								);

								/*
								 * Предполётная проверка завершена.
								 * Переходим в режим ожидания разведки.
								 */
								logger->req_mode = MODE_EXPECT;
								logger->state = COMMAND_INFO;

								break;
						}

						/*
						 * ========================================================
						 * 2. ЗАВЕРШЕНИЕ СПЕКТРАЛЬНОГО РЕЖИМА
						 * ========================================================
						 *
						 * Существующую обработку ID23 не изменяем.
						 */
						if (logger->bdbg_spectrum_stopping != 0U)
						{
								/*
								 * ==========================================
								 * 1. СОХРАНЕНИЕ НАКОПЛЕННОГО СПЕКТРА
								 * ==========================================
								 *
								 * Выполняется при штатной и аварийной
								 * остановке, независимо от результата PAED_REQ.
								 */
								SD_LoggerResult sd_result =
										SD_WriteAccumulatedSpectrum(logger);

								/*
								 * Не допускаем бесконечных повторов
								 * при ошибке записи SD.
								 */
								(void)sd_result;

								/*
								 * ==========================================
								 * 2. АВАРИЙНОЕ ЗАВЕРШЕНИЕ СПЕКТРА
								 * ==========================================
								 */
								if (logger->bdbg_spectrum_error_shutdown != 0U)
								{
										const uint8_t failed_device = 0U;

										/*
										 * ID25 отправляется здесь один раз
										 * после попытки сохранения спектра.
										 *
										 * Это выполняется и при успешном PAED_REQ,
										 * и после исчерпания попыток PAED_REQ.
										 */
										(void)RadioLine_SendPacket(
												25U,
												&failed_device,
												1U
										);

										/*
										 * Сохраняем сообщение об отказе БДБГ.
										 */
										(void)SD_Logger_WritePacket(
												25U,
												&failed_device,
												1U
										);

										/*
										 * Дальнейшая разведка запрещена.
										 */
										logger->req_mode = MODE_ERROR;
								}

								/*
								 * ==========================================
								 * 3. ЗАВЕРШЕНИЕ СПЕКТРАЛЬНОГО РЕЖИМА
								 * ==========================================
								 *
								 * Сбрасываем признаки только после
								 * попытки сохранения спектра и ID25.
								 */
								logger->bdbg_spectrum_active = 0U;
								logger->bdbg_spectrum_stopping = 0U;
								logger->bdbg_spectrum_error_shutdown = 0U;

								/*
								 * Восстанавливаем обычный запрос БДБГ.
								 */
								logger->bdbg_request = PAED_REQ;

								/*
								 * Возвращаем предыдущий рабочий режим.
								 *
								 * При MODE_ERROR автоматическое
								 * выполнение измерений не возобновляется.
								 */
								logger->work_mode = logger->old_work_mode;

								/*
								 * Останавливаем интервал измерений.
								 */
								LoggerStateMachine_StopInterval(logger);

								logger->state = COMMAND_INFO;
								break;
						}

						/*
						 * ========================================================
						 * 3. РАБОЧИЙ РЕЖИМ — MODE_WORK
						 * ========================================================
						 *
						 * ID22 формируем только в режимах:
						 * MODE_OPERATOR, MODE_TIME, MODE_DISTANCE.
						 */
						if ((logger->req_mode == MODE_WORK) &&
								((logger->work_mode == MODE_OPERATOR) ||
								 (logger->work_mode == MODE_TIME) ||
								 (logger->work_mode == MODE_DISTANCE)))
						{
								/*
								 * Буфер полезной нагрузки ID22.
								 * Размер фиксирован: 31 байт.
								 */
								uint8_t payload[ID22_PAYLOAD_SIZE];

								/*
								 * Формируем ID22 из результатов завершённого
								 * цикла измерения.
								 */
								Logger_BuildID22Payload(logger, payload);

								/*
								 * ====================================================
								 * ФИКСАЦИЯ ОПОРНОЙ ТОЧКИ ИЗМЕРЕНИЯ
								 * ====================================================
								 *
								 * Сохраняем те же GPS-координаты и высоту LIDAR,
								 * которые использованы при формировании ID22.
								 *
								 * Эти значения станут опорными для следующего
								 * сравнения расстояния и высоты в MODE_DISTANCE.
								 *
								 * Фиксация точки не зависит от успешности
								 * радиопередачи или записи на SD-карту.
								 */

								/*
								 * Сохраняем GPS-координаты измеренной точки.
								 */
								logger->gps_data.last_saved_coord =
										logger->gps_data.current_coord;

								/*
								 * Сохраняем высоту LIDAR измеренной точки.
								 */
								logger->last_saved_lidar_height_cm =
										logger->lidar_data.distance_cm;

								/*
								 * Обе опорные величины теперь определены.
								 * Разрешаем сравнение при следующем измерении.
								 */
								logger->gps_data.has_last_saved_coord = 1U;
								logger->has_last_saved_lidar_height = 1U;

								/*
								 * ====================================================
								 * ПЕРЕДАЧА И СОХРАНЕНИЕ ID22
								 * ====================================================
								 */

								/*
								 * Передаём ID22 по RadioLine.
								 */
								(void)RadioLine_SendPacket(
										22U,
										payload,
										ID22_PAYLOAD_SIZE
								);

								/*
								 * Сохраняем ID22 в CSV-файл.
								 *
								 * Ошибка записи не отменяет фиксацию
								 * уже выполненного измерения.
								 */
								(void)SD_Logger_WritePacket(
										22U,
										payload,
										ID22_PAYLOAD_SIZE
								);

								/*
								 * Возвращаемся к обработке команд.
								 */
								logger->state = COMMAND_INFO;

								break;
						}

						/*
						 * ========================================================
						 * 4. ЗАЩИТА ОТ НЕОЖИДАННОГО РЕЖИМА
						 * ========================================================
						 */
						logger->state = COMMAND_INFO;

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