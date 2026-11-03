#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include "main.h"
#include "ff.h"

#include <stdint.h>

/*
 * ============================================================
 * Модуль регистрации данных LOGGER на SD-карте.
 * ============================================================
 *
 * Формат каталога:
 *
 *   /YYMMDD
 *
 * Формат имени файла:
 *
 *   HHMMSS.CSV
 *
 * Формат строки:
 *
 *   ID,XX,XX,XX,...\r\n
 *
 * Пример:
 *
 *   19,01,AB,FF\r\n
 *
 * Модуль не управляет состояниями LOGGER.
 * Все решения о том, какие пакеты записывать,
 * принимает вызывающая стейт-машина.
 */


/*
 * Максимальное количество байтов полезной нагрузки,
 * которое можно передать за один вызов записи.
 *
 * Значение выбрано с запасом для полного спектра
 * из 1024 каналов при представлении uint32_t.
 *
 * При необходимости ограничение можно пересмотреть
 * после проверки фактического формата пакета ID23.
 */
#define SD_LOGGER_MAX_PAYLOAD_SIZE  4096U


/*
 * Результат выполнения операции.
 */
typedef enum
{
    SD_LOGGER_OK = 0,

    /*
     * SD-карта недоступна или запись отключена.
     */
    SD_LOGGER_DISABLED,

    /*
     * Файл ещё не открыт.
     */
    SD_LOGGER_NO_FILE,

    /*
     * Переданы некорректные параметры.
     */
    SD_LOGGER_INVALID_ARGUMENT,

    /*
     * Ошибка файловой системы или SD-карты.
     */
    SD_LOGGER_ERROR

} SD_LoggerResult;


/*
 * Инициализация программного состояния модуля.
 *
 * Не вызывает f_mount().
 *
 * Файловая система должна быть смонтирована
 * до начала использования модуля.
 */
void SD_Logger_Init(void);


/*
 * Передаёт модулю результат монтирования SD-карты.
 *
 * Вызывается после f_mount().
 */
void SD_Logger_SetMountResult(FRESULT result);


/*
 * Создаёт файл для новой конфигурации полёта.
 *
 * Дата и время передаются из RTC в двоичном виде:
 *
 * year   = 0..99 (2000..2099)
 * month  = 1..12
 * day    = 1..31
 * hour   = 0..23
 * minute = 0..59
 * second = 0..59
 */
SD_LoggerResult SD_Logger_OpenFlight(
    uint8_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second
);


/*
 * Записывает одну CSV-строку.
 *
 * packet_id — идентификатор пакета.
 * payload   — указатель на данные.
 * size      — количество байтов данных.
 *
 * После записи выполняется f_sync().
 */
SD_LoggerResult SD_Logger_WritePacket(
    uint8_t packet_id,
    const uint8_t *payload,
    uint16_t size
);


/*
 * Закрывает текущий файл.
 */
SD_LoggerResult SD_Logger_Close(void);


/*
 * Возвращает 1, если запись разрешена.
 */
uint8_t SD_Logger_IsEnabled(void);


/*
 * Возвращает 1, если файл открыт.
 */
uint8_t SD_Logger_IsFileOpen(void);


/*
 * Возвращает последнюю ошибку FatFs.
 */
FRESULT SD_Logger_GetLastFatFsError(void);


/*
 * Возвращает 1, если возникла новая ошибка SD.
 *
 * Чтение сбрасывает признак новой ошибки.
 *
 * Стейт-машина использует этот признак
 * для однократной отправки пакета ID16.
 */
uint8_t SD_Logger_TakeErrorEvent(void);

#endif