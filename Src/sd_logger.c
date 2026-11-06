#include "sd_logger.h"

#include <stddef.h>

/*
 * ============================================================
 * Внутренние настройки модуля.
 * ============================================================
 */

/*
 * Размер промежуточного буфера записи.
 *
 * Мы не формируем всю строку спектра в оперативной памяти.
 * Данные преобразуются в HEX небольшими порциями.
 *
 * Это особенно важно для STM32 с ограниченным объёмом RAM.
 */
#define SD_LOGGER_BUFFER_SIZE  96U


/*
 * ============================================================
 * Внутреннее состояние.
 * ============================================================
 */

/*
 * Объект открытого файла.
 *
 * Используем собственный объект FIL.
 * Глобальный USERFile из fatfs.c не требуется.
 */
static FIL sd_file;

/*
 * Разрешение операций записи.
 *
 * 0 — запись запрещена;
 * 1 — запись разрешена.
 */
static uint8_t write_card_flag = 0U;

/*
 * Признак открытого файла.
 */
static uint8_t sd_file_open = 0U;

/*
 * Признак новой ошибки.
 *
 * Устанавливается при первом обнаружении ошибки.
 * Стейт-машина считывает его через
 * SD_Logger_TakeErrorEvent().
 */
static uint8_t sd_error_event = 0U;

/*
 * Последняя ошибка FatFs.
 */
static FRESULT sd_last_error = FR_OK;

/*
 * Промежуточный буфер записи.
 */
static uint8_t sd_buffer[SD_LOGGER_BUFFER_SIZE];


/*
 * ============================================================
 * Внутренние вспомогательные функции.
 * ============================================================
 */


/*
 * Преобразование числа 0..15 в символ HEX.
 *
 * Используются заглавные буквы A..F.
 */
static char SD_Logger_HexDigit(uint8_t value)
{
    static const char digits[] = "0123456789ABCDEF";

    return digits[value & 0x0FU];
}


/*
 * Запись двух десятичных цифр.
 *
 * Применяется при формировании даты и времени.
 *
 * Например:
 *   7  -> "07"
 *   25 -> "25"
 */
static void SD_Logger_WriteTwoDigits(
    char *destination,
    uint8_t value
)
{
    destination[0] = (char)('0' + value / 10U);
    destination[1] = (char)('0' + value % 10U);
}


/*
 * Фиксируем ошибку SD-карты.
 *
 * После первой ошибки:
 *   1. Запрещаем дальнейшую запись.
 *   2. Сохраняем код ошибки.
 *   3. Устанавливаем событие для стейт-машины.
 *
 * Повторная инициализация автоматически не выполняется.
 */
static void SD_Logger_SetError(FRESULT error)
{
    /*
     * Сохраняем код первой ошибки, которая привела
     * к отключению записи.
     */
    if (write_card_flag != 0U)
    {
        sd_last_error = error;
    }

    /*
     * Создаём событие ошибки только один раз.
     *
     * Это предотвращает многократную отправку ID16
     * из-за одной неисправности SD-карты.
     */
    if (sd_error_event == 0U &&
        write_card_flag != 0U)
    {
        sd_error_event = 1U;
    }

    write_card_flag = 0U;
}


/*
 * Запись произвольного блока байтов в файл.
 *
 * Проверяем:
 *   - результат f_write();
 *   - фактическое количество записанных байтов.
 *
 * f_sync() здесь не вызывается.
 * Синхронизация выполняется после полной CSV-строки.
 */
static SD_LoggerResult SD_Logger_WriteRaw(
    const void *data,
    UINT size
)
{
    UINT written = 0U;

    FRESULT result;

    /*
     * При отключённой SD-карте операции запрещены.
     */
    if (write_card_flag == 0U)
    {
        return SD_LOGGER_DISABLED;
    }

    /*
     * Проверяем наличие открытого файла.
     */
    if (sd_file_open == 0U)
    {
        return SD_LOGGER_NO_FILE;
    }

    /*
     * Нулевой размер допустим:
     * ничего записывать не требуется.
     */
    if (size == 0U)
    {
        return SD_LOGGER_OK;
    }

    /*
     * Передаём данные файловой системе.
     */
    result = f_write(
        &sd_file,
        data,
        size,
        &written
    );

    /*
     * Ошибка FatFs.
     */
    if (result != FR_OK)
    {
        SD_Logger_SetError(result);

        return SD_LOGGER_ERROR;
    }

    /*
     * Даже при FR_OK проверяем количество байтов.
     *
     * Неполная запись считается ошибкой.
     * В частности, она возможна при заполнении карты.
     */
    if (written != size)
    {
        SD_Logger_SetError(FR_DISK_ERR);

        return SD_LOGGER_ERROR;
    }

    return SD_LOGGER_OK;
}


/*
 * ============================================================
 * Публичные функции модуля.
 * ============================================================
 */


/*
 * Инициализация программного состояния.
 *
 * Вызывать один раз при запуске LOGGER,
 * до SD_Logger_SetMountResult().
 */
void SD_Logger_Init(void)
{
    write_card_flag = 0U;

    sd_file_open = 0U;

    sd_error_event = 0U;

    sd_last_error = FR_OK;
}


/*
 * Передаём результат уже выполненного f_mount().
 *
 * Повторно монтировать карту не требуется.
 */
void SD_Logger_SetMountResult(FRESULT result)
{
    sd_last_error = result;

    if (result == FR_OK)
    {
        write_card_flag = 1U;

        sd_error_event = 0U;
    }
    else
    {
        write_card_flag = 0U;

        /*
         * Ошибка монтирования также должна быть
         * передана стейт-машине для сообщения ID16.
         */
        sd_error_event = 1U;
    }
}


/*
 * ============================================================
 * SD_Logger_TestWrite
 * ============================================================
 *
 * Проверка возможности записи на SD-карту.
 *
 * Выполняется в MODE_TEST после успешного монтирования.
 *
 * Алгоритм:
 *
 * 1. Проверяем доступность SD-карты.
 * 2. Создаём временный служебный файл.
 * 3. Записываем контрольные данные.
 * 4. Выполняем f_sync().
 * 5. Закрываем файл.
 * 6. Удаляем временный файл.
 *
 * При любой ошибке отключаем дальнейшую запись
 * через существующий механизм SD_Logger_SetError().
 *
 * Проверка не изменяет состояние основного CSV-файла.
 * ============================================================
 */
SD_LoggerResult SD_Logger_TestWrite(void)
{
    /*
     * Отдельный файловый объект.
     *
     * Основной sd_file не используем, поскольку
     * он предназначен для CSV-файла разведки.
     */
    FIL test_file;

    /*
     * Имя временного файла.
     *
     * Используем корневой каталог SD-карты.
     * Имя соответствует формату FAT 8.3.
     */
    const char test_filename[] = "0:/SDTEST.TMP";

    /*
     * Контрольные данные для проверки записи.
     */
    const uint8_t test_data[] =
    {
        0x53U,  /* S */
        0x44U,  /* D */
        0x54U,  /* T */
        0x45U,  /* E */
        0x53U,  /* S */
        0x54U   /* T */
    };

    FRESULT result;
    FRESULT close_result;

    UINT bytes_written = 0U;

    /*
     * ========================================================
     * 1. ПРОВЕРКА ДОСТУПНОСТИ
     * ========================================================
     */

    /*
     * Если монтирование ранее завершилось ошибкой
     * или запись уже была отключена, повторную
     * проверку не выполняем.
     */
    if (write_card_flag == 0U)
    {
        return SD_LOGGER_DISABLED;
    }

    /*
     * Не выполняем служебную проверку, если
     * основной файл разведки уже открыт.
     *
     * В нормальном MODE_TEST этого быть не должно.
     */
    if (sd_file_open != 0U)
    {
        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * 2. СОЗДАНИЕ ВРЕМЕННОГО ФАЙЛА
     * ========================================================
     *
     * FA_CREATE_ALWAYS:
     * создаёт файл либо перезаписывает существующий.
     *
     * Это допустимо только для выделенного
     * служебного файла SDTEST.TMP.
     */
    result = f_open(
        &test_file,
        test_filename,
        FA_CREATE_ALWAYS | FA_WRITE
    );

    if (result != FR_OK)
    {
        SD_Logger_SetError(result);
        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * 3. ЗАПИСЬ КОНТРОЛЬНЫХ ДАННЫХ
     * ========================================================
     */
    result = f_write(
        &test_file,
        test_data,
        sizeof(test_data),
        &bytes_written
    );

    /*
     * Проверяем как результат FatFs,
     * так и фактическое количество записанных байтов.
     */
    if ((result != FR_OK) ||
        (bytes_written != sizeof(test_data)))
    {
        /*
         * Если FatFs вернул FR_OK, но записал
         * меньше байтов, регистрируем ошибку диска.
         */
        if (result == FR_OK)
        {
            result = FR_DISK_ERR;
        }

        /*
         * Пытаемся закрыть служебный файл.
         * Первоначальную ошибку сохраняем.
         */
        (void)f_close(&test_file);

        SD_Logger_SetError(result);

        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * 4. СИНХРОНИЗАЦИЯ ДАННЫХ
     * ========================================================
     *
     * f_sync() передаёт FatFs требование
     * завершить запись данных и метаданных.
     */
    result = f_sync(&test_file);

    if (result != FR_OK)
    {
        (void)f_close(&test_file);

        SD_Logger_SetError(result);

        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * 5. ЗАКРЫТИЕ ФАЙЛА
     * ========================================================
     */
    close_result = f_close(&test_file);

    if (close_result != FR_OK)
    {
        SD_Logger_SetError(close_result);

        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * 6. УДАЛЕНИЕ ВРЕМЕННОГО ФАЙЛА
     * ========================================================
     */
    result = f_unlink(test_filename);

    if (result != FR_OK)
    {
        SD_Logger_SetError(result);

        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * 7. УСПЕШНОЕ ЗАВЕРШЕНИЕ
     * ========================================================
     *
     * Все операции файловой системы завершились
     * успешно.
     *
     * write_card_flag уже установлен после
     * успешного монтирования SD-карты.
     *
     * Дополнительно изменять его не требуется.
     */
    return SD_LOGGER_OK;
}

/*
 * ============================================================
 * Создание нового CSV-файла для FlightConfig.
 *
 * Каталог: 0:/YYMMDD
 * Файл:    HHMMSS.CSV
 *
 * ВАЖНО:
 * Используем FA_CREATE_NEW, чтобы никогда
 * не перезаписывать существующий файл.
 * ============================================================
 */
SD_LoggerResult SD_Logger_OpenFlight(
    uint8_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second
)
{
    /*
     * Каталог:
     *
     * 0:/YYMMDD
     *
     * Последний символ строки — '\0'.
     */
    char directory[10] = "0:/000000";

    /*
     * Полный путь:
     *
     * 0:/YYMMDD/HHMMSS.CSV
     */
    char filename[21] = "0:/000000/000000.CSV";

    FRESULT result;

    /*
     * Если запись на SD-карту отключена,
     * создание файла не выполняем.
     */
    if (write_card_flag == 0U)
    {
        return SD_LOGGER_DISABLED;
    }

    /*
     * Проверяем диапазоны даты и времени.
     *
     * Полную календарную проверку даты
     * выполняет модуль RTC.
     */
    if (year > 99U ||
        month < 1U || month > 12U ||
        day < 1U || day > 31U ||
        hour > 23U ||
        minute > 59U ||
        second > 59U)
    {
        return SD_LOGGER_INVALID_ARGUMENT;
    }

    /*
     * Если предыдущий CSV-файл открыт,
     * сначала закрываем его.
     *
     * Это необходимо при получении
     * нового корректного FlightConfig.
     */
    if (sd_file_open != 0U)
    {
        SD_LoggerResult close_result =
            SD_Logger_Close();

        /*
         * При ошибке закрытия предыдущего
         * файла новый файл не открываем.
         */
        if (close_result != SD_LOGGER_OK)
        {
            return close_result;
        }
    }

    /*
     * ========================================================
     * Формирование каталога YYMMDD.
     * ========================================================
     */

    SD_Logger_WriteTwoDigits(
        &directory[3], year
    );

    SD_Logger_WriteTwoDigits(
        &directory[5], month
    );

    SD_Logger_WriteTwoDigits(
        &directory[7], day
    );

    /*
     * ========================================================
     * Формирование полного имени файла.
     * ========================================================
     *
     * 0:/YYMMDD/HHMMSS.CSV
     */

    /*
     * Дата: позиции 3–8.
     */
    SD_Logger_WriteTwoDigits(
        &filename[3], year
    );

    SD_Logger_WriteTwoDigits(
        &filename[5], month
    );

    SD_Logger_WriteTwoDigits(
        &filename[7], day
    );

    /*
     * Время: позиции 10–15.
     */
    SD_Logger_WriteTwoDigits(
        &filename[10], hour
    );

    SD_Logger_WriteTwoDigits(
        &filename[12], minute
    );

    SD_Logger_WriteTwoDigits(
        &filename[14], second
    );

    /*
     * ========================================================
     * Создание каталога.
     * ========================================================
     *
     * FR_OK:
     * каталог успешно создан.
     *
     * FR_EXIST:
     * каталог уже существует.
     * Это нормальная ситуация.
     */
    result = f_mkdir(directory);

    if (result != FR_OK && result != FR_EXIST)
    {
        /*
         * Регистрируем ошибку FatFs.
         *
         * SD_Logger_SetError() отключает запись
         * и устанавливает событие ошибки SD.
         */
        SD_Logger_SetError(result);

        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * Создание нового CSV-файла.
     * ========================================================
     *
     * FA_WRITE:
     * разрешаем запись.
     *
     * FA_CREATE_NEW:
     * создаём только новый файл.
     *
     * Если файл уже существует, FatFs
     * возвращает FR_EXIST.
     *
     * Существующий файл не изменяется.
     */
    result = f_open(
        &sd_file,
        filename,
        FA_WRITE | FA_CREATE_NEW
    );

    /*
     * Проверяем результат создания файла.
     */
    if (result != FR_OK)
    {
        /*
         * В том числе обрабатываем FR_EXIST.
         *
         * Регистрируем ошибку SD.
         * Запись будет отключена.
         *
         * При этом существующий CSV-файл
         * останется без изменений.
         */
        SD_Logger_SetError(result);

        return SD_LOGGER_ERROR;
    }

    /*
     * ========================================================
     * Файл успешно создан и открыт.
     * ========================================================
     */

    sd_file_open = 1U;

    return SD_LOGGER_OK;
}

/*
 * Запись одного пакета в CSV.
 *
 * Пример:
 *
 * packet_id = 22 (0x16)
 * payload   = {0x01, 0xA5, 0xFF}
 *
 * Результат:
 *
 * 16,01,A5,FF\r\n
 *
 * Важно:
 * payload содержит только полезную нагрузку.
 * Идентификатор пакета передаётся отдельно.
 */
SD_LoggerResult SD_Logger_WritePacket( uint8_t packet_id, const uint8_t *payload, uint16_t size )
{
    UINT pos = 0U;

    uint16_t i;

    SD_LoggerResult write_result;

    FRESULT sync_result;

    /*
     * Проверяем состояние SD-карты.
     */
    if (write_card_flag == 0U)
    {
        return SD_LOGGER_DISABLED;
    }

    /*
     * Проверяем наличие открытого файла.
     */
    if (sd_file_open == 0U)
    {
        return SD_LOGGER_NO_FILE;
    }

    /*
     * Проверяем размер полезной нагрузки.
     */
    if (size > SD_LOGGER_MAX_PAYLOAD_SIZE)
    {
        return SD_LOGGER_INVALID_ARGUMENT;
    }

    /*
     * Нулевой размер разрешён.
     *
     * Но если размер больше нуля,
     * указатель payload должен быть действительным.
     */
    if (size > 0U && payload == NULL)
    {
        return SD_LOGGER_INVALID_ARGUMENT;
    }

       /*
     * Записываем идентификатор пакета
     * двумя шестнадцатеричными символами.
     *
     * Например:
     *
     * ID19 (десятичное 19) -> 13
     * ID20 (десятичное 20) -> 14
     * ID22 (десятичное 22) -> 16
     * ID23 (десятичное 23) -> 17
     * ID25 (десятичное 25) -> 19
     */
    sd_buffer[pos++] =
        (uint8_t)SD_Logger_HexDigit(packet_id >> 4);

    sd_buffer[pos++] =
        (uint8_t)SD_Logger_HexDigit(packet_id);

    /*
     * ========================================================
     * Преобразуем полезную нагрузку в HEX.
     * ========================================================
     *
     * Перед каждым байтом добавляем запятую.
     *
     * Каждый байт занимает три символа:
     *
     * ,XX
     *
     * Если буфер заполняется, записываем его
     * в файл и продолжаем преобразование.
     */

    for (i = 0U; i < size; i++)
    {
        /*
         * Проверяем наличие трёх свободных байтов.
         */
        if ((SD_LOGGER_BUFFER_SIZE - pos) < 3U)
        {
            write_result = SD_Logger_WriteRaw(
                sd_buffer,
                pos
            );

            if (write_result != SD_LOGGER_OK)
            {
                return write_result;
            }

            pos = 0U;
        }

        /*
         * Разделитель.
         */
        sd_buffer[pos++] = ',';

        /*
         * Старшие четыре бита.
         */
        sd_buffer[pos++] =
            (uint8_t)SD_Logger_HexDigit(
                payload[i] >> 4
            );

        /*
         * Младшие четыре бита.
         */
        sd_buffer[pos++] =
            (uint8_t)SD_Logger_HexDigit(
                payload[i]
            );
    }

    /*
     * ========================================================
     * Завершаем CSV-строку.
     * ========================================================
     */

    if ((SD_LOGGER_BUFFER_SIZE - pos) < 2U)
    {
        write_result = SD_Logger_WriteRaw(
            sd_buffer,
            pos
        );

        if (write_result != SD_LOGGER_OK)
        {
            return write_result;
        }

        pos = 0U;
    }

    /*
     * Окончание строки Windows:
     *
     * CR LF
     */
    sd_buffer[pos++] = '\r';
    sd_buffer[pos++] = '\n';

    /*
     * Записываем оставшуюся часть строки.
     */
    write_result = SD_Logger_WriteRaw(
        sd_buffer,
        pos
    );

    if (write_result != SD_LOGGER_OK)
    {
        return write_result;
    }

    /*
     * ========================================================
     * Принудительная синхронизация файла.
     * ========================================================
     *
     * FatFs записывает данные и обновляет
     * необходимые структуры файловой системы.
     *
     * Выполняем после каждой полной строки.
     */
    sync_result = f_sync(&sd_file);

    if (sync_result != FR_OK)
    {
        SD_Logger_SetError(sync_result);

        return SD_LOGGER_ERROR;
    }

    return SD_LOGGER_OK;
}














/*
 * Закрытие текущего файла.
 *
 * Используется:
 *   - при новом FlightConfig;
 *   - при переходе в MODE_RETURN;
 *   - при переходе в MODE_ERROR.
 */
SD_LoggerResult SD_Logger_Close(void)
{
    FRESULT result;

    /*
     * Если файл не открыт, закрывать нечего.
     */
    if (sd_file_open == 0U)
    {
        return SD_LOGGER_OK;
    }

    /*
     * Если запись уже отключена из-за ошибки,
     * не выполняем дополнительных операций FatFs.
     *
     * Это не является успешным закрытием файла.
     * Повторное открытие запрещено до нового запуска.
     */
    if (write_card_flag == 0U)
    {
        return SD_LOGGER_DISABLED;
    }

    /*
     * f_close() записывает необходимые данные
     * файловой системы и освобождает объект файла.
     */
    result = f_close(&sd_file);

    if (result != FR_OK)
    {
        SD_Logger_SetError(result);

        return SD_LOGGER_ERROR;
    }

    /*
     * Файл успешно закрыт.
     */
    sd_file_open = 0U;

    return SD_LOGGER_OK;
}


/*
 * Проверка разрешения записи.
 */
uint8_t SD_Logger_IsEnabled(void)
{
    return write_card_flag;
}


/*
 * Проверка открытого файла.
 */
uint8_t SD_Logger_IsFileOpen(void)
{
    return sd_file_open;
}


/*
 * Получение последней ошибки FatFs.
 */
FRESULT SD_Logger_GetLastFatFsError(void)
{
    return sd_last_error;
}


/*
 * Получение события ошибки.
 *
 * Возвращает 1 один раз после возникновения ошибки.
 *
 * Повторные вызовы возвращают 0 до появления
 * следующего события.
 */
uint8_t SD_Logger_TakeErrorEvent(void)
{
    uint8_t event = sd_error_event;

    sd_error_event = 0U;

    return event;
}