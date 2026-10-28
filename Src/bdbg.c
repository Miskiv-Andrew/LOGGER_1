#include "bdbg.h"
#include <stddef.h>

/*
 * Контрольная сумма БДБГ-15: сумма байтов с переносом старшего бита
 * обратно в младший. Алгоритм соответствует calcCrc() симулятора.
 */
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0U;

    /* Нулевой указатель не разыменовываем. */
    if (data == NULL)
    {
        return 0U;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        crc = (uint16_t)(crc + data[i]);

        /* Перенос из девятого бита прибавляем к младшим восьми. */
        if ((crc & 0x0100U) != 0U)
        {
            crc = (uint16_t)((crc & 0x00FFU) + 1U);
        }
    }

    return (uint8_t)crc;
}

/*
 * Создание одного запроса БДБГ-15.
 * Формат: 55 AA 70 <адрес> <команда> [09] <контрольная сумма>.
 * Дополнительный байт 09 присутствует только у START_SPECTRE_REQ.
 * UART4 здесь не используется: функция только готовит массив байтов.
 */
uint8_t BDBG_BuildRequest(const BdbgReq *request,
                          uint8_t output[BDBG_REQUEST_MAX_SIZE],
                          uint8_t *outputLength)
{
    uint8_t command;
    uint8_t packet[BDBG_REQUEST_MAX_SIZE];
    uint8_t length = 6U;

    /* Не допускаем обращения по нулевым указателям. */
    if ((request == NULL) || (output == NULL) || (outputLength == NULL))
    {
        return 0U;
    }

    /* Разрешаем только предусмотренные адреса детекторов. */
    if ((request->deviceAddr != BDBG_ADDR_LOWER) &&
        (request->deviceAddr != BDBG_ADDR_UPPER))
    {
        return 0U;
    }

    /* Командный байт определяется существующим перечислением. */
    switch (request->bdbgMode)
    {
        case PAED_REQ:
            command = 0x00U;
            break;

        case TEMP_REQ:
            command = 0x08U;
            break;

        case START_SPECTRE_REQ:
            command = 0x0BU;
            length = 7U;
            break;

        case GET_SPECTRE_REQ:
            command = 0x8BU;
            break;

        default:
            return 0U;
    }

    /* Постоянная часть кадра. */
    packet[0] = 0x55U;
    packet[1] = 0xAAU;
    packet[2] = 0x70U;
    packet[3] = request->deviceAddr;
    packet[4] = command;

    /* Для запуска спектра вставляем дополнительный байт 09. */
    if (request->bdbgMode == START_SPECTRE_REQ)
    {
        packet[5] = 0x09U;
    }

    /* Контрольная сумма — последний байт сформированного кадра. */
    packet[length - 1U] = BDBG_CalculateCrc(packet, (uint16_t)(length - 1U));

    /* Копируем готовый кадр только после всех проверок. */
    for (uint8_t i = 0U; i < length; ++i)
    {
        output[i] = packet[i];
    }

    *outputLength = length;
    return 1U;
}
