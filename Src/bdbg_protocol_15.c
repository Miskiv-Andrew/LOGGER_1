#include "main.h"
#include "bdbg_config.h"
#include "bdbg_protocol.h"
#include <stddef.h>
#include <string.h>

#if BDBG_MODEL == BDBG_MODEL_15

/*
 * Контрольная сумма БДБГ-15: сумма байтов с циклическим переносом.
 * Точное соответствие calcCrc() из Qt-симулятора.
 * Байт контрольной суммы во входной массив НЕ включается.
 */
uint8_t BDBG_ProtocolCalculateChecksum(const uint8_t *data, uint16_t length)
{
    uint16_t checksum = 0U;

    if (data == NULL)
    {
        return 0U;
    }

    for (uint16_t i = 0U; i < length; ++i)
    {
        checksum = (uint16_t)(checksum + data[i]);
        if ((checksum & 0x0100U) != 0U)
        {
            checksum = (uint16_t)((checksum & 0x00FFU) + 1U);
        }
    }

    return (uint8_t)checksum;
}

/*
 * Команды БДБГ-15:
 *   PAED:          55 AA 70 ADDR 00 CRC
 *   TEMP:          55 AA 70 ADDR 08 CRC
 *   START_SPECTRE: 55 AA 70 ADDR 0B 09 CRC
 *   GET_SPECTRE:   55 AA 70 ADDR 8B 00 CRC
 *
 * Функция не обращается к UART: здесь только байтовый протокол.
 */
uint8_t BDBG_ProtocolBuildRequest(uint8_t device_address,
                                  BdbgRequest command,
                                  uint8_t output[BDBG_REQUEST_MAX_SIZE],
                                  uint8_t *output_length)
{
    uint8_t packet[BDBG_REQUEST_MAX_SIZE];
    uint8_t command_byte = 0U;
    uint8_t packet_length = 6U;

    if ((output == NULL) || (output_length == NULL))
    {
        return 0U;
    }

    /* Пока поддерживаются адреса нижнего и верхнего детектора. */
    if ((device_address != 0x01U) && (device_address != 0x02U))
    {
        return 0U;
    }

    switch (command)
    {
        case PAED_REQ:          command_byte = 0x00U; break;
        case TEMP_REQ:          command_byte = 0x08U; break;
        case START_SPECTRE_REQ: command_byte = 0x0BU; packet_length = 7U; break;
        case GET_SPECTRE_REQ:   command_byte = 0x8BU; packet_length = 7U; break;
        default: return 0U;
    }

    packet[0] = 0x55U;
    packet[1] = 0xAAU;
    packet[2] = 0x70U;
    packet[3] = device_address;
    packet[4] = command_byte;

    if ((command == START_SPECTRE_REQ) || (command == GET_SPECTRE_REQ))
    {
        packet[5] = (command == START_SPECTRE_REQ) ? 0x09U : 0x00U;
    }

    packet[packet_length - 1U] =
        BDBG_ProtocolCalculateChecksum(packet, (uint16_t)(packet_length - 1U));

    memcpy(output, packet, packet_length);
    *output_length = packet_length;
    return 1U;
}

#endif /* BDBG_MODEL == BDBG_MODEL_15 */
