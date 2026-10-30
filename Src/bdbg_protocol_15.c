#include "main.h"
#include "bdbg_config.h"
#include "bdbg_protocol.h"
#include <stddef.h>
#include <string.h>
#if BDBG_MODEL == BDBG_MODEL_15
/* Сумма с циклическим переносом, идентичная Qt calcCrc(). */
uint8_t BDBG_ProtocolCalculateChecksum(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0U;
    if (data == NULL) return 0U;
    for (uint16_t i = 0U; i < length; ++i)
    {
        crc = (uint16_t)(crc + data[i]);
        if ((crc & 0x0100U) != 0U) crc = (uint16_t)((crc & 0xFFU) + 1U);
    }
    return (uint8_t)crc;
}
/* START: 55 AA 70 01 8B 09 8C 01 CRC
 * GET:   55 AA 70 01 8B 00 00 00 CRC */
uint8_t BDBG_ProtocolBuildRequest(uint8_t address, BdbgRequest command,
                                  uint8_t output[BDBG_REQUEST_MAX_SIZE], uint8_t *length)
{
    uint8_t n;
    if ((output == NULL) || (length == NULL)) return 0U;
    if ((address != 0x01U) && (address != 0x02U)) return 0U;
    output[0] = 0x55U; output[1] = 0xAAU;
    output[2] = 0x70U; output[3] = address;
    switch (command)
    {
        case PAED_REQ: output[4] = 0x00U; n = 6U; break;
        case TEMP_REQ: output[4] = 0x08U; n = 6U; break;
        case START_SPECTRE_REQ:
            output[4] = 0x8BU; output[5] = 0x09U;
            output[6] = 0x8CU; output[7] = 0x01U; n = 9U; break;
        case GET_SPECTRE_REQ:
            output[4] = 0x8BU; output[5] = 0x00U;
            output[6] = 0x00U; output[7] = 0x00U; n = 9U; break;
        default: return 0U;
    }
    output[n - 1U] = BDBG_ProtocolCalculateChecksum(output, (uint16_t)(n - 1U));
    *length = n;
    return 1U;
}
#endif
