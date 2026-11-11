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
/* Длины ответов БДБГ-15С-23. */
uint16_t BDBG_ProtocolResponseSize(BdbgRequest command)
{
    switch (command)
    {
        case PAED_REQ:          return 12U;
        case TEMP_REQ:          return 8U;
        case START_SPECTRE_REQ:
        case GET_SPECTRE_REQ:   return BDBG_SPECTRUM_RESPONSE_SIZE;
        default:                return 0U;
    }
}
static uint16_t read_u16(const uint8_t *p)
{ return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t read_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static float read_float(const uint8_t *p)
{ uint32_t raw = read_u32(p); float f; memcpy(&f, &raw, sizeof(f)); return f; }
/* Ответы (протокол БДБГ-15С-23, адрес всегда BDBG_DEFAULT_ADDRESS):
 * PAED:  55 AA 70 01 01 ПАЕД(float) Stat Test CRC                 — 12 байт
 * TEMP:  55 AA 70 01 08 Temp(uint16) CRC                           — 8 байт
 * START: 55 AA 70 01 8B 09 8C 01, 2067 резервных байт, CRC        — 2076 байт
 * GET:   55 AA 70 01 8B 00, 1024 канала, AccTime(2054) ПАЕД(2056)
 *        Stat(2060) Test(2061) Temp(2062) CPS(2064) Dev(2066)
 *        Serial(2067) Год(2071) Мес(2072) Верс(2073, 2074) CRC(2075) — 2076 байт */
uint8_t BDBG_ProtocolParseResponse(BdbgRequest command, const uint8_t *rx,
                                   uint16_t length, BDBG_Data *data)
{
    if ((rx == NULL) || (data == NULL)) return 0U;
    if ((length == 0U) || (length != BDBG_ProtocolResponseSize(command))) return 0U;
    if ((rx[0] != 0x55U) || (rx[1] != 0xAAU) ||
        (rx[2] != 0x70U) || (rx[3] != BDBG_DEFAULT_ADDRESS)) return 0U;
    if (BDBG_ProtocolCalculateChecksum(rx, (uint16_t)(length - 1U)) != rx[length - 1U]) return 0U;
    switch (command)
    {
        case START_SPECTRE_REQ:
            /* Подтверждение старта повторяет команду. Данных нет, data не изменяется. */
            if ((rx[4] != 0x8BU) || (rx[5] != 0x09U) ||
                (rx[6] != 0x8CU) || (rx[7] != 0x01U)) return 0U;
            return 1U;
        case GET_SPECTRE_REQ:
            if ((rx[4] != 0x8BU) || (rx[5] != 0x00U)) return 0U;
            for (uint16_t i = 0U; i < BDBG_SPECTRUM_CHANNELS; ++i)
                data->spectrum[i] = read_u16(&rx[6U + 2U * i]);
            data->acquisition_time = read_u16(&rx[2054]);
            data->paed_usv_h = read_float(&rx[2056]);
            data->status = rx[2060]; data->test = rx[2061];
            data->temperature_raw = read_u16(&rx[2062]);
            data->cps = read_u16(&rx[2064]); data->device_id = rx[2066];
            data->serial_number = read_u32(&rx[2067]);
            data->year = rx[2071]; data->month = rx[2072];
            data->version_major = rx[2073]; data->version_minor = rx[2074];
            return 1U;
        case PAED_REQ:
            if (rx[4] != 0x01U) return 0U;
            data->paed_usv_h = read_float(&rx[5]); data->status = rx[9];
            return 1U;
        case TEMP_REQ:
            if (rx[4] != 0x08U) return 0U;
            data->temperature_raw = read_u16(&rx[5]);
            return 1U;
        default:
            return 0U;
    }
}
#endif