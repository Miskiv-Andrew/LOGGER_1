#ifndef BDBG_PROTOCOL_H
#define BDBG_PROTOCOL_H
#include <stdint.h>
#include "logger_types.h"
/*
 * Интерфейс протокола БДБГ.
 * Реализация для каждой модели — отдельный файл bdbg_protocol_NN.c,
 * модель выбирается в bdbg_config.h.
 * bdbg.c (UART, DMA) и стейт-машина от модели не зависят.
 */
#define BDBG_REQUEST_MAX_SIZE 9U
#define BDBG_SPECTRUM_CHANNELS 1024U
/* Максимальная длина ответа прибора. */
#define BDBG_SPECTRUM_RESPONSE_SIZE 2076U
/* Запас приёмного буфера bdbg.c на лишние байты до и после ответа. */
#define BDBG_RX_MARGIN 8U
/* Данные прибора после разбора ответа. Поля, которых нет у модели, не заполняются. */
typedef struct
{
    float paed_usv_h;
    uint8_t status;
    uint16_t temperature_raw;
    uint16_t acquisition_time;
    uint16_t cps;
    uint8_t test;
    uint8_t device_id;
    uint32_t serial_number;
    uint8_t year, month, version_major, version_minor;
    uint16_t spectrum[BDBG_SPECTRUM_CHANNELS];
} BDBG_Data;
uint8_t BDBG_ProtocolBuildRequest(uint8_t address, BdbgRequest command,
                                  uint8_t output[BDBG_REQUEST_MAX_SIZE], uint8_t *length);
uint8_t BDBG_ProtocolCalculateChecksum(const uint8_t *data, uint16_t length);
/* Ожидаемая длина ответа на запрос; 0 — запрос не поддерживается. */
uint16_t BDBG_ProtocolResponseSize(BdbgRequest command);
/* Проверка и разбор ответа: 1 — ответ корректен, data заполнена. */
uint8_t BDBG_ProtocolParseResponse(BdbgRequest command, const uint8_t *rx,
                                   uint16_t length, BDBG_Data *data);
#endif