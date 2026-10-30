#ifndef BDBG_H
#define BDBG_H

#include <stdint.h>
#include "logger_types.h"
#include "bdbg_protocol.h"

/* Сохранены прежние имена и интерфейс этапа 1. */
#define BDBG_ADDR_LOWER 0x01U
#define BDBG_ADDR_UPPER 0x02U

/*
 * expectedLength — 16 бит: ответы со спектром превышают 255 байтов.
 * Поле будет использовано при реализации приёма DMA.
 */
typedef struct
{
    uint8_t deviceAddr;
    BdbgRequest bdbgMode;
    uint16_t expectedLength;
} BdbgReq;

/* Совместимость с существующим кодом: реализации делегируют протоколу. */
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length);
uint8_t BDBG_BuildRequest(const BdbgReq *request,
                          uint8_t output[BDBG_REQUEST_MAX_SIZE],
                          uint8_t *outputLength);

#endif /* BDBG_H */
