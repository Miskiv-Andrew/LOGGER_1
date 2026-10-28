#ifndef BDBG_H
#define BDBG_H

#include <stdint.h>
#include "logger_types.h"

/* Адреса блоков детектирования БДБГ-15. */
#define BDBG_ADDR_LOWER          0x01U
#define BDBG_ADDR_UPPER          0x02U

/* Максимальная длина команды: START_SPECTRE_REQ содержит 7 байтов. */
#define BDBG_REQUEST_MAX_SIZE    7U

/*
 * Параметры формируемого запроса.
 * expectedLength оставлен 16-битным: ответ со спектром длиннее 255 байтов.
 * На первом этапе поле не используется; длину ответа согласуем отдельно.
 */
typedef struct
{
    uint8_t deviceAddr;
    BdbgRequest bdbgMode;
    uint16_t expectedLength;
} BdbgReq;

/*
 * Вычисляет 8-битную контрольную сумму по алгоритму симулятора БДБГ-15.
 * Входной массив не должен содержать байт самой контрольной суммы.
 */
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length);

/*
 * Формирует команду длиной 6 или 7 байтов.
 * Фактическая длина возвращается через outputLength.
 * Возвращает 1 при успехе, 0 при ошибке аргументов или типа запроса.
 * При ошибке содержимое output не изменяется.
 */
uint8_t BDBG_BuildRequest(const BdbgReq *request,
                          uint8_t output[BDBG_REQUEST_MAX_SIZE],
                          uint8_t *outputLength);

#endif /* BDBG_H */
