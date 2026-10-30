#ifndef BDBG_PROTOCOL_H
#define BDBG_PROTOCOL_H

#include <stdint.h>
#include "logger_types.h"

/* Общий интерфейс протоколов семейства БДБГ. */
#define BDBG_REQUEST_MAX_SIZE  7U

/*
 * Сформировать команду для выбранной при сборке модели.
 * Возвращает 1 при успехе; при ошибке выходные данные не изменяются.
 */
uint8_t BDBG_ProtocolBuildRequest(uint8_t device_address,
                                  BdbgRequest command,
                                  uint8_t output[BDBG_REQUEST_MAX_SIZE],
                                  uint8_t *output_length);

/* Контрольная сумма протокола выбранной модели. */
uint8_t BDBG_ProtocolCalculateChecksum(const uint8_t *data, uint16_t length);

#endif /* BDBG_PROTOCOL_H */
