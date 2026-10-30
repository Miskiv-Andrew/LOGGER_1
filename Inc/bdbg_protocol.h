#ifndef BDBG_PROTOCOL_H
#define BDBG_PROTOCOL_H
#include <stdint.h>
#include "logger_types.h"
#define BDBG_REQUEST_MAX_SIZE 9U
#define BDBG_SPECTRUM_CHANNELS 1024U
#define BDBG_SPECTRUM_RESPONSE_SIZE 2076U
uint8_t BDBG_ProtocolBuildRequest(uint8_t address, BdbgRequest command,
                                  uint8_t output[BDBG_REQUEST_MAX_SIZE], uint8_t *length);
uint8_t BDBG_ProtocolCalculateChecksum(const uint8_t *data, uint16_t length);
#endif
