#include "bdbg.h"
#include "bdbg_protocol.h"
#include <stddef.h>

/*
 * Общий API БДБГ. Здесь нет кодов команд и контрольных сумм конкретной
 * модели: эти детали полностью принадлежат выбранному модулю протокола.
 * На следующем этапе сюда добавится управление UART4 и DMA.
 */
uint8_t BDBG_CalculateCrc(const uint8_t *data, uint16_t length)
{
    return BDBG_ProtocolCalculateChecksum(data, length);
}

uint8_t BDBG_BuildRequest(const BdbgReq *request,
                          uint8_t output[BDBG_REQUEST_MAX_SIZE],
                          uint8_t *outputLength)
{
    if (request == NULL)
    {
        return 0U;
    }

    return BDBG_ProtocolBuildRequest(request->deviceAddr,
                                     request->bdbgMode,
                                     output,
                                     outputLength);
}
