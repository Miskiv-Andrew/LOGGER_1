#ifndef BDBG_CONFIG_H
#define BDBG_CONFIG_H

/*
 * ЕДИНАЯ ТОЧКА ВЫБОРА МОДЕЛИ БДБГ.
 * При добавлении модели определите её ID и создайте отдельный протокол.
 * Для неизвестной модели сборка намеренно завершается ошибкой.
 */
#define BDBG_MODEL_15              15U
#define BDBG_MODEL                 BDBG_MODEL_15

#if BDBG_MODEL == BDBG_MODEL_15

/* Настройки UART4, соответствующие БДБГ-15 (19200, 8N1). */
#define BDBG_UART_BAUDRATE         19200U
#define BDBG_UART_WORD_LENGTH      UART_WORDLENGTH_8B
#define BDBG_UART_STOP_BITS        UART_STOPBITS_1
#define BDBG_UART_PARITY           UART_PARITY_NONE

/* Адрес основного блока и максимальное ожидание ответа. */
#define BDBG_DEFAULT_ADDRESS       0x01U
#define BDBG_RESPONSE_TIMEOUT_MS   2000U

#else
#error "BDBG_MODEL: protocol and UART configuration for this model are not implemented"
#endif

#endif /* BDBG_CONFIG_H */
