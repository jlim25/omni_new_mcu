#include "app_config.h"
#ifdef ENABLE_CLI

/**
  *****************************************************************************
  * @file    cli_console.c
  * @brief   CLI console task – bridges DEBUG_UART ↔ FreeRTOS-Plus-CLI.
  *
  *  How it works
  *  ------------
  *  1. Characters are received one at a time from DEBUG_UART (blocking with
  *     a short timeout so the RTOS can still schedule other tasks).
  *  2. They are appended to a local line buffer, echoed back so the user
  *     can see what they're typing, and backspace is handled.
  *  3. On '\r' (Enter) the assembled string is handed to
  *     FreeRTOS_CLIProcessCommand(), which may call the callback multiple
  *     times (pdTRUE) before signalling it is finished (pdFALSE).
  *  4. The shared cOutputBuffer (owned by FreeRTOS_CLI.c) is used for
  *     all command output — no extra RAM needed here.
  *****************************************************************************
  */

#include "cli_commands.h"
#include "FreeRTOS_CLI.h"
#include "debug.h"
#include "string.h"
#include "logger.h"

/* Maximum input line length (bytes including null terminator) */
#define CLI_INPUT_BUF_LEN   64

/* Prompt printed after each completed command */
#define CLI_PROMPT          "\r\n> "

/* Convenience wrapper: transmit a C-string over DEBUG_UART */
static inline void uart_puts(const char *s)
{
    HAL_UART_Transmit(DEBUG_UART, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

void cliConsoleTask(void const *argument)
{
    (void)argument;

    char pcInputBuf[CLI_INPUT_BUF_LEN];
    uint16_t usInputLen = 0;
    uint8_t ucRxChar    = 0;

    LOG_DEBUG("CLI console task started\r\n");
    /* Register all application commands before we start accepting input */
    CLI_RegisterAllCommands();

    uart_puts("\r\n=== FreeRTOS CLI ===\r\nType 'help' for a list of commands.");
    uart_puts(CLI_PROMPT);

    while (1)
    {
        /* Receive one character; use a short timeout so we don't starve
         * other tasks if the UART is idle.                                */
        if (HAL_UART_Receive(DEBUG_UART, &ucRxChar, 1, 10) != HAL_OK) {
            continue;
        }

        if (ucRxChar == '\r' || ucRxChar == '\n')
        {
            /* Echo newline and only process non-empty lines */
            uart_puts("\r\n");

            if (usInputLen == 0) {
                uart_puts(CLI_PROMPT);
                continue;
            }

            /* Null-terminate and process the line */
            pcInputBuf[usInputLen] = '\0';

            char *pcOutputBuf = FreeRTOS_CLIGetOutputBuffer();
            BaseType_t xMore;

            do {
                xMore = FreeRTOS_CLIProcessCommand(pcInputBuf,
                                                   pcOutputBuf,
                                                   configCOMMAND_INT_MAX_OUTPUT_SIZE);
                uart_puts(pcOutputBuf);
            } while (xMore == pdTRUE);

            /* Reset input buffer */
            memset(pcInputBuf, 0, sizeof(pcInputBuf));
            usInputLen = 0;

            uart_puts(CLI_PROMPT);
        }
        else if (ucRxChar == '\b' || ucRxChar == 127) /* backspace / DEL */
        {
            if (usInputLen > 0) {
                usInputLen--;
                pcInputBuf[usInputLen] = '\0';
                uart_puts("\b \b"); /* erase character on terminal */
            }
        }
        else if (usInputLen < (CLI_INPUT_BUF_LEN - 1))
        {
            /* Printable character – append and echo */
            pcInputBuf[usInputLen++] = (char)ucRxChar;
            HAL_UART_Transmit(DEBUG_UART, &ucRxChar, 1, 10); /* echo */
        }
        /* Characters beyond the buffer length are silently dropped */
    }
}

#endif /* ENABLE_CLI */
