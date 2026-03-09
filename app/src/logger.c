/**
  *****************************************************************************
  * @file    logger.c
  * @author  Jacky Lim
  * @brief   Logger implementation for debugging and testing purposes.
  ******************************************************************************
  */
#include <string.h>

#include "debug.h"
#include "logger.h"
#include "stream_buffer.h"

/* ── Config ────────────────────────────────────────────────────── */
#define LOG_STREAM_BUFFER_SIZE  512   // bytes in the stream buffer
#define LOG_TRIGGER_LEVEL       1     // wake task after this many bytes
#define LOG_TX_BUF_SIZE         256   // scratch buffer for one transmission
#define LOG_TASK_PERIOD_MS      10    // how often the task polls (ms)

#define UART_TIMEOUT_MS         100  // Timeout for UART transmission (ms)

/* ── Private variables ─────────────────────────────────────────── */
static StreamBufferHandle_t s_logStream = NULL;

/* ── Logger task ───────────────────────────────────────────────── */
void loggerTask(void const *argument)
{
    static uint8_t txBuf[LOG_TX_BUF_SIZE];

    if (s_logStream == NULL)
    {
        /* Should never happen, but just in case... */
        configASSERT(0);
        vTaskDelete(NULL);
    }

    for (;;)
    {
        size_t bytes = xStreamBufferReceive(
            s_logStream,
            txBuf,
            sizeof(txBuf),
            pdMS_TO_TICKS(LOG_TASK_PERIOD_MS)
        );

        if (bytes > 0)
        {
            HAL_UART_Transmit(DEBUG_UART, txBuf, bytes, UART_TIMEOUT_MS);
        }
    }
}

/* ── Public API ────────────────────────────────────────────────── */

void logger_init(void)
{
    s_logStream = xStreamBufferCreate(LOG_STREAM_BUFFER_SIZE, LOG_TRIGGER_LEVEL);
    configASSERT(s_logStream != NULL);
}

void logger_log(log_level_t level, const char *fmt, ...)
{
    if (s_logStream == NULL) return;

    static const char *levelStr[] = {
        ANSI_CYAN   "[DEBUG]" ANSI_RESET,
        ANSI_GREEN  "[INFO] " ANSI_RESET,
        ANSI_YELLOW "[WARNING] " ANSI_RESET,
        ANSI_RED    "[ERROR]" ANSI_RESET,
    };

    char buf[LOG_TX_BUF_SIZE];
    va_list args;
    va_start(args, fmt);
    int len = snprintf(buf, sizeof(buf), "%s ", levelStr[level]);
    len    += vsnprintf(buf + len, sizeof(buf) - len, fmt, args);
    va_end(args);

    /* Clamp in case of overflow */
    if (len >= (int)sizeof(buf))
        len = sizeof(buf) - 1;

    /* Send to stream — safe to call from any task */
    xStreamBufferSend(s_logStream, buf, len, pdMS_TO_TICKS(10));
}