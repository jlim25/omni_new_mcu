#pragma once

#include <stdarg.h>
#include <stdint.h>

/* ── ANSI color codes ──────────────────────────────────────────── */
#define ANSI_RESET   "\033[0m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_RED     "\033[31m"

typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR
} log_level_t;


void logger_init(void);
void logger_log(log_level_t level, const char *fmt, ...);
void loggerTask(void const *argument);

#define LOG_DEBUG(fmt, ...)    logger_log(LOG_LEVEL_DEBUG,   fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)     logger_log(LOG_LEVEL_INFO,    fmt, ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...)  logger_log(LOG_LEVEL_WARNING, fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)    logger_log(LOG_LEVEL_ERROR,   fmt, ##__VA_ARGS__)
