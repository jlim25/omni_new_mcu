#pragma once

/**
  *****************************************************************************
  * @file    app_config.h
  * @brief   Compile-time feature flags for the OMNI-ROBOT firmware.
  *
  *  Usage
  *  -----
  *  Either uncomment the desired flags below, or supply them as compiler
  *  preprocessor defines (-DENABLE_CLI) in your build configuration.
  *
  *  Recommended setup in STM32CubeIDE:
  *    Debug   build:  add -DENABLE_CLI under Preprocessor symbols
  *    Release build:  leave ENABLE_CLI absent (saves ~2.4 KB RAM)
  *****************************************************************************
  */

/* ── CLI console over DEBUG_UART ──────────────────────────────────────────
 * Enables the UART ↔ FreeRTOS-Plus-CLI bridge task (cliConsoleTask).
 * Costs ~2.4 KB RAM:  512-word stack + 256-byte output buffer + UART buffer.
 * Disable in Release to reclaim that RAM on the 12 KB STM32F303K8.
 * ──────────────────────────────────────────────────────────────────────── */
#define ENABLE_CLI
