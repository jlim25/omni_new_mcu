#pragma once

/**
  *****************************************************************************
  * @file    can_driver.h
  * @brief   CAN driver: RX/TX tasks and shared application data.
  *
  *  Architecture
  *  ------------
  *  canRxTask  – woken by ISR via direct task notification; unpacks frames
  *               and updates g_can_rx[i] under a mutex for the matching motor.
  *  canTxTask  – blocks on canTxQueue; packs and sends frames via HAL.
  *
  *  The ISR callback (HAL_CAN_RxFifo0MsgPendingCallback) is implemented
  *  in can_driver.c and notifies canRxTask directly.
  *
  *  With the daisy-chain architecture, one STM32 controls all MOTOR_MAX
  *  servo slots.  One can_rx_state_t entry exists per slot (0..MOTOR_MAX-1).
  *  The CAN hardware filter passes 0x200-0x3FF
  *  (RPi_Command_1..6, RPi_Reconfig, RPi_FaultClear).
  *****************************************************************************
  */

#include "debug.h"
#include "motorSelection.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── TX queue depth ─────────────────────────────────────────────── */
#define CAN_TX_QUEUE_DEPTH   8

/* ── Raw CAN frame container used for the TX queue ─────────────── */
typedef struct {
    uint32_t id;          // standard 11-bit CAN ID
    uint8_t  dlc;         // data length code (0–8)
    uint8_t  data[8];
} can_raw_frame_t;

/* ── Shared RX state (motor commands) ──────────────────────────── */
/**
 * One entry per motor slot (indexed 0..MOTOR_MAX-1).
 * Written by canRxTask, read by servoMotorTask.
 * Always access under can_rx_mutex.
 */
typedef struct {
    motor_cmd_t cmd;    // latest decoded RPi_Command for this slot
    bool        fresh;  // true until consumed by servoMotorTask
} can_rx_state_t;

/* ── Shared reconfig state ──────────────────────────────────────── */
/**
 * Populated by canRxTask when an RPi_Reconfig (0x300) frame arrives.
 * servoMotorTask polls .pending and triggers a full rescan when set.
 * Always access under can_rx_mutex.
 */
typedef struct {
    uint8_t servo_ids[MOTOR_MAX];  /**< Servo bus IDs to activate (0 = empty) */
    bool    pending;               /**< true = rescan requested, not yet handled */
} can_reconfig_state_t;

extern can_rx_state_t       g_can_rx[MOTOR_MAX];
extern can_reconfig_state_t g_can_reconfig;
extern SemaphoreHandle_t    can_rx_mutex;
extern QueueHandle_t        canTxQueue;

/* ── Shared fault-clear state ──────────────────────────────────────── */
/**
 * Populated by canRxTask when an RPi_FaultClear (0x301) frame arrives.
 * servoMotorTask polls .pending and clears the indicated motor(s).
 * Always access under can_rx_mutex.
 */
typedef struct {
    uint8_t motor_id;  /**< 1-6 = specific motor, 0 = clear all */
    bool    pending;
} can_fault_clear_state_t;

extern can_fault_clear_state_t g_can_fault_clear;

/* ── Helper: enqueue a packed MCU_Status frame for TX ───────────── */
/**
 * Call from any task to publish a status update for a single motor.
 *
 * @param motor_idx  Index into g_motor_configs[] (0..MOTOR_MAX-1).
 * @param status     Pointer to the packed status struct.
 *
 * Non-blocking: drops the message silently if the TX queue is full.
 */
void CAN_SendMcuStatus(uint8_t motor_idx, const motor_status_t *status);

/**
 * Enqueue an MCU_Fault frame (0x10F) on the TX queue.
 *
 * @param motor_id    Servo bus ID of the faulting motor (1..MOTOR_MAX).
 * @param fault_code  FAULT_CODE_* constant from motorSelection.h.
 * @param fault_value Measured value that triggered the fault (mV or degC).
 *
 * Non-blocking: drops the message silently if the TX queue is full.
 */
void CAN_SendMcuFault(uint8_t motor_id, uint8_t fault_code, uint16_t fault_value);

#ifdef __cplusplus
}
#endif
