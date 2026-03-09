#pragma once

/**
  *****************************************************************************
  * @file    motorSelection.h
  * @brief   Motor configuration table for the single-MCU daisy-chain node.
  *
  *          MOTOR_MAX is the compile-time ceiling (max possible motors = 6).
  *          The actual number of motors present is discovered at runtime via
  *          a servo-bus ping scan and stored in g_active_motor_count.
  *
  *  To add or reconfigure motors:
  *    • Edit g_motor_configs[] in motor_config.c for permanent changes.
  *    • Send an RPi_Reconfig CAN frame (0x300) for live reconfiguration.
  *
  *  Servo ID → Joint mapping
  *  ------------------------
  *  Servo bus ID 1 ↔ Joint 1 ↔ RPi_Command_1 / MCU_Status_1
  *  Servo bus ID 2 ↔ Joint 2 ↔ RPi_Command_2 / MCU_Status_2   ... etc.
  *  (IDs must be pre-programmed into the physical servos.)
  *****************************************************************************
  */

#include <stdbool.h>
#include "hiwonder_bus_servo.h"
#include "omni_robot.h"  // CAN frame ID / length constants

/* ── Compile-time motor ceiling ──────────────────────────────────── */
#define MOTOR_MAX   6   /**< Maximum number of servos on the bus */

/* ── CAN message for live reconfiguration ────────────────────────── */
#define MOTOR_RECONFIG_CAN_ID   OMNI_ROBOT_R_PI_RECONFIG_FRAME_ID  /* 0x300 */

/* ── Canonical CAN payload types ──────────────────────────────────
 *
 *  All six RPi_Command structs are layout-identical (same fields, same
 *  sizes).  Likewise for the six MCU_Status structs.  We use the _1
 *  variants as the canonical C type and cast where necessary.
 * ─────────────────────────────────────────────────────────────────── */
typedef struct omni_robot_r_pi_command_1_t  motor_cmd_t;
typedef struct omni_robot_mcu_status_1_t    motor_status_t;

/* ── Function-pointer types for CAN pack / unpack ─────────────────── */
typedef int (*motor_cmd_unpack_fn)(motor_cmd_t       *dst,
                                   const uint8_t     *data,
                                   size_t             size);

typedef int (*motor_status_pack_fn)(uint8_t              *dst,
                                    const motor_status_t *src,
                                    size_t                size);

/* ── Per-motor compile-time descriptor ────────────────────────────── */
typedef struct {
    uint8_t                servo_id;        // unique servo bus ID (1–6)
    hiwonder_servo_spec_t  spec;            // physical angular range
    const char            *name;            // human-readable label for logs

    uint32_t               can_cmd_id;      // CAN std-ID for incoming RPi_Command
    uint32_t               can_status_id;   // CAN std-ID for outgoing MCU_Status
    uint8_t                can_status_len;  // DLC of MCU_Status frame (bytes)

    motor_cmd_unpack_fn    cmd_unpack;      // cantools-generated unpack fn
    motor_status_pack_fn   status_pack;     // cantools-generated pack fn
} motor_config_t;

/* ── Global config table (defined in motor_config.c) ─────────────── */
extern const motor_config_t g_motor_configs[MOTOR_MAX];

/* ── Runtime motor presence state (defined in servoMotor.c) ──────── */
/**
 * g_servo_present[i] is true iff the servo at g_motor_configs[i].servo_id
 * responded during the last scan.  Index i is 0-based (servo ID − 1).
 *
 * g_active_motor_count is the total number of responding servos.
 *
 * Both are written exclusively by the servoMotorTask and may be read
 * by other tasks for status purposes (no mutex needed for single-byte reads).
 */
extern volatile uint8_t g_active_motor_count;
extern volatile bool    g_servo_present[MOTOR_MAX];

/* ── Safety thresholds ───────────────────────────────────────────────── */
#define MOTOR_VOLT_WARN_LOW_MV   6500u  /**< < this threshold → FAULT_CODE_VOLT_LOW  */
#define MOTOR_VOLT_WARN_HIGH_MV  13000u  /**< > this threshold → FAULT_CODE_VOLT_HIGH */
#define MOTOR_TEMP_WARN_C          65u  /**< > this threshold → FAULT_CODE_TEMP_WARN */
#define MOTOR_TEMP_FAULT_C         75u  /**< > this threshold → FAULT_CODE_TEMP_FAULT, torque disabled */

/* ── Fault codes (match MCU_Fault.FaultCode signal in DBC) ──────────── */
#define FAULT_CODE_NONE        0u
#define FAULT_CODE_VOLT_LOW    1u
#define FAULT_CODE_VOLT_HIGH   2u
#define FAULT_CODE_TEMP_WARN   3u
#define FAULT_CODE_TEMP_FAULT  4u

/* ── CAN IDs for fault reporting and clearing ────────────────────────── */
#define MOTOR_FAULT_CAN_ID         OMNI_ROBOT_MCU_FAULT_FRAME_ID         /* 0x10F */
#define MOTOR_FAULT_CLEAR_CAN_ID   OMNI_ROBOT_R_PI_FAULT_CLEAR_FRAME_ID /* 0x301 */

/* ── Per-motor runtime state ─────────────────────────────────────────── */
typedef enum {
    MOTOR_STATE_ABSENT = 0,  /**< Not found during scan / never pinged */
    MOTOR_STATE_ACTIVE,      /**< Present, torque enabled, accepting commands */
    MOTOR_STATE_FAULT,       /**< Thermal/voltage fault; torque disabled */
} motor_state_t;

/** Written exclusively by servoMotorTask; may be read by other tasks. */
extern volatile motor_state_t g_motor_state[MOTOR_MAX];

/* ── Task-level state machine states ─────────────────────────────────── */
typedef enum {
    TASK_STATE_INIT          = 0,
    TASK_STATE_SCANNING,      /**< motor_scan() in progress */
    TASK_STATE_RUNNING,       /**< Normal operation */
    TASK_STATE_RECONFIGURING, /**< Reconfig scan in progress */
} servo_task_state_t;
