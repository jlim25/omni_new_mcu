/**
  *****************************************************************************
  * @file    motor_config.c
  * @brief   Static configuration table for all daisy-chained servo motors.
  *
  *          This is the single place to edit when adding, removing, or
  *          re-assigning a motor joint.  All other firmware modules
  *          (servoMotor.c, can_driver.c, cli_commands.c) iterate over
  *          g_motor_configs[] at runtime.
  *
  *  Servo bus IDs
  *  -------------
  *  Each entry's servo_id must match the ID programmed into the physical
  *  servo.  Entry index i = servo_id - 1 (0-based).  IDs 1–6 are supported.
  *  Use the Hiwonder bus-servo configuration tool to set servo IDs.
  *
  *  Motor specs for slots 5 & 6
  *  ----------------------------
  *  Update .spec to match your actual motor model if you use those joints.
  *
  *  CAN frame mapping
  *  -----------------
  *  RPi_Command_N  (0x20N) → commands to servo ID N (received by this MCU)
  *  MCU_Status_N   (0x10N) → telemetry from servo ID N (sent by this MCU)
  *
  *  Note: only servos that respond during the boot/reconfig scan are active.
  *****************************************************************************
  */

#include "motorSelection.h"

/* ── Helper: silence pointer-incompatibility warnings when storing   ──
 *    cantools functions (typed with per-motor structs) into the
 *    generic motor_cmd_unpack_fn / motor_status_pack_fn function
 *    pointers.  All _1.._6 structs are layout-identical, so the casts
 *    are safe.
 * ──────────────────────────────────────────────────────────────────── */
#define CMD_UNPACK_FN(fn)    ((motor_cmd_unpack_fn)(fn))
#define STATUS_PACK_FN(fn)   ((motor_status_pack_fn)(fn))

/* ── Global motor configuration table ───────────────────────────── */
const motor_config_t g_motor_configs[MOTOR_MAX] = {
    /* ── Joint 1 ── Base motor, HTD-85H ─────────────────────────── */
    {
        .servo_id       = 1,
        .spec           = (hiwonder_servo_spec_t)HTD_85H_SPEC,
        .name           = "Joint-1 Base (240 deg)",
        .can_cmd_id     = OMNI_ROBOT_R_PI_COMMAND_1_FRAME_ID,
        .can_status_id  = OMNI_ROBOT_MCU_STATUS_1_FRAME_ID,
        .can_status_len = OMNI_ROBOT_MCU_STATUS_1_LENGTH,
        .cmd_unpack     = CMD_UNPACK_FN(omni_robot_r_pi_command_1_unpack),
        .status_pack    = STATUS_PACK_FN(omni_robot_mcu_status_1_pack),
    },
    /* ── Joint 2 ── Base motor, HTD-85H ─────────────────────────── */
    {
        .servo_id       = 2,
        .spec           = (hiwonder_servo_spec_t)HTD_85H_SPEC,
        .name           = "Joint-2 Base (240 deg)",
        .can_cmd_id     = OMNI_ROBOT_R_PI_COMMAND_2_FRAME_ID,
        .can_status_id  = OMNI_ROBOT_MCU_STATUS_2_FRAME_ID,
        .can_status_len = OMNI_ROBOT_MCU_STATUS_2_LENGTH,
        .cmd_unpack     = CMD_UNPACK_FN(omni_robot_r_pi_command_2_unpack),
        .status_pack    = STATUS_PACK_FN(omni_robot_mcu_status_2_pack),
    },
    /* ── Joint 3 ── Arm motor, HTD-45H ──────────────────────────── */
    {
        .servo_id       = 3,
        .spec           = (hiwonder_servo_spec_t)HTD_45H_SPEC,
        .name           = "Joint-3 Arm (240 deg)",
        .can_cmd_id     = OMNI_ROBOT_R_PI_COMMAND_3_FRAME_ID,
        .can_status_id  = OMNI_ROBOT_MCU_STATUS_3_FRAME_ID,
        .can_status_len = OMNI_ROBOT_MCU_STATUS_3_LENGTH,
        .cmd_unpack     = CMD_UNPACK_FN(omni_robot_r_pi_command_3_unpack),
        .status_pack    = STATUS_PACK_FN(omni_robot_mcu_status_3_pack),
    },
    /* ── Joint 4 ── Wrist motor, HTD-35H ────────────────────────── */
    {
        .servo_id       = 4,
        .spec           = (hiwonder_servo_spec_t)HTD_35H_SPEC,
        .name           = "Joint-4 Wrist (240 deg)",
        .can_cmd_id     = OMNI_ROBOT_R_PI_COMMAND_4_FRAME_ID,
        .can_status_id  = OMNI_ROBOT_MCU_STATUS_4_FRAME_ID,
        .can_status_len = OMNI_ROBOT_MCU_STATUS_4_LENGTH,
        .cmd_unpack     = CMD_UNPACK_FN(omni_robot_r_pi_command_4_unpack),
        .status_pack    = STATUS_PACK_FN(omni_robot_mcu_status_4_pack),
    },
    /* ── Joint 5 ── update .spec to match your motor model ──────── */
    {
        .servo_id       = 5,
        .spec           = (hiwonder_servo_spec_t)HTD_35H_SPEC,
        .name           = "Joint-5 (240 deg)",
        .can_cmd_id     = OMNI_ROBOT_R_PI_COMMAND_5_FRAME_ID,
        .can_status_id  = OMNI_ROBOT_MCU_STATUS_5_FRAME_ID,
        .can_status_len = OMNI_ROBOT_MCU_STATUS_5_LENGTH,
        .cmd_unpack     = CMD_UNPACK_FN(omni_robot_r_pi_command_5_unpack),
        .status_pack    = STATUS_PACK_FN(omni_robot_mcu_status_5_pack),
    },
    /* ── Joint 6 ── update .spec to match your motor model ──────── */
    {
        .servo_id       = 6,
        .spec           = (hiwonder_servo_spec_t)HTD_35H_SPEC,
        .name           = "Joint-6 (240 deg)",
        .can_cmd_id     = OMNI_ROBOT_R_PI_COMMAND_6_FRAME_ID,
        .can_status_id  = OMNI_ROBOT_MCU_STATUS_6_FRAME_ID,
        .can_status_len = OMNI_ROBOT_MCU_STATUS_6_LENGTH,
        .cmd_unpack     = CMD_UNPACK_FN(omni_robot_r_pi_command_6_unpack),
        .status_pack    = STATUS_PACK_FN(omni_robot_mcu_status_6_pack),
    },
};
