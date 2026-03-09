#include "app_config.h"
#ifdef ENABLE_CLI

/**
  *****************************************************************************
  * @file    cli_commands.c
  * @brief   Application CLI command definitions for FreeRTOS-Plus-CLI.
  *
  *          All commands take a motor ID (1..MOTOR_MAX) as their first
  *          argument, since one MCU now controls all daisy-chained servos.
  *
  *  Adding a new command
  *  --------------------
  *  1. Write a static callback:
  *       static BaseType_t prvMyCmd(char *pcWriteBuffer,
  *                                  size_t xWriteBufferLen,
  *                                  const char *pcCommandString);
  *
  *  2. Declare a static CLI_Command_Definition_t:
  *       static const CLI_Command_Definition_t xMyCmd = {
  *           "mycommand",            // what the user types
  *           "\r\nmycommand <id> <a>:\r\n"
  *           "  Does something on motor <id> with <a>.\r\n\r\n",
  *           prvMyCmd,
  *           2                       // number of parameters
  *       };
  *
  *  3. Call FreeRTOS_CLIRegisterCommand(&xMyCmd) inside
  *     CLI_RegisterAllCommands().
  *****************************************************************************
  */

#include "FreeRTOS_CLI.h"
#include "cli_commands.h"
#include "motorSelection.h"
#include "debug.h"
#include "string.h"
#include "stdlib.h"

/* Stringify helper for MOTOR_MAX in command help strings */
#define STRINGIFY_IMPL(x)  #x
#define STRINGIFY(x)       STRINGIFY_IMPL(x)

/* Forward-declare callbacks */
static BaseType_t prvMoveAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                      const char *pcCommandString);
static BaseType_t prvReadPosCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                    const char *pcCommandString);
static BaseType_t prvTorqueCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                   const char *pcCommandString);
static BaseType_t prvStopCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                 const char *pcCommandString);
static BaseType_t prvSetServoIdCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                       const char *pcCommandString);

/* ── Command definitions ────────────────────────────────────────── */

static const CLI_Command_Definition_t xMoveAngleCommand = {
    "move",
    "\r\nmove <id> <degrees> <time_ms>:\r\n"
    "  Move servo <id> (1.." STRINGIFY(MOTOR_MAX) ") to <degrees> over\r\n"
    "  <time_ms> milliseconds.\r\n"
    "  Example: move 1 120.0 1000\r\n\r\n",
    prvMoveAngleCommand,
    3   /* id, degrees, time_ms */
};

static const CLI_Command_Definition_t xReadPosCommand = {
    "readpos",
    "\r\nreadpos <id>:\r\n"
    "  Read the current position of servo <id> (1.." STRINGIFY(MOTOR_MAX) ").\r\n\r\n",
    prvReadPosCommand,
    1   /* id */
};

static const CLI_Command_Definition_t xTorqueCommand = {
    "torque",
    "\r\ntorque <id> <on|off>:\r\n"
    "  Enable or disable torque on servo <id> (1.." STRINGIFY(MOTOR_MAX) ").\r\n"
    "  Example: torque 2 on\r\n\r\n",
    prvTorqueCommand,
    2   /* id, on/off */
};

static const CLI_Command_Definition_t xStopCommand = {
    "stop",
    "\r\nstop <id>:\r\n"
    "  Immediately halt servo <id> (1.." STRINGIFY(MOTOR_MAX) ") at its\r\n"
    "  current position.\r\n\r\n",
    prvStopCommand,
    1   /* id */
};

static const CLI_Command_Definition_t xSetServoIdCommand = {
    "servosetid",
    "\r\nservosetid <id> <new_servo_id>:\r\n"
    "  Program a new bus ID into the servo at motor slot <id> (1.." STRINGIFY(MOTOR_MAX) ").\r\n"
    "  <new_servo_id> is the new physical servo bus ID (1..253).\r\n"
    "  Verifies the write by reading position with the new ID.\r\n"
    "  NOTE: reboot or re-scan required for the system to use the new ID.\r\n\r\n",
    prvSetServoIdCommand,
    2   /* id, new_servo_id */
};

/* ── Registration ───────────────────────────────────────────────── */

void CLI_RegisterAllCommands(void)
{
    FreeRTOS_CLIRegisterCommand(&xMoveAngleCommand);
    FreeRTOS_CLIRegisterCommand(&xReadPosCommand);
    FreeRTOS_CLIRegisterCommand(&xTorqueCommand);
    FreeRTOS_CLIRegisterCommand(&xStopCommand);
    FreeRTOS_CLIRegisterCommand(&xSetServoIdCommand);
}

/* ── Servo handle array (defined in servoMotor.c) ───────────────── */
extern hiwonder_servo_t servo[MOTOR_MAX];

/* ── Helper: parse motor ID argument (1-based) → 0-based index ──── */
/**
 * Parses pcArg as a motor ID (1..MOTOR_MAX) and writes the 0-based
 * array index to *idx_out.
 *
 * @return true on success, false if out of range.
 */
static bool prv_parse_motor_id(const char *pcArg, BaseType_t len,
                                uint8_t *idx_out)
{
    (void)len;
    long id = strtol(pcArg, NULL, 10);
    if (id < 1 || id > (long)MOTOR_MAX) {
        return false;
    }
    *idx_out = (uint8_t)(id - 1u);
    return true;
}

/* ── Callbacks ──────────────────────────────────────────────────── */
static BaseType_t prvMoveAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                      const char *pcCommandString)
{
    BaseType_t xLen1 = 0, xLen2 = 0, xLen3 = 0;

    const char *pcId   = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLen1);
    const char *pcDeg  = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xLen2);
    const char *pcTime = FreeRTOS_CLIGetParameter(pcCommandString, 3, &xLen3);

    if (pcId == NULL || pcDeg == NULL || pcTime == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: move <id> <degrees> <time_ms>\r\n");
        return pdFALSE;
    }

    uint8_t idx;
    if (!prv_parse_motor_id(pcId, xLen1, &idx)) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid motor ID. Use 1..%u\r\n", (unsigned)MOTOR_MAX);
        return pdFALSE;
    }

    float    deg  = strtof(pcDeg,  NULL);
    uint16_t t_ms = (uint16_t)strtoul(pcTime, NULL, 10);

    hwservo_status_t st = HWSERVO_MoveToAngle(&servo[idx], deg, t_ms);

    if (st == HWSERVO_OK) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: moving to %.1f deg over %u ms\r\n",
                 g_motor_configs[idx].name, deg, t_ms);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: move failed (err %d)\r\n",
                 g_motor_configs[idx].name, (int)st);
    }
    return pdFALSE;
}

static BaseType_t prvReadPosCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                    const char *pcCommandString)
{
    BaseType_t xLen1 = 0;
    const char *pcId = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLen1);

    if (pcId == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: readpos <id>\r\n");
        return pdFALSE;
    }

    uint8_t idx;
    if (!prv_parse_motor_id(pcId, xLen1, &idx)) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid motor ID. Use 1..%u\r\n", (unsigned)MOTOR_MAX);
        return pdFALSE;
    }

    float   deg = 0.0f;
    int16_t raw = 0;

    hwservo_status_t st = HWSERVO_ReadAngle_deg(&servo[idx], &deg);
    if (st == HWSERVO_OK) {
        HWSERVO_ReadPos_Raw(&servo[idx], &raw);
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: %.1f deg (raw=%d)\r\n",
                 g_motor_configs[idx].name, deg, raw);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: read failed (err %d)\r\n",
                 g_motor_configs[idx].name, (int)st);
    }
    return pdFALSE;
}

static BaseType_t prvTorqueCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                   const char *pcCommandString)
{
    BaseType_t xLen1 = 0, xLen2 = 0;
    const char *pcId  = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLen1);
    const char *pcArg = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xLen2);

    if (pcId == NULL || pcArg == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: torque <id> <on|off>\r\n");
        return pdFALSE;
    }

    uint8_t idx;
    if (!prv_parse_motor_id(pcId, xLen1, &idx)) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid motor ID. Use 1..%u\r\n", (unsigned)MOTOR_MAX);
        return pdFALSE;
    }

    bool enable;
    if (strncmp(pcArg, "on", (size_t)xLen2) == 0) {
        enable = true;
    } else if (strncmp(pcArg, "off", (size_t)xLen2) == 0) {
        enable = false;
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid argument '%.*s'. Use 'on' or 'off'.\r\n",
                 (int)xLen2, pcArg);
        return pdFALSE;
    }

    hwservo_status_t st = HWSERVO_EnableTorque(&servo[idx], enable);
    snprintf(pcWriteBuffer, xWriteBufferLen,
             "%s: torque %s%s\r\n",
             g_motor_configs[idx].name,
             enable ? "enabled" : "disabled",
             st == HWSERVO_OK ? "" : " (FAILED)");
    return pdFALSE;
}

static BaseType_t prvStopCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                 const char *pcCommandString)
{
    BaseType_t xLen1 = 0;
    const char *pcId = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLen1);

    if (pcId == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen, "Usage: stop <id>\r\n");
        return pdFALSE;
    }

    uint8_t idx;
    if (!prv_parse_motor_id(pcId, xLen1, &idx)) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid motor ID. Use 1..%u\r\n", (unsigned)MOTOR_MAX);
        return pdFALSE;
    }

    hwservo_status_t st = HWSERVO_MoveStop(&servo[idx]);

    if (st == HWSERVO_OK) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: stopped\r\n",
                 g_motor_configs[idx].name);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: stop command failed (err %d)\r\n",
                 g_motor_configs[idx].name, (int)st);
    }
    return pdFALSE;
}

static BaseType_t prvSetServoIdCommand(char *pcWriteBuffer, size_t xWriteBufferLen,
                                       const char *pcCommandString)
{
    BaseType_t xLen1 = 0, xLen2 = 0;
    const char *pcId    = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xLen1);
    const char *pcNewId = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xLen2);

    if (pcId == NULL || pcNewId == NULL) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Usage: servosetid <id> <new_servo_id>\r\n");
        return pdFALSE;
    }

    /* Resolve motor slot index (1-based → 0-based) */
    uint8_t idx;
    if (!prv_parse_motor_id(pcId, xLen1, &idx)) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid motor ID. Use 1..%u\r\n", (unsigned)MOTOR_MAX);
        return pdFALSE;
    }

    /* Parse new servo bus ID */
    long new_id_l = strtol(pcNewId, NULL, 10);
    if (new_id_l < 1 || new_id_l > 253) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "Invalid new_servo_id %ld. Must be 1..253.\r\n", new_id_l);
        return pdFALSE;
    }
    uint8_t new_servo_id = (uint8_t)new_id_l;

    /* Write new ID to the servo's non-volatile memory */
    hwservo_status_t st = HWSERVO_WriteID(&servo[idx], new_servo_id);
    if (st != HWSERVO_OK) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: ID write failed (err %d)\r\n",
                 g_motor_configs[idx].name, (int)st);
        return pdFALSE;
    }

    /* Verify: build a temporary handle with the new ID and read raw position */
    hiwonder_servo_t tmp = servo[idx];   /* copy bus wiring, mutex, spec */
    tmp.id = new_servo_id;

    int16_t raw = 0;
    st = HWSERVO_ReadPos_Raw(&tmp, &raw);
    if (st == HWSERVO_OK) {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: ID %u → %u OK (verify: raw pos=%d).\r\n"
                 "Reboot or re-scan required to use the new ID.\r\n",
                 g_motor_configs[idx].name,
                 (unsigned)servo[idx].id,
                 (unsigned)new_servo_id,
                 (int)raw);
    } else {
        snprintf(pcWriteBuffer, xWriteBufferLen,
                 "%s: ID write sent but verification FAILED (err %d).\r\n"
                 "The servo may not have accepted the new ID.\r\n",
                 g_motor_configs[idx].name, (int)st);
    }
    return pdFALSE;
}

#endif /* ENABLE_CLI */
