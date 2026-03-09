/**
  *****************************************************************************
  * @file    servoMotor.c
  * @author  Jacky Lim
  * @brief   Servo motor task – single MCU, daisy-chain bus.
  *
  *          Up to MOTOR_MAX servos share one half-duplex UART bus.  Each
  *          servo is distinguished by its unique servo bus ID (1..MOTOR_MAX).
  *
  *          State machine (task-level)
  *          ──────────────────────────
  *          INIT → SCANNING → RUNNING ⇄ RECONFIGURING
  *
  *          Per-motor states: ABSENT | ACTIVE | FAULT
  *          A motor enters FAULT when:
  *            - temperature > MOTOR_TEMP_FAULT_C, OR
  *            - voltage < MOTOR_VOLT_WARN_LOW_MV / > MOTOR_VOLT_WARN_HIGH_MV
  *          Faulted motors have torque disabled and reject CAN commands until
  *          the RPi sends RPi_FaultClear (0x301).
  *
  *          Boot behaviour
  *          ──────────────
  *          1. Init all MOTOR_MAX servo handles (no bus traffic).
  *          2. Ping every ID 1..MOTOR_MAX; mark responding ones ACTIVE.
  *          3. Enable torque only on ACTIVE servos.
  *
  *          Runtime reconfiguration
  *          ───────────────────────
  *          RPi sends RPi_Reconfig (0x300) – canRxTask stores IDs in
  *          g_can_reconfig.  servoMotorTask calls motor_scan() with new list.
  *
  *          Health monitoring
  *          ─────────────────
  *          Inline, round-robin: one motor checked every HEALTH_CHECK_INTERVAL_MS.
  *          Reads voltage and temperature.  On threshold breach: disables torque,
  *          transitions motor to FAULT, sends MCU_Fault CAN frame (0x10F).
  ******************************************************************************
  */

#include "motorSelection.h"
#include "can_driver.h"
#include "hiwonder_bus_servo.h"
#include "debug.h"
#include "logger.h"
#include "bsp.h"

/* ── Timing constants ─────────────────────────────────────────────── */
#define SERVO_STATUS_PERIOD_MS      100u  /**< MCU_Status publish rate */
#define HEALTH_CHECK_INTERVAL_MS    500u  /**< How often one motor is health-checked */

/* ── Runtime state (externs declared in motorSelection.h) ────────── */
volatile uint8_t       g_active_motor_count        = 0u;
volatile bool          g_servo_present[MOTOR_MAX]  = { false };
volatile motor_state_t g_motor_state[MOTOR_MAX]    = { MOTOR_STATE_ABSENT };

/* ── Shared UART bus mutex ────────────────────────────────────────── */
static SemaphoreHandle_t servo_bus_mutex = NULL;

/* ── Servo handle array (placed in CCMRAM to free main RAM) ──────── */
hiwonder_servo_t servo[MOTOR_MAX] __attribute__((section(".ccmram")));

// #define ENABLE_PUBLISH_STATUS
#define ENABLE_MOTOR_TEST   // Temporary: test motor comms without CAN

/* ========================================================================== */
/* motor_scan()                                                                */
/*                                                                             */
/* Disable all non-absent servos, then ping each requested ID.                 */
/* Responding IDs are set to ACTIVE and torque-enabled.                        */
/*                                                                             */
/* @param requested_ids  Array of servo bus IDs to probe (0 = skip).          */
/* @param count          Number of entries in requested_ids[].                 */
/* ========================================================================== */
static void motor_scan(const uint8_t *requested_ids, uint8_t count)
{
    /* ── 1. Disable torque on all non-absent slots ───────────────── */
    for (uint8_t i = 0; i < MOTOR_MAX; i++)
    {
        if (g_motor_state[i] != MOTOR_STATE_ABSENT) {
            HWSERVO_EnableTorque(&servo[i], false);
        }
        g_servo_present[i] = false;
        g_motor_state[i]   = MOTOR_STATE_ABSENT;
    }
    g_active_motor_count = 0u;

    /* ── 2. Probe each requested ID ──────────────────────────────── */
    for (uint8_t k = 0; k < count; k++)
    {
        uint8_t id = requested_ids[k];
        if (id == 0u || id > MOTOR_MAX) {
            continue;
        }

        uint8_t idx = (uint8_t)(id - 1u);

        HWSERVO_Init(&servo[idx],
                     SERVO_UART,
                     SERVO_DIR_GPIO_Port, SERVO_DIR_Pin,
                     true,
                     servo_bus_mutex,
                     id,
                     g_motor_configs[idx].spec);

        if (HWSERVO_Ping(&servo[idx]) == HWSERVO_OK)
        {
            g_servo_present[idx] = true;
            g_motor_state[idx]   = MOTOR_STATE_ACTIVE;
            g_active_motor_count++;
            HWSERVO_EnableTorque(&servo[idx], true);
            LOG_DEBUG("motor_scan: found %s (bus ID=%u, slot=%u)\r\n",
                      g_motor_configs[idx].name, id, idx);
        }
        else
        {
            LOG_WARNING("motor_scan: no response from bus ID=%u (%s)\r\n",
                        id, g_motor_configs[idx].name);
        }
    }

    LOG_DEBUG("motor_scan: %u motor(s) present\r\n", g_active_motor_count);
}

/* ========================================================================== */
/* motor_check_health()                                                        */
/*                                                                             */
/* Read voltage and temperature for one motor slot.                            */
/* On threshold breach: disable torque, mark FAULT, send MCU_Fault CAN frame. */
/* FAULT motors are re-checked each interval (log only, no re-trigger).        */
/* ========================================================================== */
static void motor_check_health(uint8_t idx)
{
    if (g_motor_state[idx] == MOTOR_STATE_ABSENT) {
        return;
    }

    hiwonder_servo_t *s          = &servo[idx];
    bool     fault               = false;
    uint8_t  fault_code          = FAULT_CODE_NONE;
    uint16_t fault_value         = 0u;

    /* ── Voltage ────────────────────────────────────────────────── */
    uint16_t mv = 0u;
    if (HWSERVO_ReadVin_mV(s, &mv) == HWSERVO_OK)
    {
        if (mv < MOTOR_VOLT_WARN_LOW_MV)
        {
            LOG_WARNING("%s: low voltage %u mV\r\n", g_motor_configs[idx].name, mv);
            fault = true;  fault_code = FAULT_CODE_VOLT_LOW;  fault_value = mv;
        }
        else if (mv > MOTOR_VOLT_WARN_HIGH_MV)
        {
            LOG_WARNING("%s: high voltage %u mV\r\n", g_motor_configs[idx].name, mv);
            fault = true;  fault_code = FAULT_CODE_VOLT_HIGH;  fault_value = mv;
        }
    }

    /* ── Temperature (overrides voltage fault code if more severe) ── */
    uint8_t temp = 0u;
    if (HWSERVO_ReadTemp_C(s, &temp) == HWSERVO_OK)
    {
        if (temp > MOTOR_TEMP_FAULT_C)
        {
            LOG_ERROR("%s: critical temp %u degC – disabling torque\r\n",
                      g_motor_configs[idx].name, temp);
            fault = true;  fault_code = FAULT_CODE_TEMP_FAULT;  fault_value = (uint16_t)temp;
        }
        else if (temp > MOTOR_TEMP_WARN_C && !fault)
        {
            LOG_WARNING("%s: high temp %u degC\r\n", g_motor_configs[idx].name, temp);
            fault = true;  fault_code = FAULT_CODE_TEMP_WARN;  fault_value = (uint16_t)temp;
        }
    }

    if (fault && g_motor_state[idx] == MOTOR_STATE_ACTIVE)
    {
        /* First fault detection: disable torque and notify RPi */
        HWSERVO_EnableTorque(s, false);
        g_servo_present[idx] = false;
        g_motor_state[idx]   = MOTOR_STATE_FAULT;
        g_active_motor_count--;

        CAN_SendMcuFault((uint8_t)(idx + 1u), fault_code, fault_value);

        LOG_ERROR("motor[%u] %s → FAULT (code=%u, val=%u)\r\n",
                  idx, g_motor_configs[idx].name, fault_code, fault_value);
    }
}

/* ========================================================================== */

void servoMotorTask(void const *argument)
{
    (void)argument;

    /* ── Create the shared bus mutex ────────────────────────────── */
    servo_bus_mutex = xSemaphoreCreateMutex();
    configASSERT(servo_bus_mutex != NULL);

    /* ── INIT: pre-init all servo handles (no bus traffic) ──────── */
    servo_task_state_t task_state = TASK_STATE_INIT;

    for (uint8_t i = 0; i < MOTOR_MAX; i++)
    {
        HWSERVO_Init(&servo[i],
                     SERVO_UART,
                     SERVO_DIR_GPIO_Port, SERVO_DIR_Pin,
                     true,
                     servo_bus_mutex,
                     g_motor_configs[i].servo_id,
                     g_motor_configs[i].spec);
    }

    /* ── SCANNING: boot scan ──────────────────────────────────────── */
    task_state = TASK_STATE_SCANNING;
    static const uint8_t boot_ids[MOTOR_MAX] = { 1u, 2u, 3u, 4u, 5u, 6u };
    // motor_scan(boot_ids, MOTOR_MAX); //TODO: temporary disable this. Otherwise, program is stuck

    task_state = TASK_STATE_RUNNING;
    LOG_DEBUG("Servo task started: %u motor(s) discovered\r\n", g_active_motor_count);

    /* ── Tick trackers ──────────────────────────────────────────── */
    TickType_t xLastStatusTick = xTaskGetTickCount();
    TickType_t xLastHealthTick = xTaskGetTickCount();
    uint8_t    health_slot     = 0u;

#ifdef ENABLE_MOTOR_TEST
    static float test_deg[MOTOR_MAX];
    for (uint8_t i = 0; i < MOTOR_MAX; i++) { test_deg[i] = 0.0f; }
    uint8_t test_motor = 0u;
#endif

    /* ── RUNNING loop ────────────────────────────────────────────── */
    while (1)
    {
        /* ── 1. Reconfiguration ────────────────────────────────────── */
        if (can_rx_mutex != NULL &&
            xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            bool    do_reconfig  = g_can_reconfig.pending;
            uint8_t reconfig_ids[MOTOR_MAX];
            if (do_reconfig) {
                for (uint8_t i = 0; i < MOTOR_MAX; i++) {
                    reconfig_ids[i] = g_can_reconfig.servo_ids[i];
                }
                g_can_reconfig.pending = false;
            }
            xSemaphoreGive(can_rx_mutex);

            if (do_reconfig) {
                task_state = TASK_STATE_RECONFIGURING;
                LOG_DEBUG("Reconfig requested – rescanning bus...\r\n");
                // motor_scan(reconfig_ids, MOTOR_MAX);     // motor_scan(boot_ids, MOTOR_MAX); //TODO: temporary disable this. Otherwise, program is stuck 
                task_state = TASK_STATE_RUNNING;
            }
        }

        /* ── 2. Fault clear ───────────────────────────────────────── */
        if (can_rx_mutex != NULL &&
            xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            bool    do_clear = g_can_fault_clear.pending;
            uint8_t clear_id = g_can_fault_clear.motor_id;
            if (do_clear) { g_can_fault_clear.pending = false; }
            xSemaphoreGive(can_rx_mutex);

            if (do_clear)
            {
                uint8_t start = (clear_id == 0u) ? 0u         : (uint8_t)(clear_id - 1u);
                uint8_t end   = (clear_id == 0u) ? MOTOR_MAX  : (uint8_t)clear_id;

                for (uint8_t i = start; i < end; i++)
                {
                    if (g_motor_state[i] == MOTOR_STATE_FAULT)
                    {
                        g_motor_state[i]   = MOTOR_STATE_ACTIVE;
                        g_servo_present[i] = true;
                        g_active_motor_count++;
                        HWSERVO_EnableTorque(&servo[i], true);
                        LOG_DEBUG("motor[%u] %s fault cleared by RPi\r\n",
                                  i, g_motor_configs[i].name);
                    }
                }
            }
        }

#ifdef ENABLE_MOTOR_TEST
        /* ── 3. Test mode: round-robin ACTIVE motors ──────────────── */
        {
            uint8_t tries = 0u;
            while (g_motor_state[test_motor] != MOTOR_STATE_ACTIVE &&
                   tries < MOTOR_MAX)
            {
                test_motor = (test_motor + 1u) % MOTOR_MAX;
                tries++;
            }
        }

        if (g_motor_state[test_motor] == MOTOR_STATE_ACTIVE)
        {
            const motor_config_t *cfg = &g_motor_configs[test_motor];
            hiwonder_servo_t     *s   = &servo[test_motor];

            HWSERVO_MoveToAngle(s, test_deg[test_motor], 1000);
            vTaskDelay(pdMS_TO_TICKS(1500));

            int16_t actual_raw = 0;
            if (HWSERVO_ReadPos_Raw(s, &actual_raw) == HWSERVO_OK) {
                LOG_DEBUG("%s: TEST read <- %d raw\r\n", cfg->name, actual_raw);
            } else {
                LOG_DEBUG("%s: TEST readback FAILED\r\n", cfg->name);
            }

            test_deg[test_motor] += 20.0f;
            if (test_deg[test_motor] > s->spec.deg_max) {
                test_deg[test_motor] = s->spec.deg_min;
            }
        }
        test_motor = (test_motor + 1u) % MOTOR_MAX;
        vTaskDelay(pdMS_TO_TICKS(25));  /* yield even when no motors are active */
#else /* Normal CAN-driven operation */

        /* ── 3. Process CAN commands (ACTIVE motors only) ────────── */
        for (uint8_t i = 0; i < MOTOR_MAX; i++)
        {
            if (g_motor_state[i] != MOTOR_STATE_ACTIVE || can_rx_mutex == NULL) {
                continue;
            }

            if (xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(2)) != pdTRUE) {
                continue;
            }

            if (!g_can_rx[i].fresh) {
                xSemaphoreGive(can_rx_mutex);
                continue;
            }

            motor_cmd_t cmd   = g_can_rx[i].cmd;
            g_can_rx[i].fresh = false;
            xSemaphoreGive(can_rx_mutex);

            const motor_config_t *cfg = &g_motor_configs[i];
            hiwonder_servo_t     *s   = &servo[i];

            if (cmd.stop_cmd)
            {
                HWSERVO_MoveStop(s);
                LOG_DEBUG("%s: CAN stop\r\n", cfg->name);
            }
            else
            {
                HWSERVO_EnableTorque(s, cmd.torque_enable != 0u);
                if (cmd.torque_enable) {
                    float deg = (float)cmd.target_angle_deg * 0.01f;
                    HWSERVO_MoveToAngle(s, deg, cmd.move_duration_ms);
                }
            }
        }

        /* ── 4. Health monitoring (round-robin, one motor per interval) */
        if ((xTaskGetTickCount() - xLastHealthTick) >=
            pdMS_TO_TICKS(HEALTH_CHECK_INTERVAL_MS))
        {
            xLastHealthTick = xTaskGetTickCount();

            /* Advance to next non-absent slot */
            uint8_t tries = 0u;
            do {
                health_slot = (health_slot + 1u) % MOTOR_MAX;
                tries++;
            } while (g_motor_state[health_slot] == MOTOR_STATE_ABSENT &&
                     tries < MOTOR_MAX);

            if (g_motor_state[health_slot] != MOTOR_STATE_ABSENT) {
                motor_check_health(health_slot);
            }
        }

#ifdef ENABLE_PUBLISH_STATUS
        /* ── 5. Publish MCU_Status (ACTIVE motors only) ────────────── */
        if ((xTaskGetTickCount() - xLastStatusTick) >=
            pdMS_TO_TICKS(SERVO_STATUS_PERIOD_MS))
        {
            xLastStatusTick = xTaskGetTickCount();

            for (uint8_t i = 0; i < MOTOR_MAX; i++)
            {
                if (g_motor_state[i] != MOTOR_STATE_ACTIVE) {
                    continue;
                }

                hiwonder_servo_t *s = &servo[i];
                motor_status_t status = { 0 };

                float deg = 0.0f;
                if (HWSERVO_ReadAngle_deg(s, &deg) == HWSERVO_OK) {
                    status.joint_angle_deg = (uint16_t)(deg * 100.0f + 0.5f);
                }

                uint16_t mv = 0u;
                if (HWSERVO_ReadVin_mV(s, &mv) == HWSERVO_OK) {
                    status.joint_voltage_m_v = mv;
                }

                uint8_t temp = 0u;
                if (HWSERVO_ReadTemp_C(s, &temp) == HWSERVO_OK) {
                    status.joint_temp_c = temp;
                }

                status.torque_enabled = 1u;
                CAN_SendMcuStatus(i, &status);
            }
        }
#endif /* ENABLE_PUBLISH_STATUS */

        vTaskDelay(pdMS_TO_TICKS(25));
#endif /* ENABLE_MOTOR_TEST */
    }
}

