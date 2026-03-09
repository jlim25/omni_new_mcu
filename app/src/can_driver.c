/**
  *****************************************************************************
  * @file    can_driver.c
  * @brief   CAN RX/TX tasks and ISR callback.
  *
  *  Flow
  *  ----
  *  RX:  CAN frame arrives → HAL_CAN_RxFifo0MsgPendingCallback() (ISR)
  *         → vTaskNotifyGiveFromISR() → canRxTask() wakes
  *         → HAL_CAN_GetRxMessage() → match CAN ID against g_motor_configs
  *         → motor_cmd_unpack_fn() → update g_can_rx[i] under can_rx_mutex
  *
  *  TX:  Any task calls CAN_SendMcuStatus(i, &status) → xQueueSend(canTxQueue)
  *         → canTxTask() wakes → HAL_CAN_AddTxMessage()
  *****************************************************************************
  */

#include "can_driver.h"
#include "can.h"
#include "logger.h"
#include "omni_robot.h"

/* ── Shared RX state (one entry per motor slot) ─────────────────── */
can_rx_state_t       g_can_rx[MOTOR_MAX]   = { 0 };
can_reconfig_state_t  g_can_reconfig        = { 0 };
can_fault_clear_state_t g_can_fault_clear   = { 0 };
SemaphoreHandle_t     can_rx_mutex          = NULL;

/* ── TX queue ───────────────────────────────────────────────────── */
QueueHandle_t canTxQueue = NULL;

/* ── Task handle needed by the ISR to send a notification ──────── */
static TaskHandle_t s_canRxTaskHandle = NULL;

/* ── ISR callback ───────────────────────────────────────────────── */
/**
 * Called by HAL inside CAN_RX0_IRQHandler (stm32f3xx_it.c).
 * Do the absolute minimum here: notify the RX task and yield if needed.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* CAN_IT_RX_FIFO0_MSG_PENDING is level-triggered: it stays asserted while
     * the FIFO is non-empty.  Disable it here so the ISR does not re-fire
     * continuously before canRxTask has a chance to drain the FIFO.
     * canRxTask re-enables it after draining. */
    __HAL_CAN_DISABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

    if (s_canRxTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(s_canRxTaskHandle, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ── canRxTask ──────────────────────────────────────────────────── */
void canRxTask(void const *argument)
{
    (void)argument;

    /* Store our own handle so the ISR can notify us */
    s_canRxTaskHandle = xTaskGetCurrentTaskHandle();

    /* Create the mutex that guards g_can_rx[] */
    can_rx_mutex = xSemaphoreCreateMutex();
    configASSERT(can_rx_mutex != NULL);

    CAN_RxHeaderTypeDef rxHeader;
    uint8_t             rxData[8];

    for (;;)
    {
        /* Block indefinitely until the ISR fires */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Drain all pending frames */
        while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0u)
        {
            if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,
                                     &rxHeader, rxData) != HAL_OK) {
                LOG_ERROR("CAN RX read error\r\n");
                break;
            }

            /* Route by CAN ID – search all motor configs */
            bool matched = false;
            for (uint8_t i = 0; i < MOTOR_MAX; i++)
            {
                if (rxHeader.StdId != g_motor_configs[i].can_cmd_id) {
                    continue;
                }

                motor_cmd_t decoded;
                if (g_motor_configs[i].cmd_unpack(&decoded, rxData,
                                                   rxHeader.DLC) >= 0)
                {
                    if (xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(5))
                        == pdTRUE)
                    {
                        g_can_rx[i].cmd   = decoded;
                        g_can_rx[i].fresh = true;
                        xSemaphoreGive(can_rx_mutex);
                    }
                } else {
                    LOG_WARNING("CAN RX unpack failed for %s\r\n",
                                g_motor_configs[i].name);
                }

                matched = true;
                break;
            }

            if (!matched)
            {
                /* RPi_Reconfig (0x300): explicit servo-ID list for rescan */
                if (rxHeader.StdId == MOTOR_RECONFIG_CAN_ID &&
                    rxHeader.DLC   == OMNI_ROBOT_R_PI_RECONFIG_LENGTH)
                {
                    struct omni_robot_r_pi_reconfig_t rec;
                    if (omni_robot_r_pi_reconfig_unpack(&rec, rxData,
                                                        rxHeader.DLC) >= 0)
                    {
                        if (xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(5))
                            == pdTRUE)
                        {
                            g_can_reconfig.servo_ids[0] = rec.servo_id_1;
                            g_can_reconfig.servo_ids[1] = rec.servo_id_2;
                            g_can_reconfig.servo_ids[2] = rec.servo_id_3;
                            g_can_reconfig.servo_ids[3] = rec.servo_id_4;
                            g_can_reconfig.servo_ids[4] = rec.servo_id_5;
                            g_can_reconfig.servo_ids[5] = rec.servo_id_6;
                            g_can_reconfig.pending      = true;
                            xSemaphoreGive(can_rx_mutex);
                        }
                        matched = true;
                    } else {
                        LOG_WARNING("CAN RX RPi_Reconfig unpack failed\r\n");
                    }
                }

                /* RPi_FaultClear (0x301): clear fault for a specific motor or all */
                if (!matched &&
                    rxHeader.StdId == MOTOR_FAULT_CLEAR_CAN_ID &&
                    rxHeader.DLC   == OMNI_ROBOT_R_PI_FAULT_CLEAR_LENGTH)
                {
                    struct omni_robot_r_pi_fault_clear_t fc;
                    if (omni_robot_r_pi_fault_clear_unpack(&fc, rxData,
                                                           rxHeader.DLC) >= 0)
                    {
                        if (xSemaphoreTake(can_rx_mutex, pdMS_TO_TICKS(5))
                            == pdTRUE)
                        {
                            g_can_fault_clear.motor_id = fc.motor_id;
                            g_can_fault_clear.pending  = true;
                            xSemaphoreGive(can_rx_mutex);
                        }
                        matched = true;
                    } else {
                        LOG_WARNING("CAN RX RPi_FaultClear unpack failed\r\n");
                    }
                }

                if (!matched) {
                    LOG_WARNING("CAN RX unknown ID 0x%03lX\r\n",
                                (unsigned long)rxHeader.StdId);
                }
            }
        }

        /* FIFO is now empty – re-enable the interrupt so the next arriving
         * message triggers the ISR again. */
        __HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

/* ── canTxTask ──────────────────────────────────────────────────── */
void canTxTask(void const *argument)
{
    (void)argument;

    /* Create the TX queue */
    canTxQueue = xQueueCreate(CAN_TX_QUEUE_DEPTH, sizeof(can_raw_frame_t));
    configASSERT(canTxQueue != NULL);

    can_raw_frame_t  frame;
    CAN_TxHeaderTypeDef txHeader = {
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .TransmitGlobalTime = DISABLE,
    };
    uint32_t txMailbox = 0;

    for (;;)
    {
        /* Block until something is queued for TX */
        if (xQueueReceive(canTxQueue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        txHeader.StdId = frame.id;
        txHeader.DLC   = frame.dlc;

        /* Wait for a free mailbox (up to 10 ms) before giving up */
        uint32_t deadline = HAL_GetTick() + 10u;
        while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u) {
            if (HAL_GetTick() >= deadline) {
                LOG_WARNING("CAN TX mailbox timeout, dropping frame 0x%03lX\r\n",
                         (unsigned long)frame.id);
                break;
            }
            taskYIELD();
        }

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0u) {
            if (HAL_CAN_AddTxMessage(&hcan1, &txHeader,
                                     frame.data, &txMailbox) != HAL_OK) {
                LOG_ERROR("CAN TX failed for id 0x%03lX\r\n",
                          (unsigned long)frame.id);
            }
        }
    }
}

/* ── CAN_SendMcuStatus ──────────────────────────────────────────── */
void CAN_SendMcuStatus(uint8_t motor_idx, const motor_status_t *status)
{
    if (motor_idx >= MOTOR_MAX) {
        LOG_ERROR("CAN_SendMcuStatus: invalid motor_idx %u\r\n", motor_idx);
        return;
    }

    const motor_config_t *cfg = &g_motor_configs[motor_idx];

    can_raw_frame_t frame;
    frame.id  = cfg->can_status_id;
    frame.dlc = cfg->can_status_len;

    if (cfg->status_pack(frame.data, status, sizeof(frame.data)) < 0) {
        LOG_ERROR("CAN_SendMcuStatus: pack failed for %s\r\n", cfg->name);
        return;
    }

    /* Non-blocking: drop if queue is full to avoid stalling the caller */
    if (canTxQueue != NULL) {
        xQueueSend(canTxQueue, &frame, 0);
    }
}

/* ── CAN_SendMcuFault ─────────────────────────────────────────────── */
void CAN_SendMcuFault(uint8_t motor_id, uint8_t fault_code, uint16_t fault_value)
{
    struct omni_robot_mcu_fault_t fault_msg = {
        .motor_id    = motor_id,
        .fault_code  = fault_code,
        .fault_value = fault_value,
    };

    can_raw_frame_t frame;
    frame.id  = MOTOR_FAULT_CAN_ID;
    frame.dlc = OMNI_ROBOT_MCU_FAULT_LENGTH;

    if (omni_robot_mcu_fault_pack(frame.data, &fault_msg, sizeof(frame.data)) < 0) {
        LOG_ERROR("CAN_SendMcuFault: pack failed\r\n");
        return;
    }

    if (canTxQueue != NULL) {
        xQueueSend(canTxQueue, &frame, 0);
    }
}
