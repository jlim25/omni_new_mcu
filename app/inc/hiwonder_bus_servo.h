#pragma once

#include "debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Protocol constants ────────────────────────────────────────── */

// Command bytes (Hiwonder Bus Servo Protocol)
#define HWSERVO_CMD_MOVE_TIME_WRITE        0x01
#define HWSERVO_CMD_POS_READ               0x1C  // 28
#define HWSERVO_CMD_TEMP_READ              0x1A  // 26
#define HWSERVO_CMD_VIN_READ               0x1B  // 27
#define HWSERVO_CMD_LOAD_OR_UNLOAD_WRITE   0x1F  // 31

#define HWSERVO_BROADCAST_ID               0xFE
#define HWSERVO_FRAME_HEADER               0x55

#define HWSERVO_RAW_MIN                    0
#define HWSERVO_RAW_MAX                    1000

/* ── Status codes ──────────────────────────────────────────────── */

typedef enum {
    HWSERVO_OK = 0,
    HWSERVO_ERR_TIMEOUT,
    HWSERVO_ERR_CHECKSUM,
    HWSERVO_ERR_FRAME,
    HWSERVO_ERR_BUSY,
    HWSERVO_ERR_PARAM
} hwservo_status_t;

/* ── Motor physical specification ─────────────────────────────── */
/**
 * Describes the physical angular range of a specific servo model.
 * deg_min is the angle (in degrees) when the raw position is 0,
 * deg_max is the angle (in degrees) when the raw position is 1000.
 *
 * Example for a 240-degree servo centered at 120°:
 *   deg_min = 0.0f, deg_max = 240.0f
 */
typedef struct {
    float deg_min;  // degrees at raw=0
    float deg_max;  // degrees at raw=1000
} hiwonder_servo_spec_t;

// Common presets
// in servo mode, angle is restricted to 0-240 (0-1000 raw) deg, in geared motor mode (speed), it is 0-360 deg
#define HTD_85H_SPEC  { .deg_min = 0.0f, .deg_max = 240.0f }
#define HTD_45H_SPEC  { .deg_min = 0.0f, .deg_max = 240.0f }
#define HTD_35H_SPEC  { .deg_min = 0.0f, .deg_max = 240.0f }

/* ── Servo handle ──────────────────────────────────────────────── */
/**
 * Represents a single physical servo motor.
 * When multiple servos share the same UART bus, pass the same
 * bus_mutex and huart to each instance.
 */
typedef struct {
    // --- Bus interface ---
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *dir_port;
    uint16_t            dir_pin;
    bool                dir_tx_high;  // true  → DIR=HIGH means TX (MCU→Servo)
                                      // false → DIR=LOW  means TX

    // FreeRTOS protection
    SemaphoreHandle_t bus_mutex;

    // Tuning
    uint32_t            tx_timeout_ms;
    uint32_t            rx_timeout_ms;

    // --- Motor identity ---
    uint8_t             id;    // servo ID on the bus (1–253; 254 = broadcast)

    // --- Physical spec ---
    hiwonder_servo_spec_t spec;
} hiwonder_servo_t;

/* ── Initialisation ────────────────────────────────────────────── */

/**
 * Initialise a servo handle.
 *
 * @param servo        Handle to populate.
 * @param huart        UART peripheral for the half-duplex bus.
 * @param dir_port     GPIO port of the direction-control pin.
 * @param dir_pin      GPIO pin mask of the direction-control pin.
 * @param dir_tx_high  true if DIR=HIGH selects TX direction.
 * @param bus_mutex    FreeRTOS mutex shared by all servos on this bus
 *                     (may be NULL to skip locking).
 * @param id           Servo hardware ID (1–253).
 * @param spec         Physical angular range of this servo model.
 */
hwservo_status_t HWSERVO_Init(hiwonder_servo_t *servo,
                              UART_HandleTypeDef *huart,
                              GPIO_TypeDef *dir_port, uint16_t dir_pin,
                              bool dir_tx_high,
                              SemaphoreHandle_t bus_mutex,
                              uint8_t id,
                              hiwonder_servo_spec_t spec);

/* ── Presence detection ────────────────────────────────────────── */

/** Timeout used during bus scans – shorter than normal reads. */
#define HWSERVO_PING_TIMEOUT_MS   25u

/**
 * Ping a servo to check whether it is present on the bus.
 *
 * Sends a Read-Voltage command (0x1B) using a short RX timeout
 * (HWSERVO_PING_TIMEOUT_MS).  Returns HWSERVO_OK if the servo
 * responds with a valid frame, or HWSERVO_ERR_TIMEOUT / HWSERVO_ERR_FRAME
 * if it does not.
 *
 * The bus mutex is acquired internally.  Safe to call from any task.
 */
hwservo_status_t HWSERVO_Ping(hiwonder_servo_t *servo);

/* ── Motion commands ───────────────────────────────────────────── */

/**
 * Move to a raw position (0–1000) over time_ms milliseconds.
 * Use HWSERVO_MoveToAngle() for degree-based control.
 */
hwservo_status_t HWSERVO_MoveTimeWrite_Raw(hiwonder_servo_t *servo,
                                           uint16_t pos_0_1000,
                                           uint16_t time_ms);

/**
 * Move to an angle (degrees) over time_ms milliseconds.
 * The value is clamped to the servo's configured spec (deg_min..deg_max).
 */
hwservo_status_t HWSERVO_MoveToAngle(hiwonder_servo_t *servo,
                                     float deg,
                                     uint16_t time_ms);

/** Enable (load) or disable (unload) servo torque. */
hwservo_status_t HWSERVO_EnableTorque(hiwonder_servo_t *servo, bool enable);

/* ── Read-back commands ────────────────────────────────────────── */

/** Read raw position (0–1000). */
hwservo_status_t HWSERVO_ReadPos_Raw(hiwonder_servo_t *servo, int16_t *pos_out);

/**
 * Read current position in degrees.
 * Converts the raw value using the servo's configured spec.
 */
hwservo_status_t HWSERVO_ReadAngle_deg(hiwonder_servo_t *servo, float *deg_out);

/** Read supply voltage in millivolts. */
hwservo_status_t HWSERVO_ReadVin_mV(hiwonder_servo_t *servo, uint16_t *mv_out);

/** Read internal temperature in degrees Celsius. */
hwservo_status_t HWSERVO_ReadTemp_C(hiwonder_servo_t *servo, uint8_t *tempC_out);

/* ── Unit-conversion utilities ─────────────────────────────────── */

/** Convert degrees → raw position (0–1000), clamped to valid range. */
static inline uint16_t HWSERVO_DegToRaw(const hiwonder_servo_spec_t *spec, float deg)
{
    if (deg <= spec->deg_min) return HWSERVO_RAW_MIN;
    if (deg >= spec->deg_max) return HWSERVO_RAW_MAX;
    float ratio = (deg - spec->deg_min) / (spec->deg_max - spec->deg_min);
    return (uint16_t)(ratio * (float)HWSERVO_RAW_MAX + 0.5f);
}

/** Convert raw position (0–1000) → degrees. */
static inline float HWSERVO_RawToDeg(const hiwonder_servo_spec_t *spec, int16_t raw)
{
    float ratio = (float)raw / (float)HWSERVO_RAW_MAX;
    return spec->deg_min + ratio * (spec->deg_max - spec->deg_min);
}

#ifdef __cplusplus
}
#endif