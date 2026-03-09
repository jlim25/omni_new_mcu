/**
  *****************************************************************************
  * @file    hiwonder_bus_servo.c
  * @author  Jacky Lim
  * @brief   Implementation for controlling Hiwonder Bus Servos.
  *
  *          Supports multiple servo models on the same half-duplex UART bus.
  *          Each hiwonder_servo_t handle bundles the bus interface, the
  *          servo's hardware ID, and its physical angular spec so that
  *          angle-based functions work correctly per motor type.
  ******************************************************************************
  */

#include "hiwonder_bus_servo.h"
#include "string.h"
#include "debug.h"

#define HWSERVO_MAX_FRAME   32  // plenty for all supported commands

/* ── Helper functions ─────────────────────────────────────────── */

static inline void set_dir_tx(const hiwonder_servo_t *servo, bool tx)
{
    // Active-low: pull pin LOW to enable TX, HIGH for RX
    GPIO_PinState st = tx ? GPIO_PIN_RESET : GPIO_PIN_SET;
    HAL_GPIO_WritePin(servo->dir_port, servo->dir_pin, st);
}

static uint8_t calc_checksum(uint8_t id, uint8_t len, uint8_t cmd,
                             const uint8_t *params, uint8_t nparams)
{
    // Checksum = ~(ID + Length + Cmd + Prm1 + ... + PrmN)  (low byte)
    uint16_t sum = (uint16_t)id + len + cmd;
    for (uint8_t i = 0; i < nparams; i++) {
        sum += params[i];
    }
    // Note if the checksum exceeds 255, then take the lowest byte 
    return (uint8_t)(~(sum & 0xFF));
}

static hwservo_status_t bus_lock(const hiwonder_servo_t *servo)
{
    if (servo->bus_mutex == NULL) return HWSERVO_OK;
    return (xSemaphoreTake(servo->bus_mutex, portMAX_DELAY) == pdTRUE)
           ? HWSERVO_OK : HWSERVO_ERR_BUSY;
}

static void bus_unlock(const hiwonder_servo_t *servo)
{
    if (servo->bus_mutex != NULL) {
        xSemaphoreGive(servo->bus_mutex);
    }
}

static hwservo_status_t tx_frame(hiwonder_servo_t *servo,
                                 const uint8_t *buf, uint16_t len)
{
    set_dir_tx(servo, true);

    // Switch UART peripheral to transmit mode (half-duplex: TE=1, RE=0)
    HAL_HalfDuplex_EnableTransmitter(servo->huart);

    // Brief settle time for the direction-controlled line driver
    for (volatile int i = 0; i < 200; i++) { __NOP(); }

    if (HAL_UART_Transmit(servo->huart, (uint8_t *)buf, len,
                          servo->tx_timeout_ms) != HAL_OK) {
        return HWSERVO_ERR_TIMEOUT;
    }

    // Wait for the last bit to leave the shift register before switching to RX
    while (__HAL_UART_GET_FLAG(servo->huart, UART_FLAG_TC) == RESET) { }

    // Switch to RX and flush the echo bytes: on a half-duplex bus the STM32
    // receives its own transmitted bytes in the RX shift register.  If not
    // drained here, they will be mistaken for the servo's reply.
    set_dir_tx(servo, false);
    HAL_HalfDuplex_EnableReceiver(servo->huart);

    return HWSERVO_OK;
}

static hwservo_status_t rx_byte(hiwonder_servo_t *servo,
                                uint8_t *b, uint32_t timeout_ms)
{
    if (HAL_UART_Receive(servo->huart, b, 1, timeout_ms) != HAL_OK) {
        return HWSERVO_ERR_TIMEOUT;
    }
    return HWSERVO_OK;
}

/**
 * Receive a complete frame into out_buf.
 * Frame layout: 55 55 ID LEN CMD [PARAMS...] CHECKSUM
 * LEN counts itself, so total bytes = 2(hdr) + 1(ID) + LEN.
 */
static hwservo_status_t rx_frame(hiwonder_servo_t *servo,
                                 uint8_t *out_buf, uint16_t out_max,
                                 uint16_t *out_len)
{
    // Direction switch and echo flush are already done at the end of tx_frame.
    // Small settle before we start reading the actual servo reply.
    for (volatile int i = 0; i < 200; i++) { __NOP(); }

    // Hunt for the 0x55 0x55 header
    uint8_t b = 0, prev = 0;
    while (1) {
        hwservo_status_t st = rx_byte(servo, &b, servo->rx_timeout_ms);
        if (st != HWSERVO_OK) return st;

        if (prev == HWSERVO_FRAME_HEADER && b == HWSERVO_FRAME_HEADER) break;
        prev = b;
    }

    uint16_t idx = 0;
    out_buf[idx++] = HWSERVO_FRAME_HEADER;
    out_buf[idx++] = HWSERVO_FRAME_HEADER;

    uint8_t id = 0, len = 0;
    if (rx_byte(servo, &id,  servo->rx_timeout_ms) != HWSERVO_OK) return HWSERVO_ERR_TIMEOUT;
    if (rx_byte(servo, &len, servo->rx_timeout_ms) != HWSERVO_OK) return HWSERVO_ERR_TIMEOUT;

    out_buf[idx++] = id;
    out_buf[idx++] = len;

    if (len < 3) return HWSERVO_ERR_FRAME; // minimum: LEN itself + CMD + CHECKSUM

    uint16_t total_needed = (uint16_t)(2 + 1 + len); // hdr(2) + id(1) + remaining(len)
    if (total_needed > out_max) return HWSERVO_ERR_FRAME;

    // Remaining bytes after LEN: (LEN - 1) = CMD + [PARAMS] + CHECKSUM
    uint16_t remain = (uint16_t)(len - 1);
    for (uint16_t i = 0; i < remain; i++) {
        if (rx_byte(servo, &out_buf[idx++], servo->rx_timeout_ms) != HWSERVO_OK) {
            return HWSERVO_ERR_TIMEOUT;
        }
    }

    *out_len = idx;
    return HWSERVO_OK;
}

static hwservo_status_t build_frame(uint8_t id, uint8_t cmd,
                                    const uint8_t *params, uint8_t nparams,
                                    uint8_t *out, uint16_t out_max,
                                    uint16_t *out_len)
{
    // LEN = 3 + nparams  (counts LEN itself + CMD + CHECKSUM)
    uint8_t len = (uint8_t)(3 + nparams);
    uint16_t total = (uint16_t)(2 + 1 + len);
    if (total > out_max) return HWSERVO_ERR_PARAM;

    uint16_t i = 0;
    out[i++] = 0x55; // correct
    out[i++] = 0x55; // correct
    out[i++] = id; // correct
    out[i++] = len; // includes its own byte // TODO: verify this
    out[i++] = cmd; // correct
    for (uint8_t p = 0; p < nparams; p++) {
        out[i++] = params[p]; // nparams is correct
        // params is correct
    }
    // checksum seems correct
    out[i++] = calc_checksum(id, len, cmd, params, nparams);

    *out_len = i;
    return HWSERVO_OK;
}

static hwservo_status_t send_and_optional_read(hiwonder_servo_t *servo,
                                               uint8_t cmd,
                                               const uint8_t *params,
                                               uint8_t nparams,
                                               bool expect_reply,
                                               uint8_t *rx_buf,
                                               uint16_t rx_max,
                                               uint16_t *rx_len)
{
    uint8_t tx[HWSERVO_MAX_FRAME];
    uint16_t tx_len = 0;

    hwservo_status_t st = build_frame(servo->id, cmd,
                                      params, nparams,
                                      tx, sizeof(tx), &tx_len);
    if (st != HWSERVO_OK) return st;

    st = tx_frame(servo, tx, tx_len);
    if (st != HWSERVO_OK) return st;

    if (!expect_reply) return HWSERVO_OK;

    return rx_frame(servo, rx_buf, rx_max, rx_len);
}

static hwservo_status_t parse_reply(const uint8_t *rx, uint16_t rx_len,
                                    uint8_t expect_id, uint8_t expect_cmd,
                                    const uint8_t **params_out,
                                    uint8_t *nparams_out)
{
    if (rx_len < 2 + 1 + 3)             return HWSERVO_ERR_FRAME;
    if (rx[0] != 0x55 || rx[1] != 0x55) return HWSERVO_ERR_FRAME;

    uint8_t id  = rx[2];
    uint8_t len = rx[3];
    uint8_t cmd = rx[4];

    if (id  != expect_id)  return HWSERVO_ERR_FRAME;
    if (cmd != expect_cmd) return HWSERVO_ERR_FRAME;

    uint16_t expected_total = (uint16_t)(2 + 1 + len);
    if (rx_len != expected_total) return HWSERVO_ERR_FRAME;

    uint8_t nparams     = (uint8_t)(len - 3); // LEN - (LEN byte + CMD + CHECKSUM)
    const uint8_t *params = &rx[5];
    uint8_t chk         = rx[rx_len - 1];

    if (chk != calc_checksum(id, len, cmd, params, nparams)) return HWSERVO_ERR_CHECKSUM;

    *params_out  = params;
    *nparams_out = nparams;
    return HWSERVO_OK;
}

/* ── Public API ────────────────────────────────────────────────── */

hwservo_status_t HWSERVO_Ping(hiwonder_servo_t *servo)
{
    /* Temporarily shorten the RX timeout for fast scan, restore afterward. */
    uint32_t saved_rx = servo->rx_timeout_ms;
    servo->rx_timeout_ms = HWSERVO_PING_TIMEOUT_MS;

    uint16_t mv_dummy = 0;
    hwservo_status_t st = HWSERVO_ReadVin_mV(servo, &mv_dummy);

    servo->rx_timeout_ms = saved_rx;
    return st;
}

hwservo_status_t HWSERVO_Init(hiwonder_servo_t *servo,
                              UART_HandleTypeDef *huart,
                              GPIO_TypeDef *dir_port, uint16_t dir_pin,
                              bool dir_tx_high,
                              SemaphoreHandle_t bus_mutex,
                              uint8_t id,
                              hiwonder_servo_spec_t spec)
{
    if (!servo || !huart || !dir_port) return HWSERVO_ERR_PARAM;
    if (spec.deg_min >= spec.deg_max)   return HWSERVO_ERR_PARAM;

    memset(servo, 0, sizeof(*servo));

    servo->huart       = huart;
    servo->dir_port    = dir_port;
    servo->dir_pin     = dir_pin;
    servo->dir_tx_high = dir_tx_high;
    servo->bus_mutex   = bus_mutex;
    servo->id          = id;
    servo->spec        = spec;

    servo->tx_timeout_ms = 20;
    servo->rx_timeout_ms = 50;

    set_dir_tx(servo, false); // default to RX
    return HWSERVO_OK;
}

/* ── Motion ───────────────────────────────────────────────────── */

hwservo_status_t HWSERVO_MoveTimeWrite_Raw(hiwonder_servo_t *servo,
                                           uint16_t pos_0_1000,
                                           uint16_t time_ms)
{
    if (!servo)                return HWSERVO_ERR_PARAM;
    if (pos_0_1000 > 1000)     return HWSERVO_ERR_PARAM;
    if (time_ms > 30000)       return HWSERVO_ERR_PARAM;

    uint8_t prm[4];
    prm[0] = (uint8_t)(pos_0_1000 & 0xFF); // low byte of position
    prm[1] = (uint8_t)((pos_0_1000 >> 8) & 0xFF); // high byte of position
    prm[2] = (uint8_t)(time_ms & 0xFF); // low byte of time
    prm[3] = (uint8_t)((time_ms >> 8) & 0xFF); // high byte of time
    // ^ correct

    hwservo_status_t st = bus_lock(servo);
    if (st != HWSERVO_OK) return st;

    st = send_and_optional_read(servo, HWSERVO_CMD_MOVE_TIME_WRITE,
                                prm, sizeof(prm),
                                false, NULL, 0, NULL);
    bus_unlock(servo);
    return st;
}

hwservo_status_t HWSERVO_MoveToAngle(hiwonder_servo_t *servo,
                                     float deg,
                                     uint16_t time_ms)
{
    if (!servo) return HWSERVO_ERR_PARAM;

    uint16_t raw = HWSERVO_DegToRaw(&servo->spec, deg);
    return HWSERVO_MoveTimeWrite_Raw(servo, raw, time_ms);
}

hwservo_status_t HWSERVO_EnableTorque(hiwonder_servo_t *servo, bool enable)
{
    if (!servo) return HWSERVO_ERR_PARAM;

    uint8_t prm[1] = { enable ? 1u : 0u };

    hwservo_status_t st = bus_lock(servo);
    if (st != HWSERVO_OK) return st;

    st = send_and_optional_read(servo, HWSERVO_CMD_LOAD_OR_UNLOAD_WRITE,
                                prm, sizeof(prm),
                                false, NULL, 0, NULL);
    bus_unlock(servo);
    return st;
}

/* ── Read-back ────────────────────────────────────────────────── */

hwservo_status_t HWSERVO_ReadPos_Raw(hiwonder_servo_t *servo, int16_t *pos_out)
{
    if (!servo || !pos_out) return HWSERVO_ERR_PARAM;

    uint8_t rx[HWSERVO_MAX_FRAME];
    uint16_t rx_len = 0;

    hwservo_status_t st = bus_lock(servo);
    if (st != HWSERVO_OK) return st;

    st = send_and_optional_read(servo, HWSERVO_CMD_POS_READ,
                                NULL, 0,
                                true, rx, sizeof(rx), &rx_len);
    bus_unlock(servo);
    if (st != HWSERVO_OK) return st;

    const uint8_t *params = NULL;
    uint8_t nparams = 0;
    st = parse_reply(rx, rx_len, servo->id, HWSERVO_CMD_POS_READ,
                     &params, &nparams);
    if (st != HWSERVO_OK) return st;

    if (nparams < 2) return HWSERVO_ERR_FRAME;
    *pos_out = (int16_t)((uint16_t)params[0] | ((uint16_t)params[1] << 8));
    return HWSERVO_OK;
}

hwservo_status_t HWSERVO_ReadAngle_deg(hiwonder_servo_t *servo, float *deg_out)
{
    if (!servo || !deg_out) return HWSERVO_ERR_PARAM;

    int16_t raw = 0;
    hwservo_status_t st = HWSERVO_ReadPos_Raw(servo, &raw);
    if (st != HWSERVO_OK) return st;

    *deg_out = HWSERVO_RawToDeg(&servo->spec, raw);
    return HWSERVO_OK;
}

hwservo_status_t HWSERVO_ReadVin_mV(hiwonder_servo_t *servo, uint16_t *mv_out)
{
    if (!servo || !mv_out) return HWSERVO_ERR_PARAM;

    uint8_t rx[HWSERVO_MAX_FRAME];
    uint16_t rx_len = 0;

    hwservo_status_t st = bus_lock(servo);
    if (st != HWSERVO_OK) return st;

    st = send_and_optional_read(servo, HWSERVO_CMD_VIN_READ,
                                NULL, 0,
                                true, rx, sizeof(rx), &rx_len);
    bus_unlock(servo);
    if (st != HWSERVO_OK) return st;

    const uint8_t *params = NULL;
    uint8_t nparams = 0;
    st = parse_reply(rx, rx_len, servo->id, HWSERVO_CMD_VIN_READ,
                     &params, &nparams);
    if (st != HWSERVO_OK) return st;

    if (nparams < 2) return HWSERVO_ERR_FRAME;
    *mv_out = (uint16_t)(params[0] | ((uint16_t)params[1] << 8));
    return HWSERVO_OK;
}

hwservo_status_t HWSERVO_ReadTemp_C(hiwonder_servo_t *servo, uint8_t *tempC_out)
{
    if (!servo || !tempC_out) return HWSERVO_ERR_PARAM;

    uint8_t rx[HWSERVO_MAX_FRAME];
    uint16_t rx_len = 0;

    hwservo_status_t st = bus_lock(servo);
    if (st != HWSERVO_OK) return st;

    st = send_and_optional_read(servo, HWSERVO_CMD_TEMP_READ,
                                NULL, 0,
                                true, rx, sizeof(rx), &rx_len);
    bus_unlock(servo);
    if (st != HWSERVO_OK) return st;

    const uint8_t *params = NULL;
    uint8_t nparams = 0;
    st = parse_reply(rx, rx_len, servo->id, HWSERVO_CMD_TEMP_READ,
                     &params, &nparams);
    if (st != HWSERVO_OK) return st;

    if (nparams < 1) return HWSERVO_ERR_FRAME;
    *tempC_out = params[0];
    return HWSERVO_OK;
}
