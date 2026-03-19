import time
import serial

LOBOT_FRAME_HEADER = 0x55
LOBOT_CMD_SERVO_MOVE = 3
LOBOT_CMD_ACTION_GROUP_RUN = 6
LOBOT_CMD_ACTION_GROUP_STOP = 7
LOBOT_CMD_ACTION_GROUP_SPEED = 11
LOBOT_CMD_GET_BATTERY_VOLTAGE = 15
LOBOT_CMD_MULT_SERVO_UNLOAD = 20
LOBOT_CMD_MULT_SERVO_POS_READ = 21

serialHandle = serial.Serial("/dev/ttyAMA0", 9600, timeout=0.1)


# Control single bus servo movement
def setBusServoMove(servo_id, servo_pulse, time_ms):
    buf = bytearray(b'\x55\x55')
    buf.append(0x08)
    buf.append(LOBOT_CMD_SERVO_MOVE)
    buf.append(0x01)

    time_ms = 0 if time_ms < 0 else time_ms
    time_ms = 30000 if time_ms > 30000 else time_ms
    time_list = list(time_ms.to_bytes(2, 'little'))
    buf.append(time_list[0])
    buf.append(time_list[1])

    servo_id = 254 if (servo_id < 1 or servo_id > 254) else servo_id
    buf.append(servo_id)

    servo_pulse = 0 if servo_pulse < 0 else servo_pulse
    servo_pulse = 1000 if servo_pulse > 1000 else servo_pulse
    pulse_list = list(servo_pulse.to_bytes(2, 'little'))
    buf.append(pulse_list[0])
    buf.append(pulse_list[1])

    serialHandle.write(buf)


# Control single PWM servo movement
def setPWMServoMove(servo_id, servo_pulse, time_ms):
    buf = bytearray(b'\x55\x55')
    buf.append(0x08)
    buf.append(LOBOT_CMD_SERVO_MOVE)
    buf.append(0x01)

    time_ms = 0 if time_ms < 0 else time_ms
    time_ms = 30000 if time_ms > 30000 else time_ms
    time_list = list(time_ms.to_bytes(2, 'little'))
    buf.append(time_list[0])
    buf.append(time_list[1])

    servo_id = 254 if (servo_id < 1 or servo_id > 254) else servo_id
    buf.append(servo_id)

    servo_pulse = 500 if servo_pulse < 500 else servo_pulse
    servo_pulse = 2500 if servo_pulse > 2500 else servo_pulse
    pulse_list = list(servo_pulse.to_bytes(2, 'little'))
    buf.append(pulse_list[0])
    buf.append(pulse_list[1])

    serialHandle.write(buf)


# Control multiple bus servos movement
def setMoreBusServoMove(servos, servos_count, time_ms):
    buf = bytearray(b'\x55\x55')
    buf.append(servos_count * 3 + 5)
    buf.append(LOBOT_CMD_SERVO_MOVE)

    servos_count = 1 if servos_count < 1 else servos_count
    servos_count = 254 if servos_count > 254 else servos_count
    buf.append(servos_count)

    time_ms = 0 if time_ms < 0 else time_ms
    time_ms = 30000 if time_ms > 30000 else time_ms
    time_list = list(time_ms.to_bytes(2, 'little'))
    buf.append(time_list[0])
    buf.append(time_list[1])

    for i in range(servos_count):
        buf.append(servos[i * 2])
        pos = servos[i * 2 + 1]
        pos = 0 if pos < 0 else pos
        pos = 1000 if pos > 1000 else pos
        pos_list = list(pos.to_bytes(2, 'little'))
        buf.append(pos_list[0])
        buf.append(pos_list[1])

    serialHandle.write(buf)


# Control multiple PWM servos movement
def setMorePWMServoMove(servos, servos_count, time_ms):
    buf = bytearray(b'\x55\x55')
    buf.append(servos_count * 3 + 5)
    buf.append(LOBOT_CMD_SERVO_MOVE)

    servos_count = 1 if servos_count < 1 else servos_count
    servos_count = 254 if servos_count > 254 else servos_count
    buf.append(servos_count)

    time_ms = 0 if time_ms < 0 else time_ms
    time_ms = 30000 if time_ms > 30000 else time_ms
    time_list = list(time_ms.to_bytes(2, 'little'))
    buf.append(time_list[0])
    buf.append(time_list[1])

    for i in range(servos_count):
        buf.append(servos[i * 2])
        pos = servos[i * 2 + 1]
        pos = 500 if pos < 500 else pos
        pos = 2500 if pos > 2500 else pos
        pos_list = list(pos.to_bytes(2, 'little'))
        buf.append(pos_list[0])
        buf.append(pos_list[1])

    serialHandle.write(buf)


def setGroupRun(group_id, group_count):
    buf = bytearray(b'\x55\x55')
    buf.append(5)
    buf.append(LOBOT_CMD_ACTION_GROUP_RUN)
    buf.append(group_id)
    count_list = list(group_count.to_bytes(2, 'little'))
    buf.append(count_list[0])
    buf.append(count_list[1])

    serialHandle.write(buf)


def setGroupStop():
    buf = bytearray(b'\x55\x55')
    buf.append(2)
    buf.append(LOBOT_CMD_ACTION_GROUP_STOP)
    serialHandle.write(buf)


def setGroupSpeed(group_id, group_speed):
    buf = bytearray(b'\x55\x55')
    buf.append(5)
    buf.append(LOBOT_CMD_ACTION_GROUP_SPEED)
    buf.append(group_id)

    speed_list = list(group_speed.to_bytes(2, 'little'))
    buf.append(speed_list[0])
    buf.append(speed_list[1])
    serialHandle.write(buf)


# Power off multiple bus servos
def setMultiServoUnload(servo_ids):
    servo_count = len(servo_ids)
    buf = bytearray(b'\x55\x55')
    buf.append(servo_count + 3)
    buf.append(LOBOT_CMD_MULT_SERVO_UNLOAD)
    buf.append(servo_count)

    for sid in servo_ids:
        sid = 254 if (sid < 1 or sid > 254) else sid
        buf.append(sid)

    serialHandle.write(buf)


# Read controller battery voltage, returns mV or None
def getBatteryVoltage():
    serialHandle.reset_input_buffer()

    buf = bytearray(b'\x55\x55')
    buf.append(2)
    buf.append(LOBOT_CMD_GET_BATTERY_VOLTAGE)

    serialHandle.write(buf)
    serialHandle.flush()
    time.sleep(0.05)

    resp = serialHandle.read(6)
    if len(resp) != 6:
        return None

    if resp[0] != 0x55 or resp[1] != 0x55:
        return None
    if resp[2] != 4:
        return None
    if resp[3] != LOBOT_CMD_GET_BATTERY_VOLTAGE:
        return None

    voltage = int.from_bytes(resp[4:6], byteorder='little', signed=False)
    return voltage


# Read multiple servo positions from Hiwonder Bus Servo Controller
# Returns dict like {1: 500, 2: 498, 3: 512}
def getControllerServoAngles(servo_ids):
    servo_count = len(servo_ids)
    if servo_count < 1:
        return {}

    serialHandle.reset_input_buffer()

    buf = bytearray(b'\x55\x55')
    buf.append(servo_count + 3)
    buf.append(LOBOT_CMD_MULT_SERVO_POS_READ)
    buf.append(servo_count)

    for sid in servo_ids:
        sid = 254 if (sid < 1 or sid > 254) else sid
        buf.append(sid)

    serialHandle.write(buf)
    serialHandle.flush()
    time.sleep(0.1)

    resp = serialHandle.read(200)
    print("RAW:", [hex(b) for b in resp])

    if len(resp) < 5:
        return None

    start = -1
    for i in range(len(resp) - 1):
        if resp[i] == 0x55 and resp[i + 1] == 0x55:
            start = i
            break

    if start < 0 or len(resp) < start + 5:
        return None

    pkt = resp[start:]
    pkt_len = pkt[2]
    if len(pkt) < pkt_len + 2:
        return None

    pkt = pkt[:pkt_len + 2]

    if pkt[3] != LOBOT_CMD_MULT_SERVO_POS_READ:
        return None

    returned_count = pkt[4]
    if len(pkt) != returned_count * 3 + 5:
        return None

    angles = {}
    idx = 5
    for _ in range(returned_count):
        sid = pkt[idx]
        pos = int.from_bytes(pkt[idx + 1:idx + 3], byteorder='little', signed=False)
        angles[sid] = pos
        idx += 3

    return angles

