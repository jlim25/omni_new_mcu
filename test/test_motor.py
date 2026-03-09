import cantools
import can

db = cantools.database.load_file('../app/common/omni_robot.dbc')

with can.Bus() as bus:
    msg = db.get_message_by_name('RPi_Command_2')
    data = msg.encode({'TargetAngle_deg': 90.0, 'MoveDuration_ms': 1000,
                    'TorqueEnable': 1, 'StopCmd': 0})
    bus.send(can.Message(arbitration_id=msg.frame_id, data=data))
    print("hi")