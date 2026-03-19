import importlib
import ServoControl
importlib.reload(ServoControl)

ServoControl.setMultiServoUnload([1,2,3,5])
print("Battery mV:", ServoControl.getBatteryVoltage())
print("Servo angles:", ServoControl.getControllerServoAngles([1, 2, 3, 5]))

