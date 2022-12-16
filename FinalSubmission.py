import math
from spike import PrimeHub, LightMatrix, MotorPair, Motor, MotionSensor, ColorSensor, ForceSensor, PIDController
from spike.control import Timer, wait_for_seconds
# Import the PIDController class
from spike.control import PIDController
	
# ------------------- SpikeRescue -------------
motor_pair = MotorPair('C', 'D')
motorLeft = Motor('C')
motorRight = Motor('D')
armMotor = Motor('B')
force = ForceSensor('A')
# Instantiate the sensors with the correct port names
motion_sensor = MotionSensor()
color_sensor = ColorSensor('E')
force_sensor = ForceSensor('F')
hub = PrimeHub()
timer = Timer()
timer.reset()
lm = LightMatrix()

# Create a PIDController 
pid = PIDController(kp=1.0, ki=0.1, kd=0.5)

# -------- liftArm -------------------------
# lifts Spike's effector for a set duration    

def liftArm(duration):
    armMotor.run_for_rotations(0.20)

def lowerArm(duration):
    armMotor.run_for_rotations(-0.15)

def spinSearch():
        motor_pair.move(1000, unit='in', steering=-15, speed=20)
        if force.is_pressed():
            motor_pair.stop()
            motor_pair.move(2, unit='in', steering=-100, speed=20)
            motor_pair.stop()

def rowSearch():
        motor_pair.move(100, unit='in', steering=0, speed=20)
        if force.is_pressed():
            motor_pair.move(2, unit='in', steering=-100, speed=20)
            motor_pair.move(100, unit='in', steering=0, speed=20)
            rowSearch()
    

def targetSearch():
    if force.is_pressed():
        color = color_sensor.get_color()
        if color == 'red':
            liftArm(1)
            # beep beep beep!
            hub.speaker.beep(60, 0.5)
            hub.speaker.beep(67, 0.5)
            hub.speaker.beep(60, 0.5)
        else:
            hub.speaker.beep(60, 0.5)
            rowSearch()


        

def searchAndRescue():
    targetSearch()
    spinSearch()
    rowSearch()


