import math
from spike import PrimeHub, LightMatrix, MotorPair, Motor, MotionSensor, ColorSensor, DistanceSensor, #PIDController
from spike.control import Timer, wait_for_seconds
# Import the PIDController class
# from spike.control import PIDController
	
# ------------------- SpikeRescue -------------
motor_pair = MotorPair('C', 'D')
motorLeft = Motor('C')
motorRight = Motor('D')
armMotor = Motor('B')

# Instantiate the sensors with the correct port names
color_sensor = ColorSensor('E')
distance_sensor = DistanceSensor('F')
hub = PrimeHub()
lm = LightMatrix()

distance_sensor.light_up_all(100)

timer = Timer()
timer.reset()

# Create a PIDController 
#pid = PIDController(kp=1.0, ki=0.1, kd=0.5)

# -------- liftArm -------------------------
# lifts Spike's effector for a set duration    

def liftArm(duration):
    armMotor.run_for_rotations(0.20)

def lowerArm(duration):
    armMotor.run_for_rotations(-0.15)

# ------- spinSearch -----------------------
# Spike will continue moving out in a spiral until
# it gets too close to a wall. It will then stop

def spinSearch():
    while True:
        distance_sensor.wait_for_distance_farther_than(5, 'in', short_range=False)
        motor_pair.move(1000, unit='in', steering=-30, speed=40)
        motor_pair.stop()
        motor_pair.move(10, unit='in', steering=-100, speed=80)
        motor_pair.stop()
        #rowSearch()


# ------- spinSearch -----------------------
# Spike will move back and forth in straight lines.
# When a wall is reached, it will turn around and begin again.

def rowSearch():
    if (timer < timer.now()):
        motor_pair.move(100, unit='in', steering=0, speed=20)
        distance_sensor.wait_for_distance_closer_than(1, 'in')
        motor_pair.move(2, unit='in', steering=-100, speed=20)
        motor_pair.move(100, unit='in', steering=0, speed=20)
        rowSearch()
        
    

# ------- targetSearch ---------------------
# Spike will check to see if the right color target is in
# front of it. If it is, it will lift it up and beep 3 times.
# Otherwise, it will beep once.

def targetSearch():
    if (timer < timer.now()):
        distance_sensor.wait_for_distance_closer_than(1, 'in')
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
        

        

# def searchAndRescue():
#     spinSearch()
#     targetSearch()
#     rowSearch()


# searchAndRescue()

spinSearch()