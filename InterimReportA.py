from spike import PrimeHub, LightMatrix, MotorPair, Motor
from spike.control import Timer, wait_for_seconds

import math

# ------------------- SpikeRescue -------------
# 

motor_pair = MotorPair('C', 'D')
motorLeft = Motor('C')
motorRight = Motor('D')
armMotor = Motor('B')

hub = PrimeHub()

timer = Timer()
timer.reset()
lm = LightMatrix()

# -------- driveStraight ---------------------
# drives Spike forward for a set duration

def driveStraight(duration):
    motor_pair.move(1, unit='seconds', steering=0, speed=70)

# -------- driveBackwards ---------------------
# drives Spike backwards for a set duration

def driveBackwards(duration):
        motor_pair.move(1, unit='seconds', steering=0, speed=-70)

# -------- turnRight ------------------------
# turns Spike right in place

def turnRight():
    motor_pair.move(8.8,'cm', 100, 70)

# -------- turnLeft -------------------------
# turns Spike left in place

def turnLeft():
    motor_pair.move(8.8,'cm', -100, 70)

# -------- driveRight ----------------------
# drives Spike to the right for a set duration

def driveRight(duration):
    motor_pair.move(1, unit='seconds', steering=50, speed=50)

# -------- driveLeft ----------------------
# drives Spike to the left for a set duration

def driveLeft(duration):
    motor_pair.move(1, unit='seconds', steering=-50, speed=50)

# -------- liftArm -------------------------
# lifts Spike's effector for a set duration    

def liftArm(duration):
    armMotor.run_for_rotations(0.20)

def lowerArm(duration):
    armMotor.run_for_rotations(-0.15)

# -------- armTest -------------------------
# drive Spike forward and then lifts the effector
def armTest(duration):
    while(timer.now() < duration):
        driveStraight(1)
        liftArm(1)
        driveBackwards(1)
        lowerArm(1)
        motor_pair.stop()

armTest(5)