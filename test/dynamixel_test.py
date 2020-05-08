from dynamixels.dynamixels import *

motors = DynamixelActor()

motors.reset()

motors.step([-100, -50])

angle1= motors._angle1
angle2 = motors._angle2

ini1= angle1
ini2= angle2

try:
    for i in range(200):
        for j in range(10):
            angle2+= 10
            motors.step([angle1, angle2])
        angle2 = ini2

        for j in range(10, 0, -1):
            angle2-=10
            motors.step([angle1, angle2])

        angle2=ini2
        angle1+=10

except:
    motors.reset()

motors.reset()
