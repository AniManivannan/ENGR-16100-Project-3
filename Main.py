import time
import math
from Motor_Controller import MotorController
from Odom import Odometry
from Route_Generator import RouteGenerator



# Create a motor controller for each motor
a = MotorController('A', targetSpeed = 80, direction = -1, kp = 0.5, ki = 0.2)
b = MotorController('B', targetSpeed = 80, kp = 0.2, ki = 0.2)

odometry = Odometry(leftMotor = b,rightMotor = a)
odometry.calibrateIMU()

routeGen = RouteGenerator(b,a,80,40,"l")

#MotorController('C', target_speed=360),
#MotorController('D', target_speed=360)

try:
    # Start all motor controllers
    a.start()
    b.start()
    odometry.start()
    routeGen.start()

    while True:
        # Print corrected wheel speeds

        print("Speeds:")

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopping motors")
    routeGen.stop()
    a.stop()
    b.stop()
    odometry.stop()
    

finally:
    odometry.close()




