import time
from Motor_Controller import MotorController

# Create a motor controller for each motor
motors = [
    MotorController('A', target_speed=360),
    MotorController('B', target_speed=360),
    MotorController('C', target_speed=360),
    MotorController('D', target_speed=360)
]

try:
    # Start all motor controllers
    for motor in motors:
        motor.start()

    while True:
        # Print corrected wheel speeds
        speeds = [round(m.getSpeed(), 1) for m in motors]
        powers = [round(m.getPower(), 1) for m in motors]

        print("Speeds:", speeds, "Powers:", powers)

        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopping motors")

finally:
    for motor in motors:
        motor.close()