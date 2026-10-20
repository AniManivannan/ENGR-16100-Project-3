import time
import threading
from buildhat import Motor

class MotorController:
    # Motor controller constructor
    def __init__(self, port, targetSpeeed=360, kp=0.03, ki=0.01, basePower=35, direction=1, interval=0.05):
        # Initialize motor
        self.motor = Motor(port)

        # Target speed in degrees per second
        self.targetSpeeed = abs(targetSpeeed)

        # PID controller constants
        self.kp = kp
        self.ki = ki
        self.basePower = basePower
        self.direction = direction
        self.interval = interval

        # Controller state
        self.integral = 0.0
        self.power = basePower
        self.max_power = 100.0
        self.max_integral = 2000.0

        # Threading presets
        self.running = False
        self.thread = None

        # Use degrees per second for speed readings
        self.motor.set_speed_unit_rpm(False)

    def setTargetSpeed(self, speed):
        # Change the target speed units to degrees per second.
        self.targetSpeeed = abs(speed)

    def start(self):
        if self.running:
            return

        self.running = True
        self.integral = 0.0
        self.power = self.basePower

        self.motor.pwm(self.direction * self.power / 100.0)

        self.thread = threading.Thread(target=self.controlSpeed, daemon=True)
        self.thread.start()

    def controlSpeed(self):
        try:
            while self.running:
                loop_start = time.monotonic()

                # The acutal speed of the motor * direction to account for the motor being mounted backwards if necessary
                actual_speed = (self.motor.get_speed() * self.direction)

                # The error between the target and actual speed
                error = self.targetSpeeed - actual_speed

                # Integral correction to make sure we are not compunding the error
                self.integral += error * self.interval
                self.integral = max(-self.max_integral,min(self.max_integral, self.integral))

                # PI controller
                self.power = (self.basePower + self.kp * error + self.ki * self.integral)
                # Keeps the power supplied between 0 and the max power set
                self.power = max(0.0, min(self.max_power, self.power))

                # Set the adjusted power to the motor
                self.motor.pwm(self.direction * self.power / 100.0)

                # Apply the power over a certain period of time and then wait for the next loop
                timeElapsed = time.monotonic() - loop_start
                time.sleep(max(0.0, self.interval - timeElapsed))

        finally:
            self.motor.stop()

    # Return the current speed of the motor with direction
    def getSpeed(self):
        return self.motor.get_speed() * self.direction
    
    # Return the current power to the motor which is a percentage
    def getPower(self):
        return self.power

    # Stop running the motor controller and stop the motor
    def stop(self):
        self.running = False

        if self.thread is not None:
            if self.thread is not threading.current_thread():
                self.thread.join()

        self.motor.stop()

    # Stop and close out the motor
    def close(self):
        self.stop()
        self.motor.close()

    