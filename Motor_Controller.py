import time
import threading
from buildhat import Motor

class MotorController:
    # Motor controller constructor
    def __init__(self, tPort, targetSpeed=360, kp=0.03, ki=0.01, basePower=35, direction=1, interval=0.05, wheelRadius = 5):
        #kp = 0.5 and ki = 0.2 seems to be best
        # Initialize motor
        self.motor = Motor(tPort)

        # Target speed in degrees per second
        self.targetSpeed = abs(targetSpeed)

        # PID controller constants
        self.kp = kp
        self.ki = ki
        self.basePower = basePower
        self.direction = direction
        self.interval = interval

        # Controller state
        self.integral = 0.0
        self.power = basePower
        self.maxPower = 100.0
        self.maxIntegral = 2000.0

        # Threading presets
        self.running = False
        self.thread = None

        # Port that the motor runs on
        self.port = tPort

        # Wheel radius
        self.wRadius = wheelRadius

        # Use degrees per second for speed readings
        self.motor.set_speed_unit_rpm(False)

    def setTargetSpeed(self, speed):
        # Change the target speed units to degrees per second.
        self.targetSpeed = abs(speed)

    def start(self):
        if self.running:
            return

        self.running = True
        self.integral = 0.0
        self.power = self.basePower

        self.motor.pwm(self.direction * self.power / 100.0)

        self.thread = threading.Thread(target = self.controlSpeed, daemon = True, name = f'Motor {self.port}')
        self.thread.start()

    def controlSpeed(self):
        try:
            while self.running:
                loopStart = time.monotonic()

                # The acutal speed of the motor * direction to account for the motor being mounted backwards if necessary
                actualSpeed = (self.motor.get_speed() * self.direction)

                # The error between the target and actual speed
                error = self.targetSpeed - actualSpeed

                # Integral correction to make sure we are not compunding the error
                self.integral += error * self.interval
                self.integral = max(-self.maxIntegral,min(self.maxIntegral, self.integral))

                # PI controller
                self.power = (self.basePower + self.kp * error + self.ki * self.integral)
                # Keeps the power supplied between 0 and the max power set
                self.power = max(0.0, min(self.maxPower, self.power))

                # Set the adjusted power to the motor
                self.motor.pwm(self.direction * self.power / 100.0)

                # Apply the power over a certain period of time and then wait for the next loop
                timeElapsed = time.monotonic() - loopStart
                time.sleep(max(0.0, self.interval - timeElapsed))

                print(
                    f"Target={self.targetSpeed:.1f}, "
                    f"Actual={actualSpeed:.1f}, "
                    f"Error={error:.1f}, "
                    f"Power={self.power:.1f}%"
                )
        finally:
            self.motor.stop()

    # Return the current speed of the motor with direction
    def getSpeed(self):
        return self.motor.get_speed() * self.direction
    
    # Return the current power to the motor which is a percentage
    def getPower(self):
        return self.power

    # Return absolute encoder position
    def getAbsolutePosition(self):
        return self.motor.get_aposition()

    # Return relative encoder position
    def getRelativePosition(self):
        return self.motor.get_position()

    # Return wheel radius
    def getWheelRadius(self):
        return self.wRadius

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

    