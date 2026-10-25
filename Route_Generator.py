import math
import threading
import time
from basehat import LineFinder

class RouteGenerator:

    def __init__(self, leftMotor, rightMotor, tSpeed, tuSpeed, routeType = "l", flp = 22, frp = 24, bp = 26, interval = 0.05):

        self.frontLeftSensor = LineFinder(flp)
        self.frontRightSensor = LineFinder(frp)
        self.backSensor = LineFinder(bp)

        self.sensorValues = [self.frontLeftSensor.value, self.frontRightSensor.value, self.backSensor.value]

        self.targetSpeed = tSpeed
        self.turnSpeed = tuSpeed

        self.rt = routeType
        self.running = False

        self.lMotor = leftMotor
        self.rMotor = rightMotor

        # Threading presets
        self.running = False
        self.thread = None

    def runRoute(self):
        try:
            if(self.r == "l"):
                loopStart = time.monotonic()

                while self.running:
                    self.updateSensorValues()
                    match self.sensorValues:
                        case [0,0,0]:
                            self.driveForward(self.targetSpeed)
                        case [0,0,1]:
                            self.driveForward(self.targetSpeed)
                        case [1,1,0]:
                            self.driveForward(self.targetSpeed)
                        case [1,0,0]:
                            self.rotate(self.turnSpeed, 1)
                        case [0,1,0]:
                            self.rotate(self.turnSpeed, -1)
                        case [1,0,1]:
                            self.rotate(self.turnSpeed, 1)
                        case [0,1,1]:
                            self.rotate(self.turnSpeed, -1)
                        case [1,1,1]:
                            self.driveForward(self.targetSpeed)
                        case _:
                            print("No line found")
                            self.stopMotors()
                    timeElapsed = time.monotonic() - loopStart
                    time.sleep(max(0.0, self.interval - timeElapsed))
        except IOError:
            print("t")
        finally:
            self.stopMotors()

    def driveForward(self, tSpeed):
        self.lMotor.setTargetSpeed(tSpeed)
        self.rMotor.setTargetSpeed(tSpeed)

    def rotate(self, tSpeed, direction):
        self.lMotor.setTargetSpeed(tSpeed * direction)
        self.rMotor.setTargetSpeed(tSpeed * -direction)

    def stopMotors(self):
        self.lMotor.setTargetSpeed(0)
        self.rMotor.setTargetSpeed(0)

    def updateSensorValues(self):
        self.sensorValues[0] = self.frontLeftSensor.value
        self.sensorValues[1] = self.frontRightSensor.value
        self.sensorValues[2] = self.backSensor.value

    #Start the routing
    def start(self):
        if self.running:
            return
        self.running = True

        self.thread = threading.Thread(target = self.runRoute, daemon = True, name = "Route Generator")
        self.thread.start()

        # Stop running the motor controller and stop the motor
    def stop(self):
        self.running = False

        if self.thread is not None:
            if self.thread is not threading.current_thread():
                self.thread.join()
                self.stopMotors()

    # Stop and close out the motor
    def close(self):
        self.stop()
        self.stopMotors()
