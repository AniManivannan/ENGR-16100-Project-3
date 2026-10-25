import math
import threading
import time
from basehat import IMUSensor
from Motor_Controller import MotorController

class Odometry:

    GRAVITY = 9.81
    ALPHA = 0.98

    def __init__(self, leftMotor, rightMotor, initalX = 0, initalY = 0, initalZ = 0, initalRotX = 0, initalRotY = 0, initalRotZ = 0, lInterval = 0.05, trackWidth = 3):
        #Initalize IMU
        self.imu = IMUSensor()

        #Get instances of the left and right motor
        self.lMotor = leftMotor
        self.rMotor = rightMotor

        #Scaling factor for the coordiantes
        self.leftScale = 1.2
        self.rightScale = 1

        #Wheel radius in cm
        self.wheelRadius = 5 

        #Track width, which is the distance between the center of the two wheels in cm
        self.tWidth = trackWidth

        #Previous encoder positions
        self.leftLastPosition = None
        self.rightLastPosition = None
        self.encoderHeading = math.radians(initalRotZ)        

        #World position
        self.x = initalX
        self.y = initalY
        self.z = initalZ

        #World rotation
        self.rotX = initalRotX
        self.rotY = initalRotY
        self.rotZ = initalRotZ

        #Linear acceleration 
        self.ax = 0
        self.ay = 0
        self.az = 0

        #Acceleration and gyro offsets
        self.aOffset = [0.0, 0.0, 0.0]
        self.gOffset = [0.0, 0.0, 0.0]
        self.waOffset = [0.0, 0.0, 0.0]

        #World-relative acceleration
        self.worldAx = 0.0
        self.worldAy = 0.0
        self.worldAz = 0.0

        #Linear velocity
        self.vx = 0
        self.vy = 0
        self.vz = 0

        #Rotational velocity in degrees/s
        self.rx = 0
        self.ry = 0
        self.rz = 0

        #Max interval for each loop
        self.interval = lInterval

        #Inital values for the integration
        self.lastTime = None

        #Presets for threading
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

    #Start the odometry tracking
    def start(self):
        if self.running:
            return
        self.running = True

        self.thread = threading.Thread(target = self.calcOdometry, daemon = True, name = "Odometry")
        self.thread.start()
    
    #Stop odometry tracking
    def stop(self):
        self.running = False

        if self.thread is not None:
            if self.thread is not threading.current_thread():
                self.thread.join()

    def close(self):
        self.stop()

    #Update the acceleration and rotation values from the IMU
    def getSensorValues(self):
        #Update acceleration values
        self.ax, self.ay, self.az = self.imu.getAccel()

        #Update rotational velocity values
        self.rx, self.ry, self.rz = self.imu.getGyro()

        #Account for accel offsets
        self.ax -= self.aOffset[0]
        self.ay -= self.aOffset[1]

        #Account for gyro offsets
        #self.rx -= self.gOffset[0]
        #self.ry -= self.gOffset[1]
        #self.rz -= self.gOffset[2]

    #Calculate absolute acceleration is x,y and z
    def calculateWorldAcceleration(self, ax, ay, az):
        roll = math.radians(self.rotX)
        pitch = math.radians(self.rotY)
        yaw = math.radians(self.rotZ)

        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)

        worldAx = (cy * cp * ax + (cy * sp * sr - sy * cr) * ay + (cy * sp * cr + sy * sr) * az)

        worldAy = (sy * cp * ax + (sy * sp * sr + cy * cr) * ay + (sy * sp * cr - cy * sr) * az)

        worldAz = (-sp * ax + cp * sr * ay + cp * cr * az)

        worldAz -= self.GRAVITY

        return worldAx, worldAy, worldAz

    #Calibrate the IMU to get offsets
    def calibrateIMU(self, samples = 200, delay = 0.01):
        #Acceleration samples
        aSamp = [[], [], []]
        #Gyro samples
        gSamp = [[], [], []]

        #Add samples to the lists
        for _ in range(samples):
            accel = self.imu.getAccel()
            gyro = self.imu.getGyro()

            for i in range(3):
                aSamp[i].append(accel[i])
                gSamp[i].append(gyro[i])

            time.sleep(delay)

        #Get the accelration offsets
        self.aAvg = [sum(axis) / samples for axis in aSamp]
        #Get gyro offsets
        self.gOffset = [sum(axis) / samples for axis in gSamp]

        self.aOffset = [self.aAvg[0], self.aAvg[1], 0.0]
        self.GRAVITY = self.aAvg[2]

        # Measure residual world acceleration at rest
        wSamp = [[], [], []]

        print("Calibration finished")
        time.sleep(0.01)

        for _ in range(samples):
            ax, ay, az = self.imu.getAccel()

            #ax -= self.aOffset[0]
            #ay -= self.aOffset[1]

            # Use the current orientation, initially assumed level
            wx, wy, wz = self.calculateWorldAcceleration(ax, ay, az)

            wSamp[0].append(wx)
            wSamp[1].append(wy)
            wSamp[2].append(wz)

            time.sleep(delay)

        self.worldAccelOffset = [sum(axis) / len(axis) for axis in wSamp]

        time.sleep(0.1)

    #Update x,y and z pos
    def updateOdometry(self, dt):
        #Use a complementary filter to reduce error
        accRoll  = math.degrees(math.atan2(self.ay, self.az))
        accPitch = math.degrees(math.atan2(-self.ax, math.hypot(self.ay, self.az)))

        self.rotX = self.ALPHA * (self.rotX + self.rx * dt) + (1 - self.ALPHA) * accRoll
        self.rotY = self.ALPHA * (self.rotY + self.ry * dt) + (1 - self.ALPHA) * accPitch
        #self.rotZ += self.rz * dt 

        self.worldAx, self.worldAy, self.worldAz = (self.calculateWorldAcceleration(self.ax, self.ay, self.az))

        self.worldAx -= self.worldAccelOffset[0]
        self.worldAy -= self.worldAccelOffset[1]
        self.worldAz -= self.worldAccelOffset[2]

        #Integrate to get position
        #self.x += self.vx * dt + 0.5 * self.worldAx * dt ** 2
        #self.y += self.vy * dt + 0.5 * self.worldAy * dt ** 2
        #self.z += self.vz * dt + 0.5 * self.worldAz * dt ** 2

        #Integrate and update velocity
        #self.vx += self.worldAx * dt
        #self.vy += self.worldAy * dt
        #self.vz += self.worldAz * dt

    #Update positions from motor encoders
    def updateEncoderOdometry(self, leftMotor, rightMotor):
        #Read cumulative encoder positions in degrees
        leftPosition = leftMotor.getRelativePosition()
        rightPosition = rightMotor.getRelativePosition()

        #Starting positions
        if self.leftLastPosition is None:
            self.leftLastPosition = leftPosition
            self.rightLastPosition = rightPosition
            return
        
        rawLeftDelta = leftPosition - self.leftLastPosition
        rawRightDelta = rightPosition - self.rightLastPosition

        #Change in position from previous reading
        leftDelta = rawLeftDelta
        rightDelta = -1 * rawRightDelta

        self.leftLastPosition = leftPosition
        self.rightLastPosition = rightPosition

        #Convert motor rotation into wheel distance in meters
        leftDistance = math.radians(leftDelta) * self.wheelRadius * self.leftScale
        rightDistance = math.radians(rightDelta) * self.wheelRadius * self.rightScale

        #Average distance traveled
        distance = (leftDistance + rightDistance) / 2.0

        #Change in heading
        deltaHeading = (rightDistance - leftDistance) / self.tWidth

        #Update world position using the midpoint of the turn
        midHeading = self.encoderHeading + deltaHeading / 2.0

        self.x += distance * math.cos(midHeading)
        self.y += distance * math.sin(midHeading)

        #Store the new heading
        self.encoderHeading += deltaHeading

        #Keep rotZ in degrees
        self.rotZ = math.degrees(self.encoderHeading)

        print(
                    f"Left: {leftMotor.getSpeed():7.2f} deg/s "
                    f"({leftMotor.getPower():5.1f}%) | "
                    f"Right: {rightMotor.getSpeed():7.2f} deg/s "
                    f"({rightMotor.getPower():5.1f}%)"
                )

    #Main loop to run odometry
    def calcOdometry(self):
        
        self.leftLastPosition = self.lMotor.getRelativePosition()
        self.rightLastPosition = self.rMotor.getRelativePosition()
        self.lastTime = time.monotonic()

        while self.running:
            loopStart = time.monotonic()
            dt = loopStart - self.lastTime
            self.lastTime = loopStart

            try:
                self.getSensorValues()

                if dt > 0:
                    self.updateOdometry(dt)

                self.updateEncoderOdometry(self.lMotor, self.rMotor)
                #print(f'rotz: {self.rotZ}| x = {self.x} | y = {self.y}')
        
            except IOError as e:
                print(f"IMU read error: {e}")
                continue

            timeElapsed = time.monotonic() - loopStart
            time.sleep(max(0.0, self.interval - timeElapsed))

    #Returns the current world x,y or z position
    def returnPosition(self, axis = "x"):
        match axis:
            case "x":
                return self.x
            case "y":
                return self.y
            case "z":
                return self.z
            case "a":
                return [self.x, self.y, self.z]
            case _:
                raise ValueError(f'Invalid input {axis}. Can only take x, y, z or a as an input')

    #Returns the current world x,y or z rotation       
    def returnRotation(self, axis = "x"):
        match axis:
            case "x":
                return self.rotX
            case "y":
                return self.rotY
            case "z":
                return self.rotZ
            case "a":
                return [self.rotX, self.rotY, self.rotZ]
            case _:
                raise ValueError(f'Invalid input {axis}. Can only take x, y, z or a as an input')