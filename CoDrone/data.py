from protocol import *
import time


class EventStatesFunc:
    def __init__(self):
        self.upsideDown = None
        self.takeoff = None
        self.flying = None
        self.landing = None
        self.ready = None
        self.emergencyStop = None
        self.crash = None
        self.lowBattery = None


class Timer:
    def __init__(self):
        # [ time interval, variable to save start time ]
        self.address = [0, 0]
        self.attitude = [0.1, 0]
        self.battery = [5, 0]
        self.imu = [0, 0]
        self.pressure = [3, 0]
        self.trim = [0, 0]
        self.range = [0, 0]
        self.state = [0.1, 0]
        self.imageFlow = [0, 0]

        # Event states flag
        self.upsideDown = [5, 0]
        self.takeoff = [5, 0]
        self.flying = [10, 0]
        self.landing = [5, 0]
        self.ready = [10, 0]
        self.emergencyStop = [5, 0]
        self.crash = [3, 0]
        self.lowBattery = [10, 0]


class Data(EventStatesFunc):
    def __init__(self, timer):
        # criterion for low battery
        self._LowBatteryPercent = 50

        super().__init__()
        self.timer = timer
        self.address = 0
        self.attitude = Angle(0, 0, 0)
        self.accel = Axis(0, 0, 0)
        self.batteryPercent = 0
        self.batteryVoltage = 0
        self.gyro = Angle(0, 0, 0)
        self.imageFlow = Position(0, 0)
        self.pressure = 0
        self.reversed = 0
        self.temperature = 0
        self.trim = Flight(0, 0, 0, 0)
        self.range = 0
        self.state = 0
        self.ack = Ack()

        # Depending on using flight commend
        self.takeoffFuncFlag = 0
        self.stopFuncFlag = 0

    def eventUpdateAddress(self, data):
        self.address = data.address
        self.timer.address[1] = time.time()

    def eventUpdateAttitude(self, data):
        self.attitude = Angle(data.roll, data.pitch, data.yaw)
        self.timer.address[1] = time.time()

    def eventUpdateBattery(self, data):
        self.batteryPercent = data.batteryPercent
        self.batteryVoltage = data.voltage
        self.timer.battery[1] = time.time()

    def eventUpdateImu(self, data):
        self.accel = Axis(data.accelX, data.accelY, data.accelZ)
        self.gyro = Angle(data.gyroRoll, data.gyroPitch, data.gyroYaw)
        self.timer.imu[1] = time.time()

    def eventUpdatePressure(self, data):
        self.pressure = data.pressure
        self.temperature = data.temperature
        self.timer.pressure[1] = time.time()

    def eventUpdateRange(self, data):
        self.range = data.bottom
        self.timer.range[1] = time.time()

    def eventUpdateState_(self, data):
        self.reversed = data.sensorOrientation
        self.batteryPercent = data.battery
        self.state = data.modeFlight
        self.timer.state[1] = time.time()

    def eventUpdateState(self, data):
        self.reversed = data.sensorOrientation
        self.batteryPercent = data.battery
        self.state = data.modeFlight
        # check Event states flags
        start_time = time.time()
        # automatically checking
        if self.upsideDown is not None and self.reversed != SensorOrientation.Normal:
            if start_time - self.timer.upsideDown[1] > self.timer.upsideDown[0]:
                self.upsideDown()
                self.timer.upsideDown[1] = start_time
        if self.lowBattery is not None and self.batteryPercent < self._LowBatteryPercent:
            if start_time - self.timer.lowBattery[1] > self.timer.lowBattery[0]:
                self.lowBattery()
                self.timer.lowBattery[1] = start_time
        if self.ready is not None and self.state == ModeFlight.Ready:
            if start_time - self.timer.ready[1] > self.timer.ready[0]:
                self.ready()
                self.timer.ready[1] = start_time
                return
        if self.flying is not None and self.state == ModeFlight.Flight:
            if start_time - self.timer.flying[1] > self.timer.flying[0]:
                self.flying()
                self.timer.flying[1] = start_time
                return
        if self.landing is not None and self.state == ModeFlight.Landing:
            if start_time - self.timer.landing[1] > self.timer.landing[0]:
                self.landing()
                self.timer.landing[1] = start_time
                return
        ## TO DO
        ## How to check crash ? (ModeFlight.accident is too short time)
        if self.crash is not None and self.state == ModeFlight.Accident:
            if start_time - self.timer.crash[1] > self.timer.crash[0]:
                self.crash()
                self.timer.crash[1] = start_time
                return
        # whenever user executes flight function
        if self.takeoff is not None and self.takeoffFuncFlag:
            self.takeoff()
            self.takeoffFuncFlag = 0
            return
        if self.emergencyStop is not None and self.stopFuncFlag:
            self.emergencyStop()
            self.stopFuncFlag = 0
            return

    def eventUpdateTrim(self, data):
        self.trim = Flight(data.roll, data.pitch, data.yaw, data.throttle)
        self.timer.trim[1] = time.time()

    def eventUpdateImageFlow(self, data):
        self.imageFlow = Position(data.positionX, data.positionY)
        self.timer.imageFlow[1] = time.time()

    def eventUpdateAck(self, data):
        self.ack = data
