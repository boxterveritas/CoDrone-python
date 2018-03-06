from operator import eq
from queue import Queue
from threading import RLock
from threading import Thread
from time import sleep

import colorama
from colorama import Fore, Back, Style
import serial
from serial.tools.list_ports import comports
from receiver import *
from storage import *
from data import *


def convertByteArrayToString(dataArray):
    if dataArray == None:
        return ""

    string = ""

    if (isinstance(dataArray, bytes)) or (isinstance(dataArray, bytearray)) or (not isinstance(dataArray, list)):
        for data in dataArray:
            string += "{0:02X} ".format(data)

    return string


class CoDrone:
    ### BaseFunctions -------- Start

    def __init__(self, flagCheckBackground=True, flagShowErrorMessage=False, flagShowLogMessage=False,
                 flagShowTransferData=False, flagShowReceiveData=False):

        self._serialport = None
        self._bufferQueue = Queue(4096)
        self._bufferHandler = bytearray()
        self._index = 0

        # thread
        self._thread = None
        self._lock = RLock()
        self._lockState = None
        self._lockReciving = None
        self._flagThreadRun = False

        self._receiver = Receiver()
        self._control = Control()

        self._flagCheckBackground = flagCheckBackground
        self._flagShowErrorMessage = flagShowErrorMessage
        self._flagShowLogMessage = flagShowLogMessage
        self._flagShowTransferData = flagShowTransferData
        self._flagShowReceiveData = flagShowReceiveData

        self._eventHandler = EventHandler()

        self._storageHeader = StorageHeader()
        self._storage = Storage()
        self._storageCount = StorageCount()
        self._parser = Parser()

        self._devices = []  # when using auto connect, save search list
        self._flagDiscover = False  # when using auto connect, notice is discover
        self._flagConnected = False  # when using auto connect, notice connection with device
        self.timeStartProgram = time.time()  # record program starting time

        # Data
        self._timer = Timer()
        self._data = Data(self._timer)
        self._setAllEventHandler()

        # Parameter
        self._lowBatteryPercent = 30    # when the program starts, battery alert percentage
        self._controlSleep = 1     # at the end of the control

        # LED
        self._LEDColor = [255, 0, 0]
        self._LEDArmMode = LightModeDrone.ArmHold
        self._LEDEyeMode = LightModeDrone.EyeHold
        self._LEDInterval = 100
        colorama.init()

    def __del__(self):
        self.close()

    def _receiving(self, lock, lockState):
        self._lockReciving = RLock()
        while self._flagThreadRun:
            with lock and lockState and self._lockReciving:
                self._bufferQueue.put(self._serialport.read())

            # auto-update when background check for receive data is on
            if self._flagCheckBackground:
                while self.check() != DataType.None_:
                    pass
                    # sleep(0.001)

    def _sendRequestState(self, lock):
        self._lockState = RLock()
        while self._flagThreadRun:
            if self._flagConnected:
                with lock and self._lockState:
                    self.sendRequest(DataType.State)
                    sleep(0.01)
            sleep(3)

    # Decorator
    def lockState(func):
        def wrapper(self, *args, **kwargs):
            with self._lockState:
                return func(self, *args, **kwargs)
        return wrapper

    def isOpen(self):
        if self._serialport is not None:
            return self._serialport.isOpen()
        else:
            return False

    def isConnected(self):
        if not self.isOpen():
            return False
        else:
            return self._flagConnected

    def open(self, portName="None"):
        if eq(portName, "None"):
            nodes = comports()
            size = len(nodes)
            if size > 0:
                portName = nodes[size - 1].device
            else:
                return False

        self._serialport = serial.Serial(
            port=portName,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0)

        if self.isOpen():
            self._flagThreadRun = True
            self._threadSendState = Thread(target=self._sendRequestState, args=(self._lock,), daemon=True).start()
            self._thread = Thread(target=self._receiving, args=(self._lock, self._lockState,), daemon=True).start()

            # print log
            self._printLog("Connected.({0})".format(portName))
            return True
        else:
            # print error message
            self._printError("Could not connect to PETRONE LINK.")
            return False

    def close(self):
        # print log
        if self.isOpen():
            self._printLog("Closing serial port.")

        if self._flagThreadRun:
            self._flagThreadRun = False
            sleep(0.01)

        if self._thread is not None:
            self._thread.join()

        while self.isOpen():
            self._serialport.close()
            sleep(0.01)

    def makeTransferDataArray(self, header, data):
        if (header is None) or (data is None):
            return None

        if (not isinstance(header, Header)) or (not isinstance(data, ISerializable)):
            return None

        crc16 = CRC16.calc(header.toArray(), 0)
        crc16 = CRC16.calc(data.toArray(), crc16)

        dataArray = bytearray()
        dataArray.extend((0x0A, 0x55))
        dataArray.extend(header.toArray())
        dataArray.extend(data.toArray())
        dataArray.extend(pack('H', crc16))

        return dataArray

    def transfer(self, header, data):
        if not self.isOpen():
            return
        dataArray = self.makeTransferDataArray(header, data)
        with self._lockReciving and self._lock and self._lockState:
            self._serialport.write(dataArray)

        # print transfer data
        self._printTransferData(dataArray)
        return dataArray

    def check(self):
        while not self._bufferQueue.empty():
            dataArray = self._bufferQueue.get_nowait()
            self._bufferQueue.task_done()

            if (dataArray is not None) and (len(dataArray) > 0):
                # print receive data
                self._printReceiveData(dataArray)
                self._bufferHandler.extend(dataArray)

        while len(self._bufferHandler) > 0:
            stateLoading = self._receiver.call(self._bufferHandler.pop(0))

            # print error
            if stateLoading == StateLoading.Failure:
                # print receive data
                self._printReceiveDataEnd()

                # print error
                self._printError(self._receiver.message)

            # print log
            if stateLoading == StateLoading.Loaded:
                # print receive data
                self._printReceiveDataEnd()

                # print log
                self._printLog(self._receiver.message)

            if self._receiver.state == StateLoading.Loaded:
                self._handler(self._receiver.header, self._receiver.data)
                return self._receiver.header.dataType

        return DataType.None_

    def checkDetail(self):
        while not self._bufferQueue.empty():
            dataArray = self._bufferQueue.get_nowait()
            self._bufferQueue.task_done()

            if (dataArray is not None) and (len(dataArray) > 0):
                # print receive data
                self._printReceiveData(dataArray)

                self._bufferHandler.extend(dataArray)

        while len(self._bufferHandler) > 0:
            stateLoading = self._receiver.call(self._bufferHandler.pop(0))

            # print error
            if stateLoading == StateLoading.Failure:
                # print receive data
                self._printReceiveDataEnd()

                # print error
                self._printError(self._receiver.message)

            # print log
            if stateLoading == StateLoading.Loaded:
                # print receive data
                self._printReceiveDataEnd()

                # print log
                self._printLog(self._receiver.message)

            if self._receiver.state == StateLoading.Loaded:
                self._handler(self._receiver.header, self._receiver.data)
                return self._receiver.header, self._receiver.data

        return None, None

    def _handler(self, header, dataArray):

        # save input data
        message = self._runHandler(header, dataArray)

        # print error
        if (message != None):
            self._printError(message)

        # run callback event
        self._runEventHandler(header.dataType)

        # count number of request
        self._storageCount.d[header.dataType] += 1

        # process LinkEvent separately(event check like connect or disconnect)
        if (header.dataType == DataType.LinkEvent) and (self._storage.d[DataType.LinkEvent] != None):
            self._eventLinkEvent(self._storage.d[DataType.LinkEvent])

        # process LinkEventAddress separately(event check like connect or disconnect)
        if (header.dataType == DataType.LinkEventAddress) and (self._storage.d[DataType.LinkEventAddress] != None):
            self._eventLinkEventAddress(self._storage.d[DataType.LinkEventAddress])

        # process LinkDiscoveredDevice separately(add list of searched device)
        if (header.dataType == DataType.LinkDiscoveredDevice) and (
                self._storage.d[DataType.LinkDiscoveredDevice] is not None):
            self._eventLinkDiscoveredDevice(self._storage.d[DataType.LinkDiscoveredDevice])

        # complete data process
        self._receiver.checked()

        return header.dataType

    def _runHandler(self, header, dataArray):
        if self._parser.d[header.dataType] is not None:
            self._storageHeader.d[header.dataType] = header
            self._storage.d[header.dataType] = self._parser.d[header.dataType](dataArray)

    def _runEventHandler(self, dataType):
        if (isinstance(dataType, DataType)) and (self._eventHandler.d[dataType] is not None) and (
                self._storage.d[dataType] is not None):
            return self._eventHandler.d[dataType](self._storage.d[dataType])
        else:
            return None

    def _setAllEventHandler(self):
        self._eventHandler.d[DataType.Address] = self._data.eventUpdateAddress
        self._eventHandler.d[DataType.Attitude] = self._data.eventUpdateAttitude
        self._eventHandler.d[DataType.Battery] = self._data.eventUpdateBattery
        self._eventHandler.d[DataType.Pressure] = self._data.eventUpdatePressure
        self._eventHandler.d[DataType.Range] = self._data.eventUpdateRange
        self._eventHandler.d[DataType.State] = self._data.eventUpdateState
        self._eventHandler.d[DataType.Imu] = self._data.eventUpdateImu
        self._eventHandler.d[DataType.TrimFlight] = self._data.eventUpdateTrim
        self._eventHandler.d[DataType.ImageFlow] = self._data.eventUpdateImageFlow
        self._eventHandler.d[DataType.Ack] = self._data.eventUpdateAck

    def setEventHandler(self, dataType, eventHandler):
        if (not isinstance(dataType, DataType)):
            return

        self._eventHandler.d[dataType] = eventHandler

    def getHeader(self, dataType):
        if (not isinstance(dataType, DataType)):
            return None

        return self._storageHeader.d[dataType]

    def getData(self, dataType):
        if (not isinstance(dataType, DataType)):
            return None

        return self._storage.d[dataType]

    def getCount(self, dataType):

        if (not isinstance(dataType, DataType)):
            return None

        return self._storageCount.d[dataType]

    def _eventLinkHandler(self, eventLink):
        if eventLink == EventLink.Scanning:
            self._devices.clear()
            self._flagDiscover = True

        elif eventLink == EventLink.ScanStop:
            self._flagDiscover = False

        elif eventLink == EventLink.Connected:
            self._flagConnected = True

        elif eventLink == EventLink.Disconnected:
            self._flagConnected = False

        # print log
        self._printLog(eventLink)

    def _eventLinkEvent(self, data):
        self._eventLinkHandler(data.eventLink)

    def _eventLinkEventAddress(self, data):
        self._eventLinkHandler(data.eventLink)

    def _eventLinkDiscoveredDevice(self, data):
        self._devices.append(data)

        # print log
        self._printLog(
            "LinkDiscoveredDevice / {0} / {1} / {2} / {3}".format(data.index, convertByteArrayToString(data.address),
                                                                  data.name, data.rssi))

    def connect(self, portName="None", deviceName="None", flagSystemReset=False):

        # case for serial port is None(connect to last connection)
        if not self.isOpen():
            self.close()
            self.open(portName)
            sleep(0.1)

        # if not connect with serial port print error and return
        if not self.isOpen():
            # print error
            self._printError("Could not connect to PETRONE LINK.")
            return False

        # system reset
        if flagSystemReset:
            self.sendLinkSystemReset()
            sleep(3)

        # ModeLinkBroadcast.Passive mode change
        self.sendLinkModeBroadcast(ModeLinkBroadcast.Passive)
        sleep(0.1)

        # start searching device
        self._devices.clear()
        self._flagDiscover = True
        self.sendLinkDiscoverStart()

        # wait for 5sec
        for i in range(50):
            sleep(0.1)
            if not self._flagDiscover:
                break

        sleep(2)

        length = len(self._devices)

        if eq(deviceName, "None"):
            # If not specify a name, connect to the nearest device
            if length > 0:
                closestDevice = self._devices[0]

                # If more than two device is found, select the closest device
                if len(self._devices) > 1:
                    for i in range(len(self._devices)):
                        if closestDevice.rssi < self._devices[i].rssi:
                            closestDevice = self._devices[i]

                # connect the device
                self._flagConnected = False
                self.sendLinkConnect(closestDevice.index)

                # wait for 5 seconds to connect the device
                for i in range(50):
                    sleep(0.1)
                    if self._flagConnected:
                        break
                sleep(1.2)

            else:
                self._printError("Could not find PETRONE.")

        else:
            # check the name of connected device
            targetDevice = None

            if (len(self._devices) > 0):
                if (len(deviceName) == 12):
                    for i in range(len(self._devices)):

                        if (len(self._devices[i].name) > 12) and (deviceName == self._devices[i].name[0:12]):
                            targetDevice = self._devices[i]
                            break

                    if targetDevice != None:
                        # if find the device, connect the device
                        self._flagConnected = False
                        self.sendLinkConnect(targetDevice.index)

                        # wait for 5 seconds to connect the device
                        for i in range(50):
                            sleep(0.1)
                            if self._flagConnected:
                                break

                        # connect and wait another 1.2 seconds.
                        sleep(1.2)

                    else:
                        self._printError("Could not find " + deviceName + ".")

                else:
                    self._printError("Device name length error(" + deviceName + ").")

            else:
                self._printError("Could not find PETRONE.")

        ## TO DO
        ## How to alert low battery
        if self._flagConnected:
            battery = self.getBatteryPercentage()
            print("Drone battery : [{}]".format(battery))
            if battery < self._lowBatteryPercent:
                print("Low Battery!!")

        return self._flagConnected

    def _printLog(self, message):

        if self._flagShowLogMessage and message is not None:
            print(Fore.GREEN + "[{0:10.03f}] {1}".format((time.time() - self.timeStartProgram),
                                                         message) + Style.RESET_ALL)

    def _printError(self, message):
        if self._flagShowErrorMessage and message is not None:
            print(
                Fore.RED + "[{0:10.03f}] {1}".format((time.time() - self.timeStartProgram), message) + Style.RESET_ALL)

    def _printTransferData(self, dataArray):
        if (self._flagShowTransferData) and (dataArray != None) and (len(dataArray) > 0):
            print(Back.YELLOW + Fore.BLACK + convertByteArrayToString(dataArray) + Style.RESET_ALL)

    def _printReceiveData(self, dataArray):

        if (self._flagShowReceiveData) and (dataArray != None) and (len(dataArray) > 0):
            print(Back.CYAN + Fore.BLACK + convertByteArrayToString(dataArray) + Style.RESET_ALL, end='')

    def _printReceiveDataEnd(self):

        if self._flagShowReceiveData:
            print("")

    ### BaseFunctions -------- End


    ### COMMON -------- Start

    def sendPing(self):
        header = Header()

        header.dataType = DataType.Ping
        header.length = Ping.getSize()

        data = Ping()

        data.systemTime = 0

        return self.transfer(header, data)

    def sendRequest(self, dataType):
        if not isinstance(dataType, DataType):
            return None

        header = Header()

        header.dataType = DataType.Request
        header.length = Request.getSize()

        data = Request()

        data.dataType = dataType
        return self.transfer(header, data)

    ### COMMON -------- End


    ### CONTROL -------- START

    def sendControl(self, roll, pitch, yaw, throttle):
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll, pitch, yaw, throttle)

        self.transfer(header, control)

    @lockState
    def sendControlDuration(self, roll, pitch, yaw, throttle, duration):
        if duration == 0:
            return self.sendControl(roll, pitch, yaw, throttle)

        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll, pitch, yaw, throttle)

        self.transfer(header, control)

        timeStart = time.time()
        while (time.time() - timeStart) < duration:
            self.transfer(header, control)
            sleep(0.02)

        self.hover(self._controlSleep)

    ### CONTROL -------- End

    ### FLIGHT VARIABLES -------- START

    def setRoll(self, power):
        self._control.roll = power

    def setPitch(self, power):
        self._control.pitch = power

    def setYaw(self, power):
        self._control.yaw = power

    def setThrottle(self, power):
        self._control.throttle = power

    def getRoll(self):
        return self._control.roll

    def getPitch(self):
        return self._control.pitch

    def getYaw(self):
        return self._control.yaw

    def getThrottle(self):
        return self._control.throttle

    def trim(self, roll, pitch, yaw, throttle):
        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()
        data.setAll(roll, pitch, yaw, throttle)

        self.transfer(header, data)

    def resetTrim(self, power):
        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()
        data.setAll(0, 0, 0, power)

        self.transfer(header, data)

    ### FLIGHT VARIABLES -------- END

    ### FLIGHT COMMANDS (START/STOP) -------- START

    def takeoff(self):
        self._data.takeoffFuncFlag = 1  # Event States

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.TakeOff.value

        self.transfer(header, data)
        sleep(3)

    def land(self):
        self._control.setAll(0, 0, 0, 0)

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.Landing.value

        self.transfer(header, data)
        sleep(self._controlSleep)

    def hover(self, duration=0):
        timeStart = time.time()
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(0, 0, 0, 0)

        while (time.time() - timeStart) < duration:
            self.transfer(header, control)
            sleep(0.1)

        self.transfer(header, control)
        sleep(self._controlSleep)

    def emergencyStop(self):
        # Event states
        self._data.stopFuncFlag = 1

        self._control.setAll(0, 0, 0, 0)

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Stop
        data.option = 0

        self.transfer(header, data)

    ### FLIGHT COMMANDS (START/STOP) -------- END


    ### FLIGHT COMMANDS (MOVEMENT) -------- START

    def move(self, duration=None, roll=None, pitch=None, yaw=None, throttle=None):
        # move()
        if duration is None:
            self.sendControl(*self._control.getAll())
            sleep(self._controlSleep)

        # move(duration)
        elif roll is None:
            self.sendControlDuration(*self._control.getAll(), duration)

        # move(duration, roll, pitch, yaw, throttle)
        else:
            self.sendControlDuration(roll, pitch, yaw, throttle, duration)

    def go(self, direction, duration=0.5, power=50):
        # string matching : forward/backward , right/left, up/down
        pitch = ((direction == Direction.Forward) - (direction == Direction.Backward)) * power
        roll = ((direction == Direction.Right) - (direction == Direction.Left)) * power
        yaw = 0
        throttle = ((direction == Direction.Up) - (direction == Direction.Down)) * power

        self.sendControlDuration(roll, pitch, yaw, throttle, duration)

    def turn(self, direction, duration=None, power=50):
        yaw = ((direction == Direction.Right) - (direction == Direction.Left)) * power
        if duration is None:
            self.sendControl(0, 0, yaw, 0)
        else:
            self.sendControlDuration(0, 0, yaw, 0, duration)

    @lockState
    def turnDegree(self, direction, degree):
        if not isinstance(direction, Direction) or not isinstance(degree, Degree):
            return None

        power = 20
        bias = 3

        yawPast = self.getAngularSpeed().Yaw
        direction = ((direction == Direction.Right) - (direction == Direction.Left))  # right = 1 / left = -1
        degreeGoal = direction * (degree.value - bias) + yawPast

        start_time = time.time()
        while (time.time() - start_time) < degree.value / 3:
            yaw = self._data.attitude.Yaw  # Receive attitude data every time you send a flight command
            if abs(yawPast - yaw) > 180:  # When the sign changes
                degreeGoal -= direction * 360
            yawPast = yaw
            if direction > 0 and degreeGoal > yaw:  # Clockwise
                self.sendControl(0, 0, power, 0)
            elif direction < 0 and degreeGoal < yaw:  # Counterclockwise
                self.sendControl(0, 0, -power, 0)
            else:
                break
            sleep(0.05)

        self.sendControl(0, 0, 0, 0)
        sleep(self._controlSleep)

    ## TEST
    def rotate180(self):
        power = 20
        bias = 3

        yawPast = self.getAngularSpeed().Yaw
        degreeGoal = yawPast - bias

        start_time = time.time()
        while (time.time() - start_time) < 60:
            yaw = self._data.attitude.Yaw  # Receive attitude data every time you send a flight command
            if abs(yawPast - yaw) > 180:  # When the sign changes
                degreeGoal -= 360
            yawPast = yaw
            if degreeGoal > yaw:  # Clockwise
                self.sendControl(0, 0, power, 0)
            else:
                break
            sleep(0.05)

    @lockState
    def goToHeight(self, height):
        power = 30
        interval = 20  # height - 10 ~ height + 10

        start_time = time.time()
        while time.time() - start_time < 100:
            state = self.getHeight()

            differ = height - state
            if differ > interval:   # Up
                self.sendControl(0, 0, 0, power)
                sleep(0.1)
            elif differ < -interval:    # Down
                self.sendControl(0, 0, 0, -power)
                sleep(0.1)
            else:
                break

        self.sendControl(0, 0, 0, 0)
        sleep(self._controlSleep)

    ### FLIGHT COMMANDS (MOVEMENT) -------- END


    ### SENSORS -------- START

    @lockState
    def _getDataWhile(self, dataType, timer=None):
        timeStart = time.time()

        if timer is not None:
            if timer[0] > (timeStart - timer[1]):
                return False

        recieveFlag = self._storageCount.d[dataType]
        self.sendRequest(dataType)

        # Break the loop if request time is over 0.15sec, send the request maximum 2 times
        resendFlag = 1
        while self._storageCount.d[dataType] == recieveFlag:
            interval = time.time() - timeStart
            if interval > 0.03 * resendFlag and resendFlag < 3:
                self.sendRequest(dataType)
                resendFlag += 1
            elif interval > 0.15:
                break
            sleep(0.01)
        return self._storageCount.d[dataType] > recieveFlag

    def getHeight(self):
        self._getDataWhile(DataType.Range, self._timer.range)
        return self._data.range

    def getPressure(self):
        self._getDataWhile(DataType.Pressure, self._timer.pressure)
        return self._data.pressure

    def getDroneTemp(self):
        self._getDataWhile(DataType.Pressure, self._timer.pressure)
        return self._data.temperature

    def getAngularSpeed(self):
        self._getDataWhile(DataType.Attitude, self._timer.attitude)
        return self._data.attitude

    def getGyroAngles(self):
        if self._getDataWhile(DataType.Imu, self._timer.imu):
            self._timer.imu[1] = time.time()
        return self._data.gyro

    def getAccelerometer(self):
        self._getDataWhile(DataType.Imu, self._timer.imu)
        return self._data.accel

    def getOptFlowPosition(self):
        self._getDataWhile(DataType.ImageFlow, self._timer.imageFlow)
        return self._data.imageFlow

    def getState(self):
        self._getDataWhile(DataType.State, self._timer.state)
        return self._data.state

    def getBatteryPercentage(self):
        self._getDataWhile(DataType.Battery, self._timer.battery)
        return self._data.batteryPercent

    def getBatteryVoltage(self):
        self._getDataWhile(DataType.Battery, self._timer.battery)
        return self._data.batteryVoltage

    def getTrim(self):
        self._getDataWhile(DataType.TrimFlight, self._timer.trim)
        return self._data.trim

    ### SENSORS -------- END


    ### STATUS CHECKERS -------- START
    def isUpsideDown(self):
        return self._data.reversed != SensorOrientation.Normal

    def isFlying(self):
        return self._data.state == ModeFlight.Flight

    def isReadyToFly(self):
        return self._data.state == ModeFlight.Ready

    ### STATUS CHECKERS -------- END


    ### EVENT STATES -------- START

    def onUpsideDown(self, func):
        self._data.upsideDown = func

    def onTakeoff(self, func):
        self._data.takeoff = func

    def onFlying(self, func):
        self._data.flying = func

    def onReady(self, func):
        self._data.ready = func

    def onEmergencyStop(self, func):
        self._data.emergencyStop = func

    def onCrash(self, func):
        self._data.crash = func

    def onLowBattery(self, func):
        self._data.lowBattery = func

    ### EVENT STATES -------- END


    ### LEDS -------- START

    # check response after requesting data
    @lockState
    def _checkAck(self, header, data, dataType):
        self._data.ack.dataType = 0
        flag = 1

        self.transfer(header, data)
        startTime = time.time()
        while self._data.ack.dataType != dataType:
            interval = time.time() - startTime
            # Break the loop if request time is over 0.3sec, send the request maximum 2 times
            if interval > 0.06 * flag and flag < 3:
                self.transfer(header, data)
                flag += 1
            elif interval > 0.3:
                break
            sleep(0.01)
        return self._data.ack.dataType == dataType

    def setArmRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self._LEDArmMode
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = self._LEDInterval
        self._LEDColor = [red, green, blue]

        self._checkAck(header, data, DataType.LightModeColor)

    def setEyeRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self._LEDEyeMode
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = self._LEDInterval
        self._LEDColor = [red, green, blue]

        self._checkAck(header,data,DataType.LightModeColor)

    def setAllRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        ## TO DO
        ## LightModeColor2 is not working
        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self._LEDEyeMode
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = self._LEDInterval
        self._LEDColor = [red, green, blue]

        self._checkAck(header, data, DataType.LightModeColor)

        data.mode = self._LEDArmMode
        self._checkAck(header, data, DataType.LightModeColor)

    def setArmDefaultRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self._LEDArmMode
        self._LEDColor = [red, green, blue]
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = self._LEDInterval

        self._checkAck(header, data, DataType.LightModeDefaultColor)

    def setEyeDefaultRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self._LEDEyeMode
        self._LEDColor = [red, green, blue]
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = self._LEDInterval

        self._checkAck(header, data, DataType.LightModeDefaultColor)

    def resetDefaultLED(self):
        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = LightModeDrone.EyeHold
        data.color.r = 255
        data.color.g = 0
        data.color.b = 0
        data.interval = self._LEDInterval

        self._checkAck(header, data, DataType.LightModeDefaultColor)

        data.mode = LightModeDrone.ArmHold
        self._checkAck(header, data, DataType.LightModeDefaultColor)

    def setEyeMode(self, mode):
        # EYE doesn't have flow mode
        if not isinstance(mode, Mode) or mode.value > Mode.Pulsing.value:
            return None

        self._LEDEyeMode = mode

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self._LEDEyeMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        return self._checkAck(header, data, DataType.LightModeColor)

    def setArmMode(self, mode):
        if not isinstance(mode, Mode):
            return None

        self._LEDArmMode = LightModeDrone(mode.value + 0x30)

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self._LEDArmMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        self._checkAck(header, data, DataType.LightModeColor)

    def setEyeDefaultMode(self, mode):
        # EYE doesn't have flow mode
        if not isinstance(mode, Mode) or mode.value > Mode.Pulsing.value:
            return None

        self._LEDEyeMode = mode

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self._LEDEyeMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        self._checkAck(header, data, DataType.LightModeDefaultColor)

    def setArmDefaultMode(self, mode):
        if not isinstance(mode, Mode):
            return None

        self._LEDArmMode = LightModeDrone(mode.value + 0x30)

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self._LEDArmMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        self._checkAck(header, data, DataType.LightModeDefaultColor)

    ### LEDS --------- END


    ### Link -------- Start

    def sendLinkModeBroadcast(self, modeLinkBroadcast):

        if (not isinstance(modeLinkBroadcast, ModeLinkBroadcast)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkModeBroadcast
        data.option = modeLinkBroadcast.value

        return self.transfer(header, data)

    def sendLinkSystemReset(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkSystemReset
        data.option = 0

        return self.transfer(header, data)

    def sendLinkDiscoverStart(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDiscoverStart
        data.option = 0

        return self.transfer(header, data)

    def sendLinkDiscoverStop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDiscoverStop
        data.option = 0

        return self.transfer(header, data)

    def sendLinkConnect(self, index):

        if (not isinstance(index, int)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkConnect
        data.option = index

        return self.transfer(header, data)

    def sendLinkDisconnect(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDisconnect
        data.option = 0

        return self.transfer(header, data)

    def sendLinkRssiPollingStart(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkRssiPollingStart
        data.option = 0

        return self.transfer(header, data)

    def sendLinkRssiPollingStop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkRssiPollingStop
        data.option = 0

        return self.transfer(header, data)

    ### LINK -------- END

