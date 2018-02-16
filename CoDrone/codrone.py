from operator import eq
from queue import Queue
from threading import Thread
from time import sleep

import colorama
import serial
from colorama import Fore, Back, Style
from serial.tools.list_ports import comports
from system import *
from protocol import *
from crc import *
from receiver import *
from storage import *

def convertByteArrayToString(dataArray):
    if dataArray == None:
        return ""

    string = ""

    if (isinstance(dataArray, bytes)) or (isinstance(dataArray, bytearray)) or (not isinstance(dataArray, list)):
        for data in dataArray:
            string += "{0:02X} ".format(data)

    return string

class EventStatesFlag:
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
        self.attitude = [0, 0]
        self.angle = [0, 0]
        self.battery = [5, -5]
        self.imu = [0, 0]
        self.pressure = [3, -3]
        self.trim = [0, 0]
        self.range = [0, 0]
        self.state = [0, 0]

        # Event states flag
        self.upsideDown = [5, -5]
        self.takeoff = [5, -5]
        self.flying = [10, -10]
        self.landing = [5, -5]
        self.ready = [10, -10]
        self.emergencyStop = [5, -5]
        self.crash = [3, -3]
        self.lowBattery = [10, -10]


class Data(EventStatesFlag):
    def __init__(self):
        super().__init__()
        self.timer = Timer()
        self.address = 0
        self.attitude = 0
        self.accel = 0
        self.angle = 0
        self.batteryPercent = 0
        self.batteryVoltage = 0
        self.gyro = 0
        self.imageFlow = 0
        self.pressure = 0
        self.reversed = 0
        self.temperature = 0
        self.trim = 0
        self.range = 0
        self.state = 0

        # Depending on using flight commend
        self.takeoffFuncFlag = 0
        self.stopFuncFlag = 0

    def eventUpdateAddress(self, data):
        self.address = data.address
    def eventUpdateAttitude(self, data):
        self.attitude = Angle(data.roll, data.pitch, data.yaw)
    def eventUpdateBattery(self, data):
        self.batteryPercent = data.batteryPercent
        self.batteryVoltage = data.voltage
    def eventUpdateImu(self, data):
        self.accel = Axis(data.accelX, data.accelY, data.accelZ)
        self.gyro = Angle(data.gyroRoll, data.gyroPitch, data.gyroYaw)
    def eventUpdatePressure(self, data):
        self.pressure = data.pressure
        self.temperature = data.temperature
    def eventUpdateRange(self, data):
        self.range = data.bottom
    def eventUpdateState_(self, data):
        self.reversed = data.sensorOrientation
        self.batteryPercent = data.battery
        self.state = data.modeFlight
    def eventUpdateState(self, data):
        self.reversed = data.sensorOrientation
        self.batteryPercent = data.battery
        self.state = data.modeFlight
        # check Event states flags
        start_time = time.time()
        # automatically checking (request in _receiving function)
        if self.upsideDown is not None and self.reversed != SensorOrientation.Normal:
            if start_time - self.timer.upsideDown[1] > self.timer.upsideDown[0]:
                self.upsideDown()
                self.timer.upsideDown[1] = start_time
        if self.lowBattery is not None and self.batteryPercent < 50:
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
        # TO DO
        # How to check crash ? (ModeFlight.accident is too short time)
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
    def eventUpdateImageFlow(self, data):
        self.imageFlow = Position(data.positionX, data.positionY)


class CoDrone:
    # BaseFunctions Start

    def __init__(self, flagCheckBackground=True, flagShowErrorMessage=False, flagShowLogMessage=False,
                 flagShowTransferData=False, flagShowReceiveData=False):

        self._serialport = None
        self._bufferQueue = Queue(4096)
        self._bufferHandler = bytearray()
        self._index = 0

        self._thread = None
        self._flagThreadRun = False

        self._receiver = Receiver()
        self._control = Control()

        self._flagCheckBackground = flagCheckBackground

        self._flagShowErrorMessage = flagShowErrorMessage
        self._flagShowLogMessage = flagShowLogMessage
        self._flagShowTransferData = flagShowTransferData
        self._flagShowReceiveData = flagShowReceiveData

        self._eventHandler = EventHandler()
        self._data = Data()
        self._timer = Timer()
        self._setAllEventHandler()

        self._storageHeader = StorageHeader()
        self._storage = Storage()
        self._storageCount = StorageCount()
        self._parser = Parser()

        self._devices = []  # when using auto connect, save search list
        self._flagDiscover = False  # when using auto connect, notice is discover
        self._flagConnected = False  # when using auto connect, notice connection with device
        self.timeStartProgram = time.time()  # record program starting time

        self.LEDColor = [255,0,0]
        self.LEDArmMode = LightModeDrone.ArmHold
        self.LEDEyeMode = LightModeDrone.EyeHold
        colorama.init()

    def __del__(self):
        self.close()

    def _receiving(self):
        start_timer = time.time()
        while self._flagThreadRun:
            if (time.time() - start_timer) > 1:
                start_timer = time.time()
                self.sendRequest(DataType.State)
            self._bufferQueue.put(self._serialport.read())

            # auto-update when background check for receive data is on
            if self._flagCheckBackground:
                while self.check() != DataType.None_:
                    pass
                    # sleep(0.001)

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

    def open(self, portName= "None"):
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

        if (self.isOpen()):
            self._flagThreadRun = True
            self._thread = Thread(target=self._receiving, args=(), daemon=True).start()

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
        if (header.dataType == DataType.LinkDiscoveredDevice) and (self._storage.d[DataType.LinkDiscoveredDevice] is not None):
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
                            break;

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


    # BaseFunctions End



    # Common Start

    def sendPing(self):

        header = Header()

        header.dataType = DataType.Ping
        header.length = Ping.getSize()

        data = Ping()

        data.systemTime = 0

        return self.transfer(header, data)

    def sendRequest(self, dataType):

        if (not isinstance(dataType, DataType)):
            return None

        header = Header()

        header.dataType = DataType.Request
        header.length = Request.getSize()

        data = Request()

        data.dataType = dataType

        return self.transfer(header, data)

    # Common End



    ### Control ------------
    def sendControl(self, roll, pitch, yaw, throttle):
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll,pitch,yaw,throttle)
        self.transfer(header, control)

    def sendControlDuration(self, roll, pitch, yaw, throttle, duration):
        if(duration == 0):
            return self.sendControl(roll,pitch, yaw,throttle)

        timeStart = time.time()
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll, pitch, yaw, throttle)

        self.transfer(header, control)
        while (time.time() - timeStart) < duration:
            self.transfer(header, control)
            sleep(0.02)
        control.setAll(0, 0, 0, 0)
        self.transfer(header, control)
    ### Control End ---------


    ### FLIGHT VARIABLES ----------

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

    ### FLIGHT VARIABLES ------------- END


    ### FLIGHT COMMANDS --------------

    def move(self, duration=None, roll=None, pitch=None, yaw=None, throttle=None):
        # move()
        if duration is None:
            self.sendControl(*self._control.getAll())

        # move(duration)
        elif throttle is None:
            self.sendControlDuration(*self._control.getAll(), duration)

        # move(duration, roll, pitch, yaw, throttle)
        else:
            self.sendControlDuration(roll,pitch,yaw,throttle, duration)

    def go(self, direction, duration = 0.5, power = 50):

        # string matching : forward/backward , right/left, up/down
        pitch = ((direction == Direction.Forward) - (direction == Direction.Backward)) * power
        roll = ((direction == Direction.Right) - (direction == Direction.Left)) * power
        yaw = 0
        throttle = ((direction == Direction.Up) - (direction == Direction.Down)) * power

        self.sendControlDuration(roll, pitch, yaw, throttle, duration)

    def turn(self, direction, duration=None, power=50):
        yaw = ((direction == Direction.Right) - (direction == Direction.Left)) * power
        if (duration is None):
            self.sendControl(0, 0, yaw, 0)
        else:
        	self.sendControlDuration(0, 0, yaw, 0, duration)

    def turnDegree(self):
        ### TO DO ###
        pass

    def rotate90(self):
        ### TO DO ###
        pass

    def rotate180(self):
        ### TO DO ###
        pass

    def rotate360(self):
        ### TO DO ###
        pass

    def goToHeight(self,height):
        ### TO DO ###
        pass

    ### FLIGHT COMMANDS -----------------------END


    ### FLIGHT EVENTS -----------------------

    def takeoff(self):
        # Event States
        self._data.takeoffFuncFlag = 1

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

    def hover(self, duration = 0):
        timeStart = time.time()
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(0, 0, 0, 0)

        self.transfer(header, control)
        while (time.time() - timeStart) < duration:
            sleep(0.02)

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

    ### FLIGHT EVENTS ------------------ END

    
    ### SENSORS & STATUS ---------------

    def getDataWhile(self, dataType):
        flag = self._storageCount.d[dataType]
        self.sendRequest(dataType)
        count = 0
        # Break the loop if request time is over 1sec
        while (self._storageCount.d[dataType] == flag) and (count < 100):
            sleep(0.01)
            count += 1
        return (self._storageCount.d[dataType] > flag)

    def getHeight(self):
        self.getDataWhile(DataType.Range)
        return self._data.range

    def getPressure(self):
        self.getDataWhile(DataType.Pressure)
        return self._data.pressure

    def getDroneTemp(self):
        self.getDataWhile(DataType.Pressure)
        return self._data.temperature

    def getAngularSpeed(self):
        self.getDataWhile(DataType.Attitude)
        return self._data.attitude

    def getGyroAngles(self):
        self.getDataWhile(DataType.Imu)
        return self._data.gyro

    def getAccelerometer(self):
        self.getDataWhile(DataType.Imu)
        return self._data.accel

    def getOptFlowPosition(self):
        self.getDataWhile(DataType.ImageFlow)
        return self._data.imageFlow

    def getState(self):
        self.getDataWhile(DataType.State)
        return self._data.state

    def getBatteryPercentage(self):
        timeStart = time.time()
        if self._timer.battery[0] < (timeStart - self._timer.battery[1]):
            self.getDataWhile(DataType.Battery)
            self._timer.battery[1] = timeStart
        return self._data.batteryPercent

    def getBatteryVoltage(self):
        timeStart = time.time()
        if(self._timer.battery[0] < timeStart - self._timer.battery[1]):
            self.getDataWhile(DataType.Battery)
            self._timer.battery[1] = timeStart
        return self._data.batteryVoltage

    def getTrim(self):
        self.getDataWhile(DataType.TrimFlight)
        return self._data.trim

    ### SENSORS & STATUS --------------- END


    ### LEDS -----------

    def setArmRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self.LEDArmMode
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = 100
        self.LEDColor = [red, green, blue]

        self.transfer(header, data)
        sleep(0.25)

    def setEyeRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self.LEDEyeMode
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = 100
        self.LEDColor = [red, green, blue]

        self.transfer(header, data)
        sleep(0.25)

    def setAllRGB(self, red, green, blue):

        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        self.setArmRGB(red, green, blue)
        self.setEyeRGB(red, green, blue)

    def setArmDefaultRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self.LEDArmMode
        self.LEDColor = [red, green, blue]
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = 100

        self.transfer(header, data)
        sleep(0.25)

    def setEyeDefaultRGB(self, red, green, blue):
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self.LEDEyeMode
        self.LEDColor = [red, green, blue]
        data.color.r = red
        data.color.g = green
        data.color.b = blue
        data.interval = 100

        self.transfer(header, data)
        sleep(0.25)

    def resetDefaultLED(self):
        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = LightModeDrone.EyeHold
        data.color.r = 255
        data.color.g = 0
        data.color.b = 0
        data.interval = 100

        self.transfer(header, data)
        sleep(0.3)

        data.mode = LightModeDrone.ArmHold
        self.transfer(header, data)
        sleep(0.25)

    def setEyeMode(self, mode):
        self._changeEyeMode(mode)

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self.LEDEyeMode
        data.color.r, data.color.g, data.color.b = self.LEDColor
        data.interval = 100

        self.transfer(header, data)
        sleep(0.25)

    def setArmMode(self, mode):
        self._changeArmMode(mode)

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self.LEDArmMode
        data.color.r, data.color.g, data.color.b = self.LEDColor
        data.interval = 100

        self.transfer(header, data)
        sleep(0.25)

    def setArmDefaultMode(self, mode):
        self._changeArmMode(mode)

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self.LEDArmMode
        data.color.r, data.color.g, data.color.b = self.LEDColor
        data.interval = 100

        self.transfer(header, data)
        sleep(0.25)

    def setEyeDefaultMode(self, mode):
        self._changeEyeMode(mode)

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self.LEDEyeMode
        data.color.r, data.color.g, data.color.b = self.LEDColor
        data.interval = 100

        self.transfer(header, data)
        sleep(0.25)

    def _changeEyeMode(self, mode):
        if mode == Mode.Pulsing:
            self.LEDEyeMode = LightModeDrone.EyeDimming
        elif mode == Mode.Blinking:
            self.LEDEyeMode = LightModeDrone.EyeFlicker
        elif mode == Mode.DoubleBlink:
            self.LEDEyeMode = LightModeDrone.EyeFlickerDouble
        elif mode == Mode.Hold:
            self.LEDEyeMode = LightModeDrone.EyeHold
        elif mode == Mode.Mix:
            self.LEDEyeMode = LightModeDrone.EyeMix
        elif mode == Mode.Off:
            self.LEDEyeMode = LightModeDrone.EyeNone
        else:
            return None

    def _changeArmMode(self, mode):
        if mode == Mode.Pulsing:
            self.LEDArmMode = LightModeDrone.ArmDimming
        elif mode == Mode.Blinking:
            self.LEDArmMode = LightModeDrone.ArmFlicker
        elif mode == Mode.DoubleBlink:
            self.LEDArmMode = LightModeDrone.ArmFlickerDouble
        elif mode == Mode.Flow:
            self.LEDArmMode = LightModeDrone.ArmFlow
        elif mode == Mode.ReverseFlow:
            self.LEDArmMode = LightModeDrone.ArmFlowReverse
        elif mode == Mode.Hold:
            self.LEDArmMode = LightModeDrone.ArmHold
        elif mode == Mode.Mix:
            self.LEDArmMode = LightModeDrone.ArmMix
        elif mode == Mode.Off:
            self.LEDArmMode = LightModeDrone.ArmNone
        else:
            return None

    ### LEDS ----------- END

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

    # Setup Start
    def sendCommand(self, commandType, option=0):
        if ((not isinstance(commandType, CommandType)) or (not isinstance(option, int))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = commandType
        data.option = option
        return self.transfer(header, data)

    def sendModeVehicle(self, modeVehicle):
        if (not isinstance(modeVehicle, ModeVehicle)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ModeVehicle
        data.option = modeVehicle.value

        return self.transfer(header, data)

    def sendHeadless(self, headless):

        if (not isinstance(headless, Headless)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Headless
        data.option = headless.value

        return self.transfer(header, data)

    def sendTrim(self, trim):

        if ((not isinstance(trim, Trim))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Trim
        data.option = trim.value

        return self.transfer(header, data)

    def sendTrimDrive(self, wheel):

        if (not isinstance(wheel, int)):
            return None

        header = Header()

        header.dataType = DataType.TrimDrive
        header.length = TrimDrive.getSize()

        data = TrimDrive()

        data.wheel = wheel

        return self.transfer(header, data)

    def sendFlightEvent(self, flightEvent):

        if ((not isinstance(flightEvent, FlightEvent))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = flightEvent.value

        return self.transfer(header, data)

    def sendDriveEvent(self, driveEvent):

        if ((not isinstance(driveEvent, DriveEvent))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.DriveEvent
        data.option = driveEvent.value

        return self.transfer(header, data)

    def sendClearTrim(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ClearTrim
        data.option = 0

        return self.transfer(header, data)

    def sendClearGyroBias(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ClearGyroBias
        data.option = 0

        return self.transfer(header, data)

    def sendUpdateLookupTarget(self, deviceType):

        if ((not isinstance(deviceType, DeviceType))):
            return None

        header = Header()

        header.dataType = DataType.UpdateLookupTarget
        header.length = UpdateLookupTarget.getSize()

        data = UpdateLookupTarget()

        data.deviceType = deviceType

        return self.transfer(header, data)

    # Setup End


    # Command Start

    def sendControlWhile(self, roll, pitch, yaw, throttle, timeMs):
        timeSec = timeMs / 1000
        timeStart = time.time()

        while ((time.time() - timeStart) < timeSec):
            self.sendControl(roll, pitch, yaw, throttle)
            sleep(0.02)

        return self.sendControl(roll, pitch, yaw, throttle)

    def sendControlDrive(self, wheel, accel):
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        self._control.roll = accel
        self._control.pitch = 0
        self._control.yaw = 0
        self._control.throttle = wheel

        return self.transfer(header, self._control)

    def sendControlDriveWhile(self, wheel, accel, timeMs):
        timeSec = timeMs / 1000
        timeStart = time.time()

        while ((time.time() - timeStart) < timeSec):
            self.sendControlDrive(wheel, accel)
            sleep(0.02)

        return self.sendControlDrive(wheel, accel)

    # Command End


    # Device Start

    def sendMotor(self, motor0, motor1, motor2, motor3):

        if ((not isinstance(motor0, int)) or
                (not isinstance(motor1, int)) or
                (not isinstance(motor2, int)) or
                (not isinstance(motor3, int))):
            return None

        header = Header()

        header.dataType = DataType.Motor
        header.length = Motor.getSize()

        data = Motor()

        data.motor[0].forward = motor0
        data.motor[0].reverse = 0

        data.motor[1].forward = motor1
        data.motor[1].reverse = 0

        data.motor[2].forward = motor2
        data.motor[2].reverse = 0

        data.motor[3].forward = motor3
        data.motor[3].reverse = 0

        return self.transfer(header, data)

    def sendIrMessage(self, value):

        if ((not isinstance(value, int))):
            return None

        header = Header()

        header.dataType = DataType.IrMessage
        header.length = IrMessage.getSize()

        data = IrMessage()

        data.irData = value

        return self.transfer(header, data)

    # Device End



    # Light Start


    def sendLightMode(self, lightMode, colors, interval):

        if (((not isinstance(lightMode, LightModeDrone))) or
                (not isinstance(interval, int)) or
                (not isinstance(colors, Colors))):
            return None

        header = Header()

        header.dataType = DataType.LightMode
        header.length = LightMode.getSize()

        data = LightMode()

        data.mode = lightMode
        data.colors = colors
        data.interval = interval

        return self.transfer(header, data)

    def sendLightModeCommand(self, lightMode, colors, interval, commandType, option):

        if (((not isinstance(lightMode, LightModeDrone))) or
                (not isinstance(interval, int)) or
                (not isinstance(colors, Colors)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeCommand
        header.length = LightModeCommand.getSize()

        data = LightModeCommand()

        data.mode.mode = lightMode
        data.mode.colors = colors
        data.mode.interval = interval

        data.command.commandType = commandType
        data.command.option = option

        return self.transfer(header, data)

    def sendLightModeCommandIr(self, lightMode, interval, colors, commandType, option, irData):

        if (((not isinstance(lightMode, LightModeDrone))) or
                (not isinstance(interval, int)) or
                (not isinstance(colors, Colors)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int)) or
                (not isinstance(irData, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeCommandIr
        header.length = LightModeCommandIr.getSize()

        data = LightModeCommandIr()

        data.mode.mode = lightMode
        data.mode.colors = colors
        data.mode.interval = interval

        data.command.commandType = commandType
        data.command.option = option

        data.irData = irData

        return self.transfer(header, data)

    def sendLightModeColor(self, lightMode, r, g, b, interval):

        if ((not isinstance(lightMode, LightModeDrone)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int)) or
                (not isinstance(interval, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = lightMode
        data.color.r = r
        data.color.g = g
        data.color.b = b
        data.interval = interval

        return self.transfer(header, data)

    def sendLightEvent(self, lightEvent, colors, interval, repeat):

        if (((not isinstance(lightEvent, LightModeDrone))) or
                (not isinstance(colors, Colors)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEvent
        header.length = LightEvent.getSize()

        data = LightEvent()

        data.event = lightEvent
        data.colors = colors
        data.interval = interval
        data.repeat = repeat

        return self.transfer(header, data)

    def sendLightEventCommand(self, lightEvent, colors, interval, repeat, commandType, option):

        if (((not isinstance(lightEvent, LightModeDrone))) or
                (not isinstance(colors, Colors)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEventCommand
        header.length = LightEventCommand.getSize()

        data = LightEventCommand()

        data.event.event = lightEvent
        data.event.colors = colors
        data.event.interval = interval
        data.event.repeat = repeat

        data.command.commandType = commandType
        data.command.option = option

        return self.transfer(header, data)

    def sendLightEventCommandIr(self, lightEvent, colors, interval, repeat, commandType, option, irData):

        if (((not isinstance(lightEvent, LightModeDrone))) or
                (not isinstance(colors, Colors)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int)) or
                (not isinstance(irData, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEventCommandIr
        header.length = LightEventCommandIr.getSize()

        data = LightEventCommandIr()

        data.event.event = lightEvent
        data.event.colors = colors
        data.event.interval = interval
        data.event.repeat = repeat

        data.command.commandType = commandType
        data.command.option = option

        data.irData = irData

        return self.transfer(header, data)

    def sendLightEventColor(self, lightEvent, r, g, b, interval, repeat):

        if (((not isinstance(lightEvent, LightModeDrone))) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int)) or
                (not isinstance(interval, int)) or
                (not isinstance(repeat, int))):
            return None

        header = Header()

        header.dataType = DataType.LightEventColor
        header.length = LightEventColor.getSize()

        data = LightEventColor()

        data.event = lightEvent.value
        data.color.r = r
        data.color.g = g
        data.color.b = b
        data.interval = interval
        data.repeat = repeat

        return self.transfer(header, data)

    def sendLightModeDefaultColor(self, lightMode, r, g, b, interval):

        if ((not isinstance(lightMode, LightModeDrone)) or
                (not isinstance(r, int)) or
                (not isinstance(g, int)) or
                (not isinstance(b, int)) or
                (not isinstance(interval, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()

        data.mode = lightMode
        data.color.r = r
        data.color.g = g
        data.color.b = b
        data.interval = interval

        return self.transfer(header, data)

    # Light End



    # Link Start


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

# Vibrator End

