from operator import eq
from queue import Queue
from threading import RLock
from threading import Thread
from time import sleep
import colorama
from colorama import Fore, Back, Style
import serial
from serial.tools.list_ports import comports

from CoDrone.receiver import *
from CoDrone.storage import *


def convertByteArrayToString(dataArray):
    if dataArray == None:
        return ""

    string = ""

    if (isinstance(dataArray, bytes)) or (isinstance(dataArray, bytearray)) or (not isinstance(dataArray, list)):
        for data in dataArray:
            string += "{0:02X} ".format(data)

    return string


class CoDrone:

    def __init__(self, flagCheckBackground=True, flagShowErrorMessage=False, flagShowLogMessage=False,
                 flagShowTransferData=False, flagShowReceiveData=False):

        self._serialport = None
        self._bufferQueue = Queue(4096)
        self._bufferHandler = bytearray()
        self._index = 0

        # Thread
        self._threadReceiving = None
        self._threadSendState = None
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
        self.timeStartProgram = time()  # record program starting time

        # Data
        self._timer = Timer()
        self._data = Data(self._timer)
        self._setAllEventHandler()

        # Parameter
        self._lowBatteryPercent = 30    # when the program starts, battery alert percentage

        # LED
        self._LEDColor = [255, 0, 0]
        self._LEDArmMode = LightModeDrone.ArmHold
        self._LEDEyeMode = LightModeDrone.EyeHold
        self._LEDInterval = 100
        colorama.init()

    def __del__(self):
        self.close()


    ### DATA PROCESSING THREAD -------- START

    def _receiving(self, lock, lockState):
        """Data receiving Thread, Save received data to buffer.

        Args:
            lock: main thread lock
            lockState: _sendRequestState lock
        """
        self._lockReciving = RLock()
        while self._flagThreadRun:
            # lock other threads for reading
            with lock and lockState and self._lockReciving:
                self._bufferQueue.put(self._serialport.read())

            # auto-update when background check for receive data is on
            if self._flagCheckBackground:
                while self._check() != DataType.None_:
                    pass
                    # sleep(0.001)

    def _check(self):
        """Read 1-byte of data stored in the buffer and pass it to the receiver.

        Returns: A member value in the DataType class.
            If one data block received call _handler(), data parsing and return the datatype.
            Returns DataType.None_ if no data received.
        """
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

    def _handler(self, header, dataArray):
        """Save header internally. Data parsing and saved internal class.
        If event handler is registered, call function.

        Returns: datatype, A member value in the DataType class.
        """

        # save input data
        message = self._runHandler(header, dataArray)

        if (message != None):   # print error
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
        """Store header and data into instance variables.
        """
        if self._parser.d[header.dataType] is not None:
            self._storageHeader.d[header.dataType] = header
            self._storage.d[header.dataType] = self._parser.d[header.dataType](dataArray)

    def _runEventHandler(self, dataType):
        """Call event handler with specified type of data
        """
        if (isinstance(dataType, DataType)) and (self._eventHandler.d[dataType] is not None) and (
                self._storage.d[dataType] is not None):
            return self._eventHandler.d[dataType](self._storage.d[dataType])
        else:
            return None

    def _setAllEventHandler(self):
        """Set all event handlers for SENSORS part functions.
        """
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

    def _sendRequestState(self, lock):
        """Data request Thread, Send state data request every 2 sec.
        Args:
            lock: main thread lock
        """
        self._lockState = RLock()
        while self._flagThreadRun:
            if self._flagConnected:
                with lock and self._lockState:
                    self.sendRequest(DataType.State)
                    sleep(0.01)
            sleep(2)

    def lockState(func):
        """This function is a decorator for thread-locking.
        If you apply this decorator to the function, the data request thread doesn't work while the function works.

        Examples:
            @lockState
            def func:
                pass
        """
        def wrapper(self, *args, **kwargs):
            with self._lockState:
                return func(self, *args, **kwargs)
        return wrapper

    ### DATA PROCESSING THREAD -------- END


    ### PRIVATE -------- START

    def _makeTransferDataArray(self, header, data):
        """Make transfer byte data array
        """
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

    def _transfer(self, header, data):
        """Transfer data
        """
        if not self.isOpen():
            return

        dataArray = self._makeTransferDataArray(header, data)
        with self._lockReciving and self._lock and self._lockState:
            self._serialport.write(dataArray)

        # print _transfer data
        self._printTransferData(dataArray)
        return dataArray

    @lockState
    def _checkAck(self, header, data, timeOnce=0.03, timeAll=0.2, count=5):
        """This function checks the ack response after the data transfer.
        If not received, repeat the data transfer depending on parameters.

        Args:
            timeOnce: The time interval between the retransmissions of data. The number of seconds as type float.
            timeAll: The time until the function ends. The number of seconds as type float.
            count: The number of transfers

        Returns: True if the transfer works well, False otherwise.
        """
        self._data.ack.dataType = 0
        flag = 1

        self._transfer(header, data)
        startTime = time()
        while self._data.ack.dataType != header.dataType:
            interval = time() - startTime
            # Break the loop if request time is over timeAll sec, send the request maximum flagAll times
            if interval > timeOnce * flag and flag < count:
                self._transfer(header, data)
                flag += 1
            elif interval > timeAll:
                self._printError(">> Failed to receive ack : {}".format(header.dataType))
                break
            sleep(0.01)
        return self._data.ack.dataType == header.dataType

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

    def _printLog(self, message):
        if self._flagShowLogMessage and message is not None:
            print(Fore.GREEN + "[{0:10.03f}] {1}".format((time() - self.timeStartProgram),
                                                         message) + Style.RESET_ALL)

    def _printError(self, message):
        if self._flagShowErrorMessage and message is not None:
            print(
                Fore.RED + "[{0:10.03f}] {1}".format((time() - self.timeStartProgram), message) + Style.RESET_ALL)

    def _printTransferData(self, dataArray):
        if (self._flagShowTransferData) and (dataArray != None) and (len(dataArray) > 0):
            print(Back.YELLOW + Fore.BLACK + convertByteArrayToString(dataArray) + Style.RESET_ALL)

    def _printReceiveData(self, dataArray):
        if (self._flagShowReceiveData) and (dataArray != None) and (len(dataArray) > 0):
            print(Back.CYAN + Fore.BLACK + convertByteArrayToString(dataArray) + Style.RESET_ALL, end='')

    def _printReceiveDataEnd(self):
        if self._flagShowReceiveData:
            print("")

    ### PRIVATE -------- END


    ### PUBLIC COMMON -------- START

    def isOpen(self):
        """Serial port connection status return.

        Returns: True if port is opened, false otherwise.
        """
        if self._serialport is not None:
            return self._serialport.isOpen()
        else:
            return False

    def isConnected(self):
        """BLE connection status return.

        Returns: True if BLE is connected, false otherwise.
        """
        if not self.isOpen():
            return False
        else:
            return self._flagConnected

    def open(self, portName="None"):
        """Open serial port. If not specify a port name, connect to the last detected device.

        Args: Serial port name such as "COM14"

        Returns: True if port is opened, false otherwise.
        """
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
            self._threadReceving = Thread(target=self._receiving, args=(self._lock, self._lockState,), daemon=True).start()

            # print log
            self._printLog(">> Connected.({0})".format(portName))
            return True
        else:
            # print error message
            self._printError(">> Could not open the serial port.")
            return False

    def close(self):
        """Close serial port
        """

        # print log
        if self.isOpen():
            self._printLog("Closing serial port.")

        # close thread
        if self._flagThreadRun:
            self._flagThreadRun = False
            sleep(0.01)

        for i in range(5):
            self.sendLinkDisconnect()
            sleep(0.01)

        while self.isOpen():
            self._serialport.close()
            sleep(0.01)

    def connect(self, deviceName="None", portName="None", flagSystemReset=False):
        """If the serial port is not open, open the serial port,
        Search for CODRONE and connect it to the device with the strongest signal.

        Args:
            deviceName: If specify a deviceName, Connect only when the specified device is discovered.
            portName: Serial port name.
            flagSystemReset: Use to reset and start the first CODRONE LINK after the serial communication connection.

        Returns: True if connected, false otherwise.
        """

        # case for serial port is None(connect to last connection)
        if not self.isOpen():
            self.close()
            self.open(portName)
            sleep(0.1)

        # if not connect with serial port print error and return
        if not self.isOpen():
            # print error
            self._printError(">> Could not connect to serial port.")
            return False

        # system reset
        if flagSystemReset:
            self.sendLinkSystemReset()
            sleep(3)

        # ModeLinkBroadcast.Passive mode change
        self.sendLinkModeBroadcast(ModeLinkBroadcast.Passive)
        sleep(0.1)

        for reconnection in range(5):
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
                    self._printError(">> Could not find CODRONE.")

            else:
                # check the name of connected device
                targetDevice = None

                if (len(self._devices) > 0):
                    if (len(deviceName) == 4):
                        for i in range(len(self._devices)):
                            if (len(self._devices[i].name) > 12) and (deviceName == self._devices[i].name[8:12]):
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
                            self._printError(">> Could not find " + deviceName + ".")

                    else:
                        self._printError(">> Device name length error(" + deviceName + ").")

                else:
                    self._printError(">> Could not find CODRONE.")

            ## TO DO
            ## How to alert low battery
            if self._flagConnected:
                battery = self.getBatteryPercentage()
                print(">> Drone battery : [{}]".format(battery))
                if battery < self._lowBatteryPercent:
                    print(">> Low Battery!!")
                sleep(3)
                return self._flagConnected
            else:
                self._printError(">> Trying to connect : {}/5".format(reconnection+1))
                if i == 4:
                    self._printError(">> Fail to connect.")
        return self._flagConnected

    def disconnect(self):
        """Disconnect the drone.
        """
        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDisconnect
        data.option = 0

        return self._transfer(header, data)

    ### PUBLIC COMMON -------- END


    ### SENDING -------- Start

    def sendPing(self):
        header = Header()

        header.dataType = DataType.Ping
        header.length = Ping.getSize()

        data = Ping()

        data.systemTime = 0

        return self._transfer(header, data)

    @lockState
    def sendRequest(self, dataType):
        """This function sends data request with specified datatype.

        Args:
            dataType: a member value in the DataType enum class.

        Examples:
            >>> sendRequest(DataType.State)

        Returns: True if responds well, false otherwise.
        """
        if not isinstance(dataType, DataType):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        header = Header()

        header.dataType = DataType.Request
        header.length = Request.getSize()

        data = Request()

        data.dataType = dataType
        return self._transfer(header,data)

    @lockState
    def sendControl(self, roll, pitch, yaw, throttle):
        """This function sends control request.

        Args:
            roll: the power of the roll, which is an int from -100 to 100
            pitch: the power of the pitch, which is an int from -100 to 100
            yaw: the power of the yaw, which is an int from -100 to 100
            throttle: the power of the throttle, which is an int from -100 to 100

        Returns: True if responds well, false otherwise.
        """
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll, pitch, yaw, throttle)

        timeStart = time()

        receivingFlag = self._storageCount.d[DataType.Attitude]

        while (time() - timeStart) < 0.2:
            self._transfer(header, control)
            sleep(0.02)
            if self._storageCount.d[DataType.Attitude] > receivingFlag:
                break
        if self._storageCount.d[DataType.Attitude] == receivingFlag:
            self._printError(">> Failed to send control.")

        return self._storageCount.d[DataType.Attitude] == receivingFlag

    @lockState
    def sendControlDuration(self, roll, pitch, yaw, throttle, duration):
        """This function sends control request for the duration

        Args:
            roll: the power of the roll, which is an int from -100 to 100
            pitch: the power of the pitch, which is an int from -100 to 100
            yaw: the power of the yaw, which is an int from -100 to 100
            throttle: the power of the throttle, which is an int from -100 to 100

        Returns: True if responds well, false otherwise.
        """
        if duration == 0:
            return self.sendControl(roll, pitch, yaw, throttle)

        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(roll, pitch, yaw, throttle)

        self._transfer(header, control)

        timeStart = time()
        while (time() - timeStart) < duration:
            self._transfer(header, control)
            sleep(0.02)

        self.hover(1)

    ### SENDING -------- End


    ### FLIGHT VARIABLES -------- START
    """
    Setter:
        Args:
            power: An int from -100 to 100 that sets the variable.

    Getter:
        Returns: The power of the variable(int)        
    """
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
        """This is a setter function that allows you to set the trim of the drone if it's drifting.

        Args:
            roll: An int from -100(left) to 100(right) that sets the roll trim.
            pitch: An int from -100(backward) to 100(forward) that sets the pitch trim.
            yaw: An int from -100(counterclockwise) to 100(clockwise) that sets the yaw trim.
            throttle: An int from -100(downward) to 100(upward) that sets the throttle trim.
        """
        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()
        data.setAll(roll, pitch, yaw, throttle)

        if not self._checkAck(header, data):
            self._printError(">> Failed to trim")

    def resetTrim(self, power):
        """This is a setter function that allows you to set the throttle variable.

        Args:
            power: An int from -100 to 100 that sets the throttle variable.
            The number represents the direction and power of the output for that flight motion variable.
            Negative throttle descends, positive throttle ascends.
        """
        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()
        data.setAll(0, 0, 0, power)

        if not self._checkAck(header, data):
            self._printError(">> Failed to reset trim")

    ### FLIGHT VARIABLES -------- END


    ### FLIGHT COMMANDS (START/STOP) -------- START

    def takeoff(self):
        """This function makes the drone take off and begin hovering.
        The drone will always hover for 3 seconds in order to stabilize before it executes the next command.
        If it receives no command for 8 seconds, it will automatically land.
        """
        self._data.takeoffFuncFlag = 1  # Event States

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.TakeOff.value

        if not self._checkAck(header, data):
            self._printError(">> Failed to takeoff")
        sleep(3)

    def land(self):
        """This function makes the drone stop all commands, hovers, and makes a soft landing where it is.
        The function will also zero-out all of the flight motion variables to 0.
        """
        self._control.setAll(0, 0, 0, 0)    # set the flight motion variables to 0.

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.Landing.value

        if not self._checkAck(header, data):
            self._printError(">> Failed to land")
        sleep(3)

    def hover(self, duration=0):
        """This function makes the drone hover for a given amount of time.

        Args:
            duration: The number of seconds to hover as type float. If 0, the duration is infinity.
        """
        timeStart = time()
        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        control = Control()
        control.setAll(0, 0, 0, 0)

        if duration != 0:
            while (time() - timeStart) < duration:
                self._transfer(header, control)
                sleep(0.1)
        else:
            if not self._checkAck(header, control):
                self._printError(">> Failed to hover")

    def emergencyStop(self):
        """This function immediately stops all commands and stops all motors, so the drone will stop flying immediately.
        The function will also zero-out all of the flight motion variables to 0.
        """
        self._data.stopFuncFlag = 1    # Event states
        self._control.setAll(0, 0, 0, 0)     # set the flight motion variables to 0

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Stop
        data.option = 0

        if not self._checkAck(header, data):
            self._printError(">> Failed to emergency stop")

    ### FLIGHT COMMANDS (START/STOP) -------- END


    ### FLIGHT COMMANDS (MOVEMENT) -------- START

    def move(self, duration=None, roll=None, pitch=None, yaw=None, throttle=None):
        """Once flying, the drone goes in the direction of the set flight motion variables.
        If the drone is not flying, nothing will happen.
        If you provide no parameters or only duration, it will execute it based on the current flight motion variables set.
        Args:
            duration: the duration of the flight motion in seconds. If 0, the duration is infinity.
            roll: the power of the roll, which is an int from -100 to 100
            pitch: the power of the pitch, which is an int from -100 to 100
            yaw: the power of the yaw, which is an int from -100 to 100
            throttle: the power of the throttle, which is an int from -100 to 100

        Examples:
            >>> move()  #goes infinity
            >>> move(3)    #goes for 3 seconds
            >>> move(3, 0,0,0,50)   #goes upward for 3 seconds at 50% power
        """
        if duration is None:    # move()
            self.sendControl(*self._control.getAll())
            sleep(1)
        elif roll is None:      # move(duration)
            self.sendControlDuration(*self._control.getAll(), duration)
        else:                   # move(duration, roll, pitch, yaw, throttle)
            self.sendControlDuration(roll, pitch, yaw, throttle, duration)

    def go(self, direction, duration=0, power=50):
        """A simpler Junior level function that represents positive flight with a direction, but with more natural language.
        It simply flies forward for the given duration and power.

        Args:
            direction: member values in the Direction enum class which can be one of the following: FORWARD, BACKWARD, LEFT, RIGHT, UP, and DOWN.
            duration: the duration of the flight motion in seconds.
                If 0, it will turn right indefinitely. Defaults to infinite if not defined.
            power: the power at which the drone flies. Takes a value from 0 to 100. Defaults to 50 if not defined.

        Examples:
            >>> go(Direction.FORWARD)   # goes forward for 0.5 seconds at 50% power
            >>> go(Direction.UP, 3)    # goes up for 3 seconds at 50% power
            >>> go(Direction.BACKWARD, 3, 30)   # goes backward for 3 seconds at 30% power
        """
        # power or -power
        pitch = ((direction == Direction.FORWARD) - (direction == Direction.BACKWARD)) * power
        roll = ((direction == Direction.RIGHT) - (direction == Direction.LEFT)) * power
        yaw = 0
        throttle = ((direction == Direction.UP) - (direction == Direction.DOWN)) * power

        self.sendControlDuration(roll, pitch, yaw, throttle, duration)

    def turn(self, direction, duration=0, power=50):
        """A simpler Junior level function that represents yaw, but with more natural language.
        It simply turns in the given direction, duration and power.

        Args:
            direction: member values in the Direction enum class which can be one of the following: LEFT, RIGHT
            duration: the duration of the turn in seconds.
                If 0, it will turn right indefinitely. Defaults to 0.5 seconds if not defined.
            power: the power at which the drone turns right. Takes a value from 0 to 100. Defaults to 50 if not defined.

        Examples:
            >>> turn(Direction.LEFT)    # yaws left for 0.5 sec at 50 power
            >>> turn(Direction.RIGHT, 3)    # yaws right for 3 seconds at 50 power
            >>> turn(Direction.RIGHT, 5, 100)   # yaws right for 5 seconds at 100 power
        """
        yaw = ((direction == Direction.RIGHT) - (direction == Direction.LEFT)) * power
        if duration is None:
            self.sendControl(0, 0, yaw, 0)
        else:
            self.sendControlDuration(0, 0, yaw, 0, duration)

    @lockState
    def turnDegree(self, direction, degree):
        """An Senior level function that yaws by a given degree in a given direction.
        This function takes an input degree in an input direction, and turns until it reaches the given degree.

        Args:
            direction: member values in the Direction enum class which can be one of the following: LEFT, RIGHT
            degree: member values in the Degree enum class which can be one of the following:
                ANGLE_30, ANGLE_45, ANGLE_60, ANGLE_90, ANGLE_120, ANGLE_135, ANGLE_150, ANGLE_180

        Examples:
            >>> turnDegree(Direction.LEFT, Degree.ANGLE_30)    # turn left 30 degrees
        """
        if not isinstance(direction, Direction) or not isinstance(degree, Degree):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        power = 20
        bias = 3

        yawPast = self.getGyroAngles().YAW
        direction = ((direction == Direction.RIGHT) - (direction == Direction.LEFT))  # right = 1 / left = -1
        degreeGoal = direction * (degree.value - bias) + yawPast

        start_time = time()
        while (time() - start_time) < degree.value / 3:
            yaw = self._data.attitude.YAW  # Receive attitude data every time you send a flight command
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
        sleep(1)

    @lockState
    def rotate180(self):
        """This function makes the drone rotate 180 degrees.
        """
        power = 20
        bias = 3

        yawPast = self.getGyroAngles().YAW
        degreeGoal = 180 - bias + yawPast

        start_time = time()
        while (time() - start_time) < 60:
            yaw = self._data.attitude.YAW  # Receive attitude data every time you send a flight command
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
        """This is a setter function will make the drone fly to the given height above the object directly below its IR sensor (usually the ground).
        It’s effective between 20 and 1500 millimeters.
        It uses the IR sensor to continuously check for its height.

        height: An int from 20 to 2000 in millimeters.
        """
        power = 30
        interval = 20  # height - 10 ~ height + 10

        start_time = time()
        while time() - start_time < 100:
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
        sleep(1)

    ### FLIGHT COMMANDS (MOVEMENT) -------- END


    ### SENSORS -------- START

    @lockState
    def _getDataWhile(self, dataType, timer=None):
        """This function checks if a request arrived or not and requests again maximum 3 times, 0.15sec

        Args:
            dataType: member values in the DataType class
            timer: member values in the Timer class
        """
        timeStart = time()

        if timer is not None:
            if timer[0] > (timeStart - timer[1]):
                return False

        header = Header()
        header.dataType = DataType.Request
        header.length = Request.getSize()

        data = Request()
        data.dataType = dataType

        # Break the loop if request time is over 0.15sec, send the request maximum 3 times
        receivingFlag = self._storageCount.d[dataType]
        resendFlag = 1
        self._transfer(header, data)
        while self._storageCount.d[dataType] == receivingFlag:
            interval = time() - timeStart
            if interval > 0.03 * resendFlag and resendFlag < 3:
                self._transfer(header, data)
                resendFlag += 1
            elif interval > 0.15:
                break
            sleep(0.01)
        return self._storageCount.d[dataType] > receivingFlag

    def getHeight(self):
        """This is a getter function gets the current height of the drone from the object directly below its IR sensor.

        Returns:  The current height above the object directly below the drone’s IR height sensor.
        """

        #Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Range, self._timer.range)
        return self._data.range

    def getPressure(self):
        """This is a getter function gets the data from the barometer sensor.

        Returns: The barometer’s air pressure in milibars at (0.13 resolution).
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Pressure, self._timer.pressure)
        return self._data.pressure

    def getDroneTemp(self):
        """This is a getter function gets the data from the drone’s temperature sensor.
        Importantly, it reads the drone’s temperature, not the air around it.

        Returns: The temperature in celsius as an integer.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Pressure, self._timer.pressure)
        return self._data.temperature

    def getAngularSpeed(self):
        """This function gets the data from the gyrometer sensor for the roll, pitch, and yaw angular speed.

        Returns: The Angle class. Angle has ROLL, PITCH, YAW.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Imu, self._timer.imu)
        return self._data.gyro

    def getGyroAngles(self):
        """This function gets the data from the gyrometer sensor to determine the roll, pitch, and yaw as angles.

        Returns: The Angle class. Angle has ROLL, PITCH, YAW.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Attitude, self._timer.attitude)
        return self._data.attitude

    def getAccelerometer(self):
        """This function gets the accelerometer sensor data, which returns x, y, and z values in m/s2.

        Returns: The Axis class. Axis has X,Y,Z
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Imu, self._timer.imu)
        return self._data.accel

    def getOptFlowPosition(self):
        """This function gets the x and y coordinates from the optical flow sensor.

        Returns: The Position class. Position has X,Y
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.ImageFlow, self._timer.imageFlow)
        return self._data.imageFlow

    def getState(self):
        """This function gets the state of the drone, as in whether it’s: ready, take off, flight, flip, stop, landing, reverse, accident, error

        Returns: string of member values in the ModeFlight class.
            READY, TAKE_OFF, FLIGHT, FLIP, STOP, LANDING, REVERSE, ACCIDENT, ERROR

        Examples:
            >>>print(getState())
            Ready
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.State, self._timer.state)
        return self._data.state.name

    def getBatteryPercentage(self):
        """This function gets the battery percentage of the drone.

        Returns: The battery’s percentage as an integer from 0 - 100.
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Battery, self._timer.battery)
        return self._data.batteryPercent

    def getBatteryVoltage(self):
        """This function gets the voltage of the battery.

        Returns: The voltage of the battery as an a float
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.Battery, self._timer.battery)
        return self._data.batteryVoltage

    def getTrim(self):
        """This function gets the current trim values of the drone.

        Returns: The Flight class. Flight has ROLL, PITCH, YAW, THROTTLE
        """

        # Checks if a request arrived or not and requests again maximum 3 times, 0.15sec
        self._getDataWhile(DataType.TrimFlight, self._timer.trim)
        return self._data.trim

    ### SENSORS -------- END


    ### STATUS CHECKERS -------- START
    def isUpsideDown(self):
        """This function checks the current drone status if it's reversed or not.

        Returns:
             boolean: True if upside down, False otherwise.
        """
        return self._data.reversed == SensorOrientation.Normal

    def isFlying(self):
        """This function checks the current drone status if it's flying or not.

        Returns:
             boolean: True if flying, False otherwise.
        """
        return self._data.state == ModeFlight.FLIGHT

    def isReadyToFly(self):
        """This function checks the current drone status if it's ready or not.

        Returns:
             boolean: True if ready to fly, False otherwise.
        """
        return self._data.state == ModeFlight.READY

    ### STATUS CHECKERS -------- END


    ### EVENT STATES -------- START

    def onUpsideDown(self, func):
        """This function executes the function if drone is reversed.

        Args: A function.

        Example:
             def func():
                pass
             onUpsideDown(func)
        """
        self._data.upsideDown = func

    def onTakeoff(self, func):
        """This function executes the function if drone takeoff.

        Args: A function.

        Example:
             def func():
                pass
             onTakeoff(func)
        """
        self._data.takeoff = func

    def onFlying(self, func):
        """This function executes the function if drone is on flying.

        Args: A function.

        Example:
             def func():
                pass
             onFlying(func)
        """
        self._data.flying = func

    def onReady(self, func):
        """This function executes the function if drone is on ready.

        Args: A function.

        Example:
             def func():
                pass
             onReady(func)
        """
        self._data.ready = func

    def onEmergencyStop(self, func):
        """This function executes the function if drone is on emergency stop.

        Args: A function.

        Example:
             def func():
                pass
             onEmergencyStop(func)
        """
        self._data.emergencyStop = func

    def onLowBattery(self, func):
        """This function executes the function if drone is on low battery.

        Args: A function.

        Example:
             def func():
                pass
             onLowBattery(func)
        """
        self._data.lowBattery = func

    ### EVENT STATES -------- END


    ### LEDS -------- START

    def setArmRGB(self, red, green, blue):
        """This function sets the LED color of the arms based on input red, green, and blue values.
        Mode and bright are set with Hold and 100% for default

        Args:
            red: int value from 0 to 255
            green: int value from 0 to 255
            blue: int value from 0 to 255
        """
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            self._printError(">>> Parameter Type Error")    # print error message
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

        self._checkAck(header, data, 0.06, 0.3, 3)

    def setEyeRGB(self, red, green, blue):
        """This function sets the LED color of the eyes based on input red, green, and blue values

        Args:
             red: int value from 0 to 255
             green: int value from 0 to 255
             blue: int value from 0 to 255
        """
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            self._printError(">>> Parameter Type Error")    # print error message
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

        self._checkAck(header,data, 0.06, 0.3, 3)

    def setAllRGB(self, red, green, blue):
        """This function sets the LED color of the drone (except the green tail light) to the given color

        Args:
             red: int value from 0 to 255
             green: int value from 0 to 255
             blue: int value from 0 to 255
        """
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            self._printError(">>> Parameter Type Error")    # print error message
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

        self._checkAck(header, data, 0.06, 0.3, 3)

        data.mode = self._LEDArmMode
        self._checkAck(header, data, 0.06, 0.3, 3)

    def setArmDefaultRGB(self, red, green, blue):
        """This function sets the default LED color of the arms.
        It will remain that color after powering off and back on.

        Args:
            red: int value from 0 to 255
            green: int value from 0 to 255
            blue: int value from 0 to 255
        """
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            self._printError(">>> Parameter Type Error")    # print error message
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

        self._checkAck(header, data, 0.06, 0.4, 3)

    def setEyeDefaultRGB(self, red, green, blue):
        """This function sets the default LED color of the eyes.
        It will remain that color after powering off and back on.
        Based on input red, green, and blue values

        Args:
            red: int value from 0 to 255
            green: int value from 0 to 255
            blue: int value from 0 to 255
        """
        if ((not isinstance(red, int)) or
                (not isinstance(green, int)) or
                (not isinstance(blue, int))):
            self._printError(">>> Parameter Type Error")    # print error message
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

        self._checkAck(header, data, 0.06, 0.4, 3)

    def resetDefaultLED(self):
        """This function sets the LED color of the eyes and arms back to red, which is the original default color.
        """
        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = LightModeDrone.EyeHold
        data.color.r = 255
        data.color.g = 0
        data.color.b = 0
        data.interval = self._LEDInterval

        self._checkAck(header, data, 0.06, 0.4, 3)

        data.mode = LightModeDrone.ArmHold
        self._checkAck(header, data, 0.06, 0.4, 3)

    def setEyeMode(self, mode):
        """This function sets the LED light mode of the eyes to behave in different patterns.

        Args:
            Member values in the Mode enum class. Mode class has NONE,HOLD, MIX, FLICKER, FLICKER_DOUBLE, DIMMING

        Examples:
            >>> setEyeMode(Mode.HOLD)
        """
        # EYE doesn't have flow mode
        if not isinstance(mode, Mode) or mode.value > Mode.DIMMING.value:
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        self._LEDEyeMode = mode

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self._LEDEyeMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        return self._checkAck(header, data, 0.06, 0.3, 3)

    def setArmMode(self, mode):
        """This function sets the LED light mode of the arms to behave in different patterns.

        Args:
            Member values in the Mode enum class. Mode class has NONE,HOLD, MIX, FLICKER, FLICKER_DOUBLE, DIMMING, FLOW, FLOW_REVERSE

        Examples:
            >>> setArmMode(Mode.HOLD)
        """
        if not isinstance(mode, Mode):
            self._printError(">>> Parameter Type Error")  # print error message
            return None

        self._LEDArmMode = LightModeDrone(mode.value + 0x30)

        header = Header()

        header.dataType = DataType.LightModeColor
        header.length = LightModeColor.getSize()

        data = LightModeColor()

        data.mode = self._LEDArmMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        self._checkAck(header, data, 0.06, 0.3, 3)

    def setEyeDefaultMode(self, mode):
        """This function sets the LED light default mode of the eyes to behave in different patterns.

        Args:
            Member values in the Mode enum class. Mode class has NONE,HOLD, MIX, FLICKER, FLICKER_DOUBLE, DIMMING

        Examples:
            >>> setEyeDefaultMode(Mode.HOLD)
        """
        # EYE doesn't have flow mode
        if not isinstance(mode, Mode) or mode.value > Mode.DIMMING.value:
            self._printError(">>> Parameter Type Error")  # print error message
            return None

        self._LEDEyeMode = mode

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self._LEDEyeMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        self._checkAck(header, data, 0.06, 0.4, 3)

    def setArmDefaultMode(self, mode):
        """This function sets the LED light default mode of the arms to behave in different patterns.

        Args:
            Member values in the Mode enum class. Mode class has NONE,HOLD, MIX, FLICKER, FLICKER_DOUBLE, DIMMING, FLOW, FLOW_REVERSE

        Examples:
            >>> setArmDefaultMode(Mode.HOLD)
        """
        if not isinstance(mode, Mode):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        self._LEDArmMode = LightModeDrone(mode.value + 0x30)

        header = Header()

        header.dataType = DataType.LightModeDefaultColor
        header.length = LightModeDefaultColor.getSize()

        data = LightModeDefaultColor()
        data.mode = self._LEDArmMode
        data.color.r, data.color.g, data.color.b = self._LEDColor
        data.interval = self._LEDInterval

        self._checkAck(header, data, 0.06, 0.3, 3)

    ### LEDS --------- END


    ### FLIGHT SEQUENCES -------- START

    def flySequence(self, sequence):
        """This function makes the drone fly in a given pattern, then land.

        Args:
            Member values in the Sequence class. Sequence class has SQUARE, CIRCLE, SPIRAL, TRIANGLE, HOP, SWAY, ZIG_ZAG
        """
        if sequence == Sequence.SQUARE:
            self.flySquare()
        elif sequence == Sequence.CIRCLE:
            self.flyCircle()
        elif sequence == Sequence.SPIRAL:
            self.flySpiral()
        elif sequence == Sequence.TRIANGLE:
            self.flyTriangle()
        elif sequence == Sequence.HOP:
            self.flyHop()
        elif sequence == Sequence.SWAY:
            self.flySway()
        elif sequence == Sequence.ZIGZAG:
            self.flyZigzag()
        else:
            return None

    def flyRoulette(self):
        """This function makes yaw for a random number of seconds between 5 and 10, then pitch forward in that direction.
        """

        self.turn(Direction.RIGHT, 5 + (self.timeStartProgram % 5), 30)
        self.go(Direction.FORWARD, 1)

        self.hover(1)

    def turtleTurn(self):
        """If the drone is in the upside down state.
        This function makes the drone turn right side up by spinning the right two propellers
        """
        self.go(Direction.UP, 1, 100)

    def flySquare(self):

        self.go(Direction.RIGHT, 2, 30)
        self.go(Direction.FORWARD, 2, 30)
        self.go(Direction.LEFT, 2, 30)
        self.go(Direction.BACKWARD, 2, 30)

        self.hover(1)

    def flyCircle(self):

        power = -50
        yaw = self.getGyroAngles().YAW
        degree = -360 + yaw

        startTime = time()
        while (time() - startTime) < 15:
            if abs(yaw - self._data.attitude.YAW) > 180:
                degree += 360
            yaw = self._data.attitude.YAW
            if degree < yaw:
                self.sendControl(10, 0, power, 0)
                sleep(0.1)
            else:
                break

        self.hover(1)

    def flySpiral(self):

        for i in range(5):
            self.sendControl(10, 0, -50, -i * 2)
            sleep(1)

        self.hover(1)

    def flyTriangle(self):

        self.turnDegree(Direction.RIGHT, Degree.ANGLE_30)
        self.go(Direction.FORWARD, 2, 30)
        self.turnDegree(Direction.LEFT, Degree.ANGLE_120)
        self.go(Direction.FORWARD, 2, 30)
        self.turnDegree(Direction.LEFT, Degree.ANGLE_120)
        self.go(Direction.FORWARD, 2, 30)

        self.hover(1)

    def flyHop(self):

        self.sendControlDuration(0, 30, 0, 50, 1)
        self.sendControlDuration(0, 30, 0, -50, 1)

        self.hover(1)

    def flySway(self):

        for i in range(2):
            self.go(Direction.LEFT, 1, 50)
            self.go(Direction.RIGHT, 1, 50)

        self.hover(1)

    def flyZigzag(self):

        for i in range(2):
            self.move(1, 50, 50, 0, 0)
            self.move(1, -50, 50, 0, 0)

        self.hover(1)

    ### FLIGHT SEQUENCES -------- END


    ### Link -------- Start

    def sendLinkModeBroadcast(self, modeLinkBroadcast):
        if (not isinstance(modeLinkBroadcast, ModeLinkBroadcast)):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkModeBroadcast
        data.option = modeLinkBroadcast.value

        return self._transfer(header, data)

    def sendLinkSystemReset(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkSystemReset
        data.option = 0

        return self._transfer(header, data)

    def sendLinkDiscoverStart(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDiscoverStart
        data.option = 0

        return self._transfer(header, data)

    def sendLinkDiscoverStop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDiscoverStop
        data.option = 0

        return self._transfer(header, data)

    def sendLinkConnect(self, index):

        if (not isinstance(index, int)):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkConnect
        data.option = index

        return self._transfer(header, data)

    def sendLinkDisconnect(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkDisconnect
        data.option = 0

        return self._transfer(header, data)

    def sendLinkRssiPollingStart(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkRssiPollingStart
        data.option = 0

        return self._transfer(header, data)

    def sendLinkRssiPollingStop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.LinkRssiPollingStop
        data.option = 0

        return self._transfer(header, data)

    ### LINK -------- END


    def setEventHandler(self, dataType, eventHandler):
        if (not isinstance(dataType, DataType)):
            return

        self._eventHandler.d[dataType] = eventHandler

    def getHeader(self, dataType):
        if (not isinstance(dataType, DataType)):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        return self._storageHeader.d[dataType]

    def getData(self, dataType):
        if (not isinstance(dataType, DataType)):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        return self._storage.d[dataType]

    def getCount(self, dataType):

        if (not isinstance(dataType, DataType)):
            self._printError(">>> Parameter Type Error")    # print error message
            return None

        return self._storageCount.d[dataType]


    ### LEGACY CODE -------- START

    def sendTakeOff(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.TakeOff.value

        return self._transfer(header, data)

    def sendLanding(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = FlightEvent.Landing.value

        return self._transfer(header, data)

    def sendStop(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Stop
        data.option = 0

        return self._transfer(header, data)

    def sendControlWhile(self, roll, pitch, yaw, throttle, timeMs):

        if ((not isinstance(roll, int)) or (not isinstance(pitch, int)) or (not isinstance(yaw, int)) or (
        not isinstance(throttle, int))):
            return None

        timeSec = timeMs / 1000
        timeStart = time()

        while ((time() - timeStart) < timeSec):
            self.sendControl(roll, pitch, yaw, throttle)
            sleep(0.02)

        return self.sendControl(roll, pitch, yaw, throttle)

    def sendControlDrive(self, wheel, accel):

        if ((not isinstance(wheel, int)) or (not isinstance(accel, int))):
            return None

        header = Header()

        header.dataType = DataType.Control
        header.length = Control.getSize()

        data = Control()

        data.roll = accel
        data.pitch = 0
        data.yaw = 0
        data.throttle = wheel

        return self._transfer(header, data)

    def sendControlDriveWhile(self, wheel, accel, timeMs):

        if ((not isinstance(wheel, int)) or (not isinstance(accel, int))):
            return None

        timeSec = timeMs / 1000
        timeStart = time()

        while ((time() - timeStart) < timeSec):
            self.sendControlDrive(wheel, accel)
            sleep(0.02)

        return self.sendControlDrive(wheel, accel)

    # Control End



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

        return self._transfer(header, data)

    def sendModeVehicle(self, modeVehicle):

        if (not isinstance(modeVehicle, ModeVehicle)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ModeVehicle
        data.option = modeVehicle.value

        return self._transfer(header, data)

    def sendHeadless(self, headless):

        if (not isinstance(headless, Headless)):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Headless
        data.option = headless.value

        return self._transfer(header, data)

    def sendTrim(self, trim):

        if ((not isinstance(trim, Trim))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.Trim
        data.option = trim.value

        return self._transfer(header, data)

    def sendTrimFlight(self, roll, pitch, yaw, throttle):

        if ((not isinstance(roll, int)) or (not isinstance(pitch, int)) or (not isinstance(yaw, int)) or (
        not isinstance(throttle, int))):
            return None

        header = Header()

        header.dataType = DataType.TrimFlight
        header.length = TrimFlight.getSize()

        data = TrimFlight()

        data.roll = roll
        data.pitch = pitch
        data.yaw = yaw
        data.throttle = throttle

        return self._transfer(header, data)

    def sendTrimDrive(self, wheel):

        if (not isinstance(wheel, int)):
            return None

        header = Header()

        header.dataType = DataType.TrimDrive
        header.length = TrimDrive.getSize()

        data = TrimDrive()

        data.wheel = wheel

        return self._transfer(header, data)

    def sendFlightEvent(self, flightEvent):

        if ((not isinstance(flightEvent, FlightEvent))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.FlightEvent
        data.option = flightEvent.value

        return self._transfer(header, data)

    def sendDriveEvent(self, driveEvent):

        if ((not isinstance(driveEvent, DriveEvent))):
            return None

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.DriveEvent
        data.option = driveEvent.value

        return self._transfer(header, data)

    def sendClearTrim(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ClearTrim
        data.option = 0

        return self._transfer(header, data)

    def sendClearGyroBias(self):

        header = Header()

        header.dataType = DataType.Command
        header.length = Command.getSize()

        data = Command()

        data.commandType = CommandType.ClearGyroBias
        data.option = 0

        return self._transfer(header, data)

    def sendUpdateLookupTarget(self, deviceType):

        if ((not isinstance(deviceType, DeviceType))):
            return None

        header = Header()

        header.dataType = DataType.UpdateLookupTarget
        header.length = UpdateLookupTarget.getSize()

        data = UpdateLookupTarget()

        data.deviceType = deviceType

        return self._transfer(header, data)

    # Setup End



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

        return self._transfer(header, data)

    def sendIrMessage(self, value):

        if ((not isinstance(value, int))):
            return None

        header = Header()

        header.dataType = DataType.IrMessage
        header.length = IrMessage.getSize()

        data = IrMessage()

        data.irData = value

        return self._transfer(header, data)

    # Device End



    # Light Start


    def sendLightMode(self, lightMode, colors, interval):

        if (((not isinstance(lightMode, LightMode))) or
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

        return self._transfer(header, data)

    def sendLightModeCommand(self, lightMode, colors, interval, commandType, option):

        if (((not isinstance(lightMode, LightMode))) or
                (not isinstance(interval, int)) or
                (not isinstance(colors, Colors)) or
                (not isinstance(commandType, CommandType)) or
                (not isinstance(option, int))):
            return None

        header = Header()

        header.dataType = DataType.LightModeCommand
        header.length = LightModeCommand.getSize()

        data = LightModeCommand()

        data.lightMode.mode = lightMode
        data.lightMode.colors = colors
        data.lightMode.interval = interval

        data.command.commandType = commandType
        data.command.option = option

        return self._transfer(header, data)

    def sendLightModeCommandIr(self, lightMode, interval, colors, commandType, option, irData):

        if (((not isinstance(lightMode, LightMode))) or
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

        data.lightMode.mode = lightMode
        data.lightMode.colors = colors
        data.lightMode.interval = interval

        data.command.commandType = commandType
        data.command.option = option

        data.irData = irData

        return self._transfer(header, data)

    def sendLightModeColor(self, lightMode, r, g, b, interval):

        if ((not isinstance(lightMode, LightMode)) or
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

        return self._transfer(header, data)

    def sendLightEvent(self, lightEvent, colors, interval, repeat):

        if (((not isinstance(lightEvent, LightMode))) or
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

        return self._transfer(header, data)

    def sendLightEventCommand(self, lightEvent, colors, interval, repeat, commandType, option):

        if (((not isinstance(lightEvent, LightMode))) or
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

        data.lightEvent.event = lightEvent
        data.lightEvent.colors = colors
        data.lightEvent.interval = interval
        data.lightEvent.repeat = repeat

        data.command.commandType = commandType
        data.command.option = option

        return self._transfer(header, data)

    def sendLightEventCommandIr(self, lightEvent, colors, interval, repeat, commandType, option, irData):

        if (((not isinstance(lightEvent, LightMode))) or
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

        data.lightEvent.event = lightEvent
        data.lightEvent.colors = colors
        data.lightEvent.interval = interval
        data.lightEvent.repeat = repeat

        data.command.commandType = commandType
        data.command.option = option

        data.irData = irData

        return self._transfer(header, data)

    def sendLightEventColor(self, lightEvent, r, g, b, interval, repeat):

        if (((not isinstance(lightEvent, LightMode))) or
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

        data.event = lightEvent.mode
        data.color.r = r
        data.color.g = g
        data.color.b = b
        data.interval = interval
        data.repeat = repeat

        return self._transfer(header, data)

    def sendLightModeDefaultColor(self, lightMode, r, g, b, interval):

        if ((not isinstance(lightMode, LightMode)) or
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

        return self._transfer(header, data)

    # Light End
    ### LEGACY CODE -------- END