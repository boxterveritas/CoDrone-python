import abc
from struct import *
import time
from CoDrone.system import *


# ISerializable Start


class ISerializable:
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def getSize(self):
        pass

    @abc.abstractmethod
    def toArray(self):
        pass


# ISerializable End


class EventStatesFunc:
    def __init__(self):
        self.upsideDown = None
        self.takeoff = None
        self.flying = None
        self.landing = None
        self.ready = None
        self.emergencyStop = None
        self.lowBattery = None


class Timer:
    def __init__(self):
        # [ time interval, variable to save start time ]
        self.address = [0, 0]
        self.attitude = [0.5, 0]
        self.battery = [5, 0]
        self.imu = [0, 0]
        self.pressure = [0, 0]
        self.trim = [0, 0]
        self.range = [0, 0]
        self.state = [0.1, 0]
        self.imageFlow = [0, 0]
        self.motor = [0, 0]

        # Event states flag
        self.upsideDown = [5, 0]
        self.takeoff = [5, 0]
        self.flying = [10, 0]
        self.landing = [5, 0]
        self.ready = [10, 0]
        self.emergencyStop = [5, 0]
        self.lowBattery = [10, 0]


class Data(EventStatesFunc):
    def __init__(self, timer):
        # criterion for low battery
        self._LowBatteryPercent = 50

        super().__init__()
        self.star_time = 0
        self.timer = timer
        self.address = 0
        self.attitude = Angle(0, 0, 0)
        self.attitude_roll = [0]
        self.attitude_pitch = [0]
        self.attitude_yaw = [0]
        self.accel = Axis(0, 0, 0)
        self.accel_x = [0]
        self.accel_y = [0]
        self.accel_z = [0]
        self.batteryPercent = 0
        self.batteryVoltage = 0
        self.gyro = Angle(0, 0, 0)
        self.gyro_roll = [0]
        self.gyro_pitch = [0]
        self.gyro_yaw = [0]
        self.imageFlow = Position(0, 0)
        self.imageFlow_x = [0]
        self.imageFlow_y = [0]
        self.pressure = [0]
        self.reversed = 0
        self.temperature = [0]
        self.trim = Flight(0, 0, 0, 0)
        self.range = [0]

        self.range_time = [0.0]
        self.attitude_time = [0.0]
        self.imu_time = [0.0]
        self.pressure_time = [0.0]
        self.motor_time = [0.0]
        self.state = 0
        self.motor = MotorVolt(0, 0, 0, 0)
        self.m1 = [0]
        self.m2 = [0]
        self.m3 = [0]
        self.m4 = [0]

        self.ack = Ack()

        # Depending on using flight commend
        self.takeoffFuncFlag = 0
        self.stopFuncFlag = 0

    def eventUpdateAddress(self, data):
        self.address = data.address
        self.timer.address[1] = time.time()

    def eventUpdateAttitude(self, data):
        self.attitude = Angle(data.roll, data.pitch, data.yaw)
        self.attitude_roll.append(self.attitude.ROLL)
        self.attitude_pitch.append(self.attitude.PITCH)
        self.attitude_yaw.append(self.attitude.YAW)
        self.attitude_time.append(time.process_time()-self.star_time)
        self.timer.attitude[1] = time.time()
        #print("update Attitude")

    def eventUpdateBattery(self, data):
        self.batteryPercent = data.batteryPercent
        self.batteryVoltage = data.voltage
        self.timer.battery[1] = time.time()
        #print("update battery")

    def eventUpdateImu(self, data):
        self.accel = Axis(data.accelX, data.accelY, data.accelZ)
        self.accel_x.append(self.accel.X)
        self.accel_y.append(self.accel.Y)
        self.accel_z.append(self.accel.Z)
        self.gyro = Angle(data.gyroRoll, data.gyroPitch, data.gyroYaw)
        self.gyro_roll.append(self.gyro.ROLL)
        self.gyro_pitch.append(self.gyro.PITCH)
        self.gyro_yaw.append(self.gyro.YAW)
        self.imu_time.append(time.process_time()-self.star_time)
        self.timer.imu[1] = time.time()
        #print("update imu")

    def eventUpdatePressure(self, data):
        self.pressure.append(data.pressure)
        self.temperature.append(data.temperature)
        self.pressure_time.append(time.process_time()-self.star_time)
        self.timer.pressure[1] = time.time()

    def eventUpdateRange(self, data):
        self.range.append(data.bottom)
        self.timer.range[1] = time.time()
        self.range_time.append(time.process_time()-self.star_time)
        # print(self.range)
        # print(self.rangetime)
        # print("update Range")

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
        if self.ready is not None and self.state == ModeFlight.READY:
            if start_time - self.timer.ready[1] > self.timer.ready[0]:
                self.ready()
                self.timer.ready[1] = start_time
                return
        if self.flying is not None and self.state == ModeFlight.FLIGHT:
            if start_time - self.timer.flying[1] > self.timer.flying[0]:
                self.flying()
                self.timer.flying[1] = start_time
                return
        if self.landing is not None and self.state == ModeFlight.LANDING:
            if start_time - self.timer.landing[1] > self.timer.landing[0]:
                self.landing()
                self.timer.landing[1] = start_time
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
        self.imageFlow_x.append(self.imageFlow.X)
        self.imageFlow_y.append(self.imageFlow.Y)
        self.timer.imageFlow[1] = time.time()
        # print("update Flow")

    def eventUpdateAck(self, data):
        self.ack = data

    def eventUpdateMotor(self, data):
        self.motor = MotorVolt(data.motor[0].forward,
                               data.motor[1].forward,
                               data.motor[2].forward,
                               data.motor[3].forward)
        self.m1.append(self.motor.m1)
        self.m2.append(self.motor.m2)
        self.m3.append(self.motor.m3)
        self.m4.append(self.motor.m4)
        self.motor_time.append(time.process_time()-self.star_time)
        # print('--------------time : {:4.2}---------------------'.format(self.motor_time[-1])
        # print('Left front({:4})\tRight front({:4})'.format(self.m1[-1], self.m2[-1]))
        # print("Left back ({:4})\tRight back ({:4})\n".format(self.m4[-1], self.m3[-1]))
        # # print('----------------------------------------')

# DataType Start
class DataType(Enum):
    None_ = 0x00

    # BLE + Serial
    Ping = 0x01  # check
    Ack = 0x02  # reply
    Error = 0x03
    Request = 0x04
    Passcode = 0x05  # new passcode for pairing

    Control = 0x10
    Command = 0x11
    Command2 = 0x12
    Command3 = 0x13

    # Light
    LightMode = 0x20  # set LED Mode
    LightMode2 = 0x21  # set LED Mode
    LightModeCommand = 0x22  # set LED Mode Commend
    LightModeCommandIr = 0x23  # set LED Mode Commend IR
    LightModeColor = 0x24  # set LED Mode RGB color
    LightModeColor2 = 0x25  # set LED Mode RGB color

    LightEvent = 0x26  # LED event
    LightEvent2 = 0x27  # LED event
    LightEventCommand = 0x28  # LED event Commend
    LightEventCommandIr = 0x29  # LED event Commend IR
    LightEventColor = 0x2A  # LED event RGB color
    LightEventColor2 = 0x2B  # LED event RGB color

    LightModeDefaultColor = 0x2C  # set LED default color
    LightModeDefaultColor2 = 0x2D  # set LED default color

    # status
    Address = 0x30  # IEEE Address
    State = 0x31  # drone's state(flight mode, cordinate, battery level)
    Attitude = 0x32  # attitude(Angle)
    GyroBias = 0x33
    TrimAll = 0x34  # trim for roll pitch yaw throttle wheel
    TrimFlight = 0x35  # trim for roll pitch yaw throttle
    TrimDrive = 0x36  # trim for wheel

    CountFlight = 0x37  # count about flight
    CountDrive = 0x38  # count about drive

    # IR
    IrMessage = 0x40  # IR data transfer or receive

    # Sensor and control
    Imu = 0x50  # IMU Raw
    Pressure = 0x51
    ImageFlow = 0x52  # ImageFlow
    Button = 0x53
    Battery = 0x54
    Motor = 0x55  # moter control value and current value for control
    Temperature = 0x56
    Range = 0x57  # bottom Ir range sensor

    # Firmware update
    UpdateLookupTarget = 0x90
    UpdateInformation = 0x91
    Update = 0x92
    UpdateLocationCorrect = 0x93

    # LINK board
    LinkState = 0xE0
    LinkEvent = 0xE1
    LinkEventAddress = 0xE2  # event + address
    LinkRssi = 0xE3  # rssi signal power which is connected with link board
    LinkDiscoveredDevice = 0xE4
    LinkPasscode = 0xE5  # set passcode for pairing

    Message = 0xF0  # string message

    EndOfType = 0xFF


# DataType End

class PlotType(Enum):
    pressure = 1
    image_flow = 2
    temperature = 3
    height = 4
    gyro = 5
    accel = 6
    angle = 7
    motor = 8

# CommandType Start


class CommandType(Enum):
    None_ = 0x00

    # setting
    ModeVehicle = 0x10

    # control
    Headless = 0x20  # headless mode
    Trim = 0x21
    FlightEvent = 0x22
    DriveEvent = 0x23
    Stop = 0x24  # killswitch

    ResetHeading = 0x50  # head reset
    ClearGyroBias = 0x51  # clear trim and gyroBias
    ClearTrim = 0x52  # clear trim
    Calibrate = 0x53  # calibration

    # Wireless Lan
    ResetWirelessLan = 0x70
    WirelessLanConnected = 0x70
    WirelessLanDisconnected = 0x70

    # Bluetooth
    PairingActivate = 0x80
    PairingDeactivate = 0x81
    AdvertisingStart = 0x82
    AdvertisingStop = 0x83
    TerminateConnection = 0x84
    ClearBondList = 0x85  # clear bond device info

    # request
    Request = 0x90
    UpdateCompleteSub = 0x90
    ClearUpdateAreaMain = 0x90

    # LINK module
    LinkModeBroadcast = 0xE0  # convert LINK mode
    LinkSystemReset = 0xE1  # system reset
    LinkDiscoverStart = 0xE2  # start searching device
    LinkDiscoverStop = 0xE3  # stop searching device
    LinkConnect = 0xE4  # connect with specific named device
    LinkDisconnect = 0xE5  # disconnect
    LinkRssiPollingStart = 0xE6  # RSSI polling start
    LinkRssiPollingStop = 0xE7  # RSSI polling stop

    EndOfType = 0xFF


# CommandType End


# Header Start


class Header(ISerializable):
    def __init__(self):
        self.dataType = DataType.None_
        self.length = 0

    @classmethod
    def getSize(cls):
        return 2

    def toArray(self):
        return pack('<BB', self.dataType.value, self.length)

    @classmethod
    def parse(cls, dataArray):
        header = Header()

        if len(dataArray) != cls.getSize():
            return None

        header.dataType, header.length = unpack('<BB', dataArray)

        header.dataType = DataType(header.dataType)

        return header


# Header End


# Common Start


class Ping(ISerializable):
    def __init__(self):
        self.systemTime = 0

    @classmethod
    def getSize(cls):
        return 4

    def toArray(self):
        return pack('<I', self.systemTime)

    @classmethod
    def parse(cls, dataArray):
        data = Ping()

        if len(dataArray) != cls.getSize():
            return None

        data.systemTime, = unpack('<I', dataArray)
        return data


class Ack(ISerializable):
    def __init__(self):
        self.systemTime = 0
        self.dataType = DataType.None_

    @classmethod
    def getSize(cls):
        return 5

    def toArray(self):
        return pack('<IB', self.systemTime, self.dataType.value)

    @classmethod
    def parse(cls, dataArray):
        data = Ack()

        if len(dataArray) != cls.getSize():
            return None

        data.systemTime, data.dataType = unpack('<IB', dataArray)
        data.dataType = DataType(data.dataType)

        return data


class Request(ISerializable):
    def __init__(self):
        self.dataType = DataType.None_

    @classmethod
    def getSize(cls):
        return 1

    def toArray(self):
        return pack('<B', self.dataType.value)

    @classmethod
    def parse(cls, dataArray):
        data = Request()

        if len(dataArray) != cls.getSize():
            return None

        data.dataType, = unpack('<B', dataArray)
        data.dataType = DataType(data.dataType)

        return data


class Passcode(ISerializable):
    def __init__(self):
        self.passcode = 0

    @classmethod
    def getSize(cls):
        return 4

    def toArray(self):
        return pack('<I', self.passcode)

    @classmethod
    def parse(cls, dataArray):
        data = Passcode()

        if len(dataArray) != cls.getSize():
            return None

        data.passcode, = unpack('<I', dataArray)

        return data


class Move():
    def __init__(self):
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        self._throttle = 0

    def _checkValue(self, value):
        try:
            value = int(value)
        except ValueError as err:
            raise ValueError('only integer values are permitted') from err

        if value > 100:
            raise ValueError('only under 100 values are permitted')
        elif value < -100:
            raise ValueError('only over -100 values are permitted')
        return value

    def getAll(self):
        return self._roll, self._pitch, self._yaw, self._throttle

    def setAll(self, roll, pitch, yaw, throttle):
        self._roll = self._checkValue(roll)
        self._pitch = self._checkValue(pitch)
        self._yaw = self._checkValue(yaw)
        self._throttle = self._checkValue(throttle)

    @property
    def roll(self):
        return self._roll

    @roll.setter
    def roll(self, value):
        self._roll = self._checkValue(value)

    @property
    def pitch(self):
        return self._pitch

    @pitch.setter
    def pitch(self, value):
        self._pitch = self._checkValue(value)

    @property
    def yaw(self):
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        self._yaw = self._checkValue(value)

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = self._checkValue(value)


class Control(ISerializable, Move):
    def __init__(self):
        Move.__init__(self)

    @classmethod
    def getSize(cls):
        return 4

    def toArray(self):
        return pack('<bbbb', self._roll, self._pitch, self._yaw, self._throttle)

    @classmethod
    def parse(cls, dataArray):
        data = Control()

        if len(dataArray) != cls.getSize():
            return None

        data._roll, data._pitch, data._yaw, data._throttle = unpack('<bbbb', dataArray)
        return data


class Command(ISerializable):
    def __init__(self):
        self.commandType = CommandType.None_
        self.option = 0

    @classmethod
    def getSize(cls):
        return 2

    def toArray(self):
        return pack('<BB', self.commandType.value, self.option)

    @classmethod
    def parse(cls, dataArray):
        data = Command()

        if len(dataArray) != cls.getSize():
            return None

        data.commandType, data.option = unpack('<BB', dataArray)
        data.commandType = CommandType(data.commandType)

        return data


class Command2(ISerializable):
    def __init__(self):
        self.command1 = Command()
        self.command2 = Command()

    @classmethod
    def getSize(cls):
        return Command.getSize() + Command.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.command1.toArray())
        dataArray.extend(self.command2.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = Command2()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = Command.getSize()
        data.command1 = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += Command.getSize()
        data.command2 = Command.parse(dataArray[indexStart:indexEnd])
        return data


class Command3(ISerializable):
    def __init__(self):
        self.command1 = Command()
        self.command2 = Command()
        self.command3 = Command()

    @classmethod
    def getSize(cls):
        return Command.getSize() + Command.getSize() + Command.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.command1.toArray())
        dataArray.extend(self.command2.toArray())
        dataArray.extend(self.command3.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = Command3()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = Command.getSize()
        data.command1 = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += Command.getSize()
        data.command2 = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += Command.getSize()
        data.command3 = Command.parse(dataArray[indexStart:indexEnd])
        return data


# Common End


# Light Start

class LightModeDrone(Enum):
    None_ = 0x00

    EyeNone = 0x10
    EyeHold = 0x11
    EyeMix = 0x12
    EyeFlicker = 0x13
    EyeFlickerDouble = 0x14
    EyeDimming = 0x15

    EndOfEye = 0x30

    ArmNone = 0x40
    ArmHold = 0x41
    ArmMix = 0x42
    ArmFlicker = 0x43
    ArmFlickerDouble = 0x44
    ArmDimming = 0x45
    ArmFlow = 0x46
    ArmFlowReverse = 0x47

    EndOfType = 0x48


class LEDColor(ISerializable):
    def __init__(self):
        self.r = 0
        self.g = 0
        self.b = 0

    @classmethod
    def getSize(cls):
        return 3

    def toArray(self):
        return pack('<BBB', self.r, self.g, self.b)

    @classmethod
    def parse(cls, dataArray):
        data = LEDColor()

        if len(dataArray) != cls.getSize():
            return None

        data.r, data.g, data.b = unpack('<BBB', dataArray)
        return data


class Color(Enum):
    AliceBlue = 0
    AntiqueWhite = 1
    Aqua = 2
    Aquamarine = 3
    Azure = 4
    Beige = 5
    Bisque = 6
    Black = 7
    BlanchedAlmond = 8
    Blue = 9
    BlueViolet = 10
    Brown = 11
    BurlyWood = 12
    CadetBlue = 13
    Chartreuse = 14
    Chocolate = 15
    Coral = 16
    CornflowerBlue = 17
    Cornsilk = 18
    Crimson = 19
    Cyan = 20
    DarkBlue = 21
    DarkCyan = 22
    DarkGoldenRod = 23
    DarkGray = 24
    DarkGreen = 25
    DarkKhaki = 26
    DarkMagenta = 27
    DarkOliveGreen = 28
    DarkOrange = 29
    DarkOrchid = 30
    DarkRed = 31
    DarkSalmon = 32
    DarkSeaGreen = 33
    DarkSlateBlue = 34
    DarkSlateGray = 35
    DarkTurquoise = 36
    DarkViolet = 37
    DeepPink = 38
    DeepSkyBlue = 39
    DimGray = 40
    DodgerBlue = 41
    FireBrick = 42
    FloralWhite = 43
    ForestGreen = 44
    Fuchsia = 45
    Gainsboro = 46
    GhostWhite = 47
    Gold = 48
    GoldenRod = 49
    Gray = 50
    Green = 51
    GreenYellow = 52
    HoneyDew = 53
    HotPink = 54
    IndianRed = 55
    Indigo = 56
    Ivory = 57
    Khaki = 58
    Lavender = 59
    LavenderBlush = 60
    LawnGreen = 61
    LemonChiffon = 62
    LightBlue = 63
    LightCoral = 64
    LightCyan = 65
    LightGoldenRodYellow = 66
    LightGray = 67
    LightGreen = 68
    LightPink = 69
    LightSalmon = 70
    LightSeaGreen = 71
    LightSkyBlue = 72
    LightSlateGray = 73
    LightSteelBlue = 74
    LightYellow = 75
    Lime = 76
    LimeGreen = 77
    Linen = 78
    Magenta = 79
    Maroon = 80
    MediumAquaMarine = 81
    MediumBlue = 82
    MediumOrchid = 83
    MediumPurple = 84
    MediumSeaGreen = 85
    MediumSlateBlue = 86
    MediumSpringGreen = 87
    MediumTurquoise = 88
    MediumVioletRed = 89
    MidnightBlue = 90
    MintCream = 91
    MistyRose = 92
    Moccasin = 93
    NavajoWhite = 94
    Navy = 95
    OldLace = 96
    Olive = 97
    OliveDrab = 98
    Orange = 99
    OrangeRed = 100
    Orchid = 101
    PaleGoldenRod = 102
    PaleGreen = 103
    PaleTurquoise = 104
    PaleVioletRed = 105
    PapayaWhip = 106
    PeachPuff = 107
    Peru = 108
    Pink = 109
    Plum = 110
    PowderBlue = 111
    Purple = 112
    RebeccaPurple = 113
    Red = 114
    RosyBrown = 115
    RoyalBlue = 116
    SaddleBrown = 117
    Salmon = 118
    SandyBrown = 119
    SeaGreen = 120
    SeaShell = 121
    Sienna = 122
    Silver = 123
    SkyBlue = 124
    SlateBlue = 125
    SlateGray = 126
    Snow = 127
    SpringGreen = 128
    SteelBlue = 129
    Tan = 130
    Teal = 131
    Thistle = 132
    Tomato = 133
    Turquoise = 134
    Violet = 135
    Wheat = 136
    White = 137
    WhiteSmoke = 138
    Yellow = 139
    YellowGreen = 140

    EndOfType = 141


class LightMode(ISerializable):
    def __init__(self):
        self.mode = LightModeDrone.None_
        self.color = Color.Black
        self.interval = 0

    @classmethod
    def getSize(cls):
        return 3

    def toArray(self):
        return pack('<BBB', self.mode.value, self.color.value, self.interval)

    @classmethod
    def parse(cls, dataArray):
        data = LightMode()

        if len(dataArray) != cls.getSize():
            return None

        data.mode, data.color, data.interval = unpack('<BBB', dataArray)
        data.mode = LightModeDrone(data.mode)
        data.color = Color(data.color)

        return data


class LightMode2(ISerializable):
    def __init__(self):
        self.lightMode1 = LightMode()
        self.lightMode2 = LightMode()

    @classmethod
    def getSize(cls):
        return LightMode.getSize() + LightMode.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightMode1.toArray())
        dataArray.extend(self.lightMode2.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightMode2()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightMode.getSize()
        data.lightMode1 = LightMode.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += LightMode.getSize()
        data.lightMode2 = LightMode.parse(dataArray[indexStart:indexEnd])
        return data


class LightModeCommand(ISerializable):
    def __init__(self):
        self.lightMode = LightMode()
        self.command = Command()

    @classmethod
    def getSize(cls):
        return LightMode.getSize() + Command.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightMode.toArray())
        dataArray.extend(self.command.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightModeCommand()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightMode.getSize()
        data.lightMode = LightMode.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += Command.getSize()
        data.command = Command.parse(dataArray[indexStart:indexEnd])
        return data


class LightModeCommandIr(ISerializable):
    def __init__(self):
        self.lightMode = LightMode()
        self.command = Command()
        self.irData = 0

    @classmethod
    def getSize(cls):
        return LightMode.getSize() + Command.getSize() + 4

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightMode.toArray())
        dataArray.extend(self.command.toArray())
        dataArray.extend(pack('<I', self.irData))
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightModeCommandIr()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightMode.getSize()
        data.lightMode = LightMode.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += Command.getSize()
        data.command = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += 4
        data.irData, = unpack('<I', dataArray[indexStart:indexEnd])

        return data


class LightModeColor(ISerializable):
    def __init__(self):
        self.mode = LightModeDrone.None_
        self.color = LEDColor()
        self.interval = 0

    @classmethod
    def getSize(cls):
        return 1 + LEDColor.getSize() + 1

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(pack('<B', self.mode.value))
        dataArray.extend(self.color.toArray())
        dataArray.extend(pack('<B', self.interval))
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightModeColor()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = 1
        data.mode = unpack('<B', dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += LEDColor.getSize()
        data.color = LEDColor.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += 1
        data.interval = unpack('<B', dataArray[indexStart:indexEnd])

        data.mode = LightModeDrone(data.mode)

        return data


class LightModeColor2(ISerializable):
    def __init__(self):
        self.lightModeColor1 = LightModeColor()
        self.lightModeColor2 = LightModeColor()

    @classmethod
    def getSize(cls):
        return LightModeColor.getSize() + LightModeColor.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightModeColor1.toArray())
        dataArray.extend(self.lightModeColor2.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightModeColor2()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightModeColor.getSize()
        data.lightModeColor1 = LightModeColor.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += LightModeColor.getSize()
        data.lightModeColor2 = LightModeColor.parse(dataArray[indexStart:indexEnd])
        return data


class LightEvent(ISerializable):
    def __init__(self):
        self.event = LightModeDrone.None_
        self.color = Color.Black
        self.interval = 0
        self.repeat = 0

    @classmethod
    def getSize(cls):
        return 4

    def toArray(self):
        return pack('<BBBB', self.event.value, self.color.value, self.interval, self.repeat)

    @classmethod
    def parse(cls, dataArray):
        data = LightEvent()

        if len(dataArray) != cls.getSize():
            return None

        data.event, data.color, data.interval, data.repeat = unpack('<BBBB', dataArray)
        data.event = LightModeDrone(data.event)
        data.color = Color(data.color)

        return data


class LightEvent2(ISerializable):
    def __init__(self):
        self.lightEvent1 = LightEvent()
        self.lightEvent2 = LightEvent()

    @classmethod
    def getSize(cls):
        return LightEvent.getSize() + LightEvent.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightEvent1.toArray())
        dataArray.extend(self.lightEvent2.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightEvent2()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightEvent.getSize()
        data.lightEvent1 = LightEvent.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += LightEvent.getSize()
        data.lightEvent2 = LightEvent.parse(dataArray[indexStart:indexEnd])
        return data


class LightEventCommand(ISerializable):
    def __init__(self):
        self.lightEvent = LightEvent()
        self.command = Command()

    @classmethod
    def getSize(cls):
        return LightEvent.getSize() + Command.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightEvent.toArray())
        dataArray.extend(self.command.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightEventCommand()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightEvent.getSize()
        data.lightEvent = LightEvent.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += Command.getSize()
        data.command = Command.parse(dataArray[indexStart:indexEnd])

        return data


class LightEventCommandIr(ISerializable):
    def __init__(self):
        self.lightEvent = LightEvent()
        self.command = Command()
        self.irData = 0

    @classmethod
    def getSize(cls):
        return LightEvent.getSize() + Command.getSize() + 4

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightEvent.toArray())
        dataArray.extend(self.command.toArray())
        dataArray.extend(pack('<I', self.irData))
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightEventCommandIr()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightEvent.getSize()
        data.lightEvent = LightEvent.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += Command.getSize()
        data.command = Command.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += 4
        data.irData, = unpack('<I', dataArray[indexStart:indexEnd])

        return data


class LightEventColor(ISerializable):
    def __init__(self):
        self.event = LightModeDrone.None_
        self.color = LEDColor()
        self.interval = 0
        self.repeat = 0

    @classmethod
    def getSize(cls):
        return 1 + LEDColor.getSize() + 2

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(pack('<B', self.event))
        dataArray.extend(self.color.toArray())
        dataArray.extend(pack('<B', self.interval))
        dataArray.extend(pack('<B', self.repeat))
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightEventColor()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = 1
        data.event = unpack('<B', dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd = LightEvent.getSize()
        data.color = LEDColor.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += 1
        data.interval = unpack('<B', dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += 1
        data.repeat = unpack('<B', dataArray[indexStart:indexEnd])

        data.event = LightModeDrone(data.event)

        return data


class LightEventColor2(ISerializable):
    def __init__(self):
        self.lightEventColor1 = LightEventColor()
        self.lightEventColor1 = LightEventColor()

    @classmethod
    def getSize(cls):
        return LightEventColor.getSize() + LightEventColor.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.lightEventColor1.toArray())
        dataArray.extend(self.lightEventColor2.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LightEventColor2()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = LightEventColor.getSize()
        data.lightEventColor1 = LightEventColor.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += LightEventColor.getSize()
        data.lightEventColor2 = LightEventColor.parse(dataArray[indexStart:indexEnd])
        return data


class LightModeDefaultColor(LightModeColor):
    pass


class LightModeDefaultColor2(LightModeColor2):
    pass


# Light End


# Information Start


class Address(ISerializable):
    def __init__(self):
        self.address = bytearray()

    @classmethod
    def getSize(cls):
        return 6

    def toArray(self):
        return self.address

    @classmethod
    def parse(cls, dataArray):
        data = Address()

        if len(dataArray) != cls.getSize():
            return None

        data.address = dataArray[0:6]
        return data


class State(ISerializable):
    def __init__(self):
        self.modeVehicle = ModeVehicle.None_

        self.modeSystem = ModeSystem.None_
        self.modeFlight = ModeFlight.None_
        self.modeDrive = ModeDrive.None_

        self.sensorOrientation = SensorOrientation.None_
        self.headless = Headless.None_
        self.battery = 0

    @classmethod
    def getSize(cls):
        return 7

    def toArray(self):
        return pack('<BBBBBBB', self.modeVehicle.value, self.modeSystem.value, self.modeFlight.value,
                    self.modeDrive.value, self.sensorOrientation.value, self.headless.value, self.battery)

    @classmethod
    def parse(cls, dataArray):
        data = State()

        if len(dataArray) != cls.getSize():
            return None

        data.modeVehicle, data.modeSystem, data.modeFlight, data.modeDrive, data.sensorOrientation, data.headless, data.battery = unpack(
            '<BBBBBBB', dataArray)

        data.modeVehicle = ModeVehicle(data.modeVehicle)

        data.modeSystem = ModeSystem(data.modeSystem)
        data.modeFlight = ModeFlight(data.modeFlight)
        data.modeDrive = ModeDrive(data.modeDrive)

        data.sensorOrientation = SensorOrientation(data.sensorOrientation)
        data.headless = Headless(data.headless)

        return data


class Attitude(ISerializable):
    def __init__(self):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    @classmethod
    def getSize(cls):
        return 6

    def toArray(self):
        return pack('<hhh', self.roll, self.pitch, self.yaw)

    @classmethod
    def parse(cls, dataArray):
        data = Attitude()

        if len(dataArray) != cls.getSize():
            return None

        data.roll, data.pitch, data.yaw = unpack('<hhh', dataArray)

        return data


class GyroBias(Attitude):
    pass


class TrimFlight(ISerializable, Move):
    def __init__(self):
        Move.__init__(self)

    @classmethod
    def getSize(cls):
        return 8

    def toArray(self):
        return pack('<hhhh', self._roll, self._pitch, self._yaw, self._throttle)

    @classmethod
    def parse(cls, dataArray):
        data = TrimFlight()

        if len(dataArray) != cls.getSize():
            return None

        data._roll, data._pitch, data._yaw, data._throttle = unpack('<hhhh', dataArray)
        return data


class TrimDrive(ISerializable):
    def __init__(self):
        self.wheel = 0

    @classmethod
    def getSize(cls):
        return 2

    def toArray(self):
        return pack('<h', self.wheel)

    @classmethod
    def parse(cls, dataArray):
        data = TrimDrive()

        if len(dataArray) != cls.getSize():
            return None

        data.wheel = unpack('<h', dataArray)
        return data


class TrimAll(ISerializable):
    def __init__(self):
        self.flight = TrimFlight()
        self.drive = TrimDrive()

    @classmethod
    def getSize(cls):
        return TrimFlight.getSize() + TrimDrive.getSize()

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.flight.toArray())
        dataArray.extend(self.drive.toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = TrimAll()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = TrimFlight.getSize()
        data.flight = TrimFlight.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += TrimDrive.getSize()
        data.drive = TrimDrive.parse(dataArray[indexStart:indexEnd])
        return data


class CountFlight(ISerializable):
    def __init__(self):
        self.timeFlight = 0

        self.countTakeOff = 0
        self.countLanding = 0
        self.countAccident = 0

    @classmethod
    def getSize(cls):
        return 14

    def toArray(self):
        return pack('<QHHH', self.timeFlight, self.countTakeOff, self.countLanding, self.countAccident)

    @classmethod
    def parse(cls, dataArray):
        data = CountFlight()

        if len(dataArray) != cls.getSize():
            return None

        data.timeFlight, data.countTakeOff, data.countLanding, data.countAccident = unpack('<QHHH', dataArray)

        return data


class CountDrive(ISerializable):
    def __init__(self):
        self.timeDrive = 0

        self.countAccident = 0

    @classmethod
    def getSize(cls):
        return 10

    def toArray(self):
        return pack('<QH', self.timeDrive, self.countAccident)

    @classmethod
    def parse(cls, dataArray):
        data = CountDrive()

        if len(dataArray) != cls.getSize():
            return None

        data.timeDrive, data.countAccident = unpack('<QH', dataArray)

        return data


class IrMessage(ISerializable):
    def __init__(self):
        self.direction = Direction.None_
        self.irData = 0

    @classmethod
    def getSize(cls):
        return 5

    def toArray(self):
        return pack('<BI', self.direction.value, self.irData)

    @classmethod
    def parse(cls, dataArray):
        data = IrMessage()

        if len(dataArray) != cls.getSize():
            return None

        data.direction, data.irData = unpack('<BI', dataArray)

        data.direction = Direction(data.direction)

        return data


class Imu(ISerializable):
    def __init__(self):
        self.accelX = 0
        self.accelY = 0
        self.accelZ = 0
        self.gyroRoll = 0
        self.gyroPitch = 0
        self.gyroYaw = 0
        self.angleRoll = 0
        self.anglePitch = 0
        self.angleYaw = 0

    @classmethod
    def getSize(cls):
        return 18

    def toArray(self):
        return pack('<hhhhhhhhh', self.accelX, self.accelY, self.accelZ, self.gyroRoll, self.gyroPitch, self.gyroYaw,
                    self.angleRoll, self.anglePitch, self.angleYaw)

    @classmethod
    def parse(cls, dataArray):
        data = Imu()

        if len(dataArray) != cls.getSize():
            return None

        data.accelX, data.accelY, data.accelZ, data.gyroRoll, data.gyroPitch, data.gyroYaw, data.angleRoll, data.anglePitch, data.angleYaw = unpack(
            '<hhhhhhhhh', dataArray)

        return data


class Pressure(ISerializable):
    def __init__(self):
        self.d1 = 0
        self.d2 = 0
        self.temperature = 0
        self.pressure = 0

    @classmethod
    def getSize(cls):
        return 16

    def toArray(self):
        return pack('<iiii', self.d1, self.d2, self.temperature, self.pressure)

    @classmethod
    def parse(cls, dataArray):
        data = Pressure()

        if len(dataArray) != cls.getSize():
            return None

        data.d1, data.d2, data.temperature, data.pressure = unpack('<iiii', dataArray)

        return data


class ImageFlow(ISerializable):
    def __init__(self):
        self.positionX = 0
        self.positionY = 0

    @classmethod
    def getSize(cls):
        return 8

    def toArray(self):
        return pack('<ii', self.positionX, self.positionY)

    @classmethod
    def parse(cls, dataArray):
        data = ImageFlow()

        if len(dataArray) != cls.getSize():
            return None

        data.positionX, data.positionY = unpack('<ii', dataArray)

        return data


class ButtonFlag(Enum):
    None_ = 0x00

    Reset = 0x01


class Button(ISerializable):
    def __init__(self):
        self.button = 0

    @classmethod
    def getSize(cls):
        return 1

    def toArray(self):
        return pack('<B', self.button)

    @classmethod
    def parse(cls, dataArray):
        data = Button()

        if len(dataArray) != cls.getSize():
            return None

        data.button = unpack('<B', dataArray)

        return data


class Battery(ISerializable):
    def __init__(self):
        self.adjustGradient = 0
        self.adjustYIntercept = 0
        self.gradient = 0
        self.yIntercept = 0
        self.flagBatteryCalibration = False
        self.batteryRaw = 0
        self.batteryPercent = 0
        self.voltage = 0

    @classmethod
    def getSize(cls):
        return 16

    def toArray(self):
        return pack('<hhhhBibh', self.adjustGradient, self.adjustYIntercept, self.gradient, self.yIntercept,
                    self.flagBatteryCalibration, self.batteryRaw, self.batteryPercent, self.voltage)

    @classmethod
    def parse(cls, dataArray):
        data = Battery()

        if len(dataArray) != cls.getSize():
            return None

        data.adjustGradient, data.adjustYIntercept, data.gradient, data.yIntercept, data.flagBatteryCalibration, data.batteryRaw, data.batteryPercent, data.voltage = unpack(
            '<hhhhBibh', dataArray)

        data.flagBatteryCalibration = bool(data.flagBatteryCalibration)

        return data


class MotorBlock(ISerializable):
    def __init__(self):
        self.forward = 0
        self.reverse = 0

    @classmethod
    def getSize(cls):
        return 4

    def toArray(self):
        return pack('<hh', self.forward, self.reverse)

    @classmethod
    def parse(cls, dataArray):
        data = MotorBlock()

        if len(dataArray) != cls.getSize():
            return None

        data.forward, data.reverse = unpack('<hh', dataArray)

        return data


class Motor(ISerializable):
    def __init__(self):
        self.motor = []
        self.motor.append(MotorBlock())
        self.motor.append(MotorBlock())
        self.motor.append(MotorBlock())
        self.motor.append(MotorBlock())

    @classmethod
    def getSize(cls):
        return MotorBlock.getSize() * 4

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.motor[0].toArray())
        dataArray.extend(self.motor[1].toArray())
        dataArray.extend(self.motor[2].toArray())
        dataArray.extend(self.motor[3].toArray())
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = Motor()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = MotorBlock.getSize()
        data.motor[0] = MotorBlock.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += MotorBlock.getSize()
        data.motor[1] = MotorBlock.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += MotorBlock.getSize()
        data.motor[2] = MotorBlock.parse(dataArray[indexStart:indexEnd])
        indexStart = indexEnd
        indexEnd += MotorBlock.getSize()
        data.motor[3] = MotorBlock.parse(dataArray[indexStart:indexEnd])

        return data


class Temperature(ISerializable):
    def __init__(self):
        self.imu = 0
        self.pressure = 0

    @classmethod
    def getSize(cls):
        return 8

    def toArray(self):
        return pack('<ii', self.imu, self.pressure)

    @classmethod
    def parse(cls, dataArray):
        data = Temperature()

        if len(dataArray) != cls.getSize():
            return None

        data.imu, data.pressure = unpack('<ii', dataArray)

        return data


class Range(ISerializable):
    def __init__(self):
        self.left = 0
        self.front = 0
        self.right = 0
        self.rear = 0
        self.top = 0
        self.bottom = 0

    @classmethod
    def getSize(cls):
        return 12

    def toArray(self):
        return pack('<HHHHHH', self.left, self.front, self.right, self.rear, self.top, self.bottom)

    @classmethod
    def parse(cls, dataArray):
        data = Range()

        if len(dataArray) != cls.getSize():
            return None

        data.left, data.front, data.right, data.rear, data.top, data.bottom = unpack('<HHHHHH', dataArray)

        return data


# Information End


# Update


class UpdateLookupTarget(ISerializable):
    def __init__(self):
        self.deviceType = DeviceType.None_

    @classmethod
    def getSize(cls):
        return 4

    def toArray(self):
        return pack('<I', self.deviceType.value)

    @classmethod
    def parse(cls, dataArray):
        data = UpdateLookupTarget()

        if len(dataArray) != cls.getSize():
            return None

        data.deviceType, = unpack('<I', dataArray)
        data.deviceType = DeviceType(data.deviceType)

        return data


class UpdateInformation(ISerializable):
    def __init__(self):
        self.modeUpdate = ModeUpdate.None_  # 현재 업데이트 모드

        self.deviceType = DeviceType.None_  # 장치 Type
        self.imageType = ImageType.None_  # 현재 펌웨어의 이미지 타입(0은 Debug 또는 Release, 1 -> Image A, 2 -> Image B)
        self.version = 0  # 현재 펌웨어의 버젼

        self.year = 0  # 빌드 년
        self.month = 0  # 빌드 월
        self.day = 0  # 빌드 일

    @classmethod
    def getSize(cls):
        return 11

    def toArray(self):
        return pack('<BIBHBBB', self.modeUpdate.value, self.deviceType.value, self.imageType.value, self.version,
                    self.year, self.month, self.day)

    @classmethod
    def parse(cls, dataArray):
        data = UpdateInformation()

        if len(dataArray) != cls.getSize():
            return None

        data.modeUpdate, data.deviceType, data.imageType, data.version, data.year, data.month, data.day = unpack(
            '<BIBHBBB', dataArray)

        data.modeUpdate = ModeUpdate(data.modeUpdate)
        data.deviceType = DeviceType(data.deviceType)
        data.imageType = ImageType(data.imageType)

        return data


class Update(ISerializable):
    def __init__(self):
        self.indexBlock = 0
        self.dataArray = bytearray()

    @classmethod
    def getSize(cls):
        return 18

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(pack('<H', self.indexBlock))
        dataArray.extend(self.dataArray)
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = Update()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = 1
        data.indexBlock, = unpack('<H', (dataArray[indexStart:indexEnd]))
        indexStart = indexEnd
        indexEnd += 6
        data.dataArray = dataArray[indexStart:indexEnd]

        return data


class UpdateLocationCorrect(ISerializable):
    def __init__(self):
        self.indexBlockNext = 0

    @classmethod
    def getSize(cls):
        return 2

    def toArray(self):
        return pack('<H', self.indexBlockNext)

    @classmethod
    def parse(cls, dataArray):
        data = UpdateLocationCorrect()

        if len(dataArray) != cls.getSize():
            return None

        data.indexBlockNext, = unpack('<H', dataArray)

        return data


# Update End


# Link Start


class LinkState(ISerializable):
    def __init__(self):
        self.modeLink = ModeLink.None_
        self.modeLinkBroadcast = ModeLinkBroadcast.None_

    @classmethod
    def getSize(cls):
        return 2

    def toArray(self):
        return pack('<BB', self.modeLink, self.modeLinkBroadcast)

    @classmethod
    def parse(cls, dataArray):
        data = LinkState()

        if len(dataArray) != cls.getSize():
            return None

        data.modeLink, data.modeLinkBroadcast = unpack('<BB', dataArray)
        data.modeLink = ModeLink(data.modeLink)
        data.modeLinkBroadcast = ModeLinkBroadcast(data.modeLinkBroadcast)

        return data


class LinkEvent(ISerializable):
    def __init__(self):
        self.eventLink = EventLink.None_
        self.eventResult = 0

    @classmethod
    def getSize(cls):
        return 2

    def toArray(self):
        return pack('<BB', self.modeLink.value, self.eventResult)

    @classmethod
    def parse(cls, dataArray):
        data = LinkEvent()

        if len(dataArray) != cls.getSize():
            return None

        data.eventLink, data.eventResult = unpack('<BB', dataArray)
        data.eventLink = EventLink(data.eventLink)

        return data


class LinkEventAddress(ISerializable):
    def __init__(self):
        self.eventLink = EventLink.None_
        self.eventResult = 0
        self.address = bytearray()

    @classmethod
    def getSize(cls):
        return 8

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(pack('<BB', self.modeLink.value, self.eventResult))
        dataArray.extend(self.address)
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LinkEventAddress()

        if len(dataArray) != cls.getSize():
            return None

        data.eventLink, data.eventResult = unpack('<BB', dataArray[0:2])
        data.address = dataArray[2:8]

        data.eventLink = EventLink(data.eventLink)

        return data


class LinkRssi(ISerializable):
    def __init__(self):
        self.rssi = 0

    @classmethod
    def getSize(cls):
        return 1

    def toArray(self):
        return pack('<b', self.rssi)

    @classmethod
    def parse(cls, dataArray):
        data = LinkRssi()

        if len(dataArray) != cls.getSize():
            return None

        data.rssi, = unpack('<b', dataArray)

        return data


class LinkDiscoveredDevice(ISerializable):
    def __init__(self):
        self.index = 0
        self.address = bytearray()
        self.name = ""
        self.rssi = 0

    @classmethod
    def getSize(cls):
        return 28

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(pack('<B', self.index))
        dataArray.extend(self.address)
        dataArray.extend(self.name.encode('ascii',
                                          'ignore'))  # 문자열 데이터의 길이가 고정이기 때문에 그에 대한 처리가 필요하나, 파이썬에서 이 데이터를 전송하지는 않기 때문에 일단 이대로 둠
        dataArray.extend(pack('<b', self.rssi))
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = LinkEventAddress()

        if len(dataArray) != cls.getSize():
            return None

        indexStart = 0
        indexEnd = 1
        data.index, = unpack('<B', (dataArray[indexStart:indexEnd]))
        indexStart = indexEnd
        indexEnd += 6
        data.address = dataArray[indexStart:indexEnd]
        indexStart = indexEnd
        indexEnd += 20
        data.name = dataArray[indexStart:indexEnd].decode()
        indexStart = indexEnd
        indexEnd += 1
        data.rssi, = unpack('<b', (dataArray[indexStart:indexEnd]))

        return data


class LinkPasscode(ISerializable):
    def __init__(self):
        self.passcode = 0

    @classmethod
    def getSize(cls):
        return 4

    def toArray(self):
        return pack('<I', self.passcode)

    @classmethod
    def parse(cls, dataArray):
        data = LinkPasscode()

        if len(dataArray) != cls.getSize():
            return None

        data.passcode, = unpack('<I', dataArray)

        return data


# Link End


# Message Start


class Message():
    def __init__(self):
        self.message = ""

    def getSize(self):
        return len(self.message)

    def toArray(self):
        dataArray = bytearray()
        dataArray.extend(self.message.encode('ascii', 'ignore'))
        return dataArray

    @classmethod
    def parse(cls, dataArray):
        data = Message()

        if len(dataArray) == 0:
            return ""

        data.message = dataArray[0:len(dataArray)].decode()

        return data

# Message End
