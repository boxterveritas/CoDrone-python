from codrone import CoDrone
from protocol import *
import time
from time import sleep


def test_STARTSTOP(drone):
    print("land > takeoff > hover > emergencyStop")
    drone.land()
    print("takeoff")
    drone.takeoff()
    drone.hover(3)
    drone.emergencyStop()


def test_move(drone):
    drone.move(1, 0, 0, 0, 10)
    drone.move(1, 0, 0, 0, -10)

    drone.setThrottle(10)
    drone.setPitch(10)
    drone.move(1)

    drone.setThrottle(-10)
    drone.setPitch(-10)
    drone.move(1)


def test_go(drone):
    drone.go(Direction.DOWN)
    drone.go(Direction.UP)

    drone.go(Direction.LEFT, 1)
    drone.go(Direction.RIGHT, 1)

    drone.go(Direction.FORWARD, 1, 50)
    drone.go(Direction.BACKWARD, 1, 50)


def test_turn(drone):
    drone.turn(Direction.LEFT)
    drone.turn(Direction.RIGHT)

    drone.turn(Direction.LEFT, 3)
    drone.turn(Direction.RIGHT, 3)

    drone.turn(Direction.LEFT, 3, 100)
    drone.turn(Direction.RIGHT, 3, 100)


def test_turnDegree(drone):
    drone.turnDegree(Direction.LEFT, Degree.ANGLE_30)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_30)

    drone.turnDegree(Direction.LEFT, Degree.ANGLE_45)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_45)

    drone.turnDegree(Direction.LEFT, Degree.ANGLE_60)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_60)

    drone.turnDegree(Direction.LEFT, Degree.ANGLE_90)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_90)

    drone.turnDegree(Direction.LEFT, Degree.ANGLE_120)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_120)

    drone.turnDegree(Direction.LEFT, Degree.ANGLE_135)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_135)

    drone.turnDegree(Direction.LEFT, Degree.ANGLE_150)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_150)

    drone.turnDegree(Direction.LEFT, Degree.ANGLE_180)
    drone.turnDegree(Direction.RIGHT, Degree.ANGLE_180)


def test_rotate180(drone):
    drone.rotate180()
    drone.rotate180()


def test_goToHeight(drone):
    start_time = time.time()
    drone.goToHeight(100)
    print("100 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(300)
    print("300 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(500)
    print("500 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(1000)
    print("1000 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(1500)
    print("1500 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(2000)
    print("2000 -- {}".format(time.time() - start_time))


### SENSORS

def test_getData(drone):
    for i in range(10):
        startTime = time.time()
        print("Height :", drone.getHeight())
        print("battery percentage :", drone.getBatteryPercentage())
        print("battery voltage :", drone.getBatteryVoltage())
        temp = drone.getAccelerometer()
        print("accel :", temp.X, temp.Y, temp.Z)
        temp = drone.getAngularSpeed()
        print("angular speed :", temp.Roll, temp.Pitch, temp.Yaw)
        print("pressure :", drone.getPressure())
        print("state :", drone.getState())
        temp = drone.getTrim()
        print("trim :", temp.Roll, temp.Pitch, temp.Yaw, temp.Throttle)
        print("temperature :", drone.getDroneTemp())
        temp = drone.getGyroAngles()
        print("gyro :", temp.Roll, temp.Pitch, temp.Yaw)
        temp = drone.getOptFlowPosition()
        print("image flow :", temp.X, temp.Y)
        print(">>{}".format(time.time() - startTime))
        sleep(2)

def test_LEDs(drone):
    print("START Color Test ----")

    print("Default setting")
    drone.setArmDefaultRGB(0, 0, 255)
    drone.setArmDefaultMode(Mode.FLOW)
    drone.setEyeDefaultRGB(0, 0, 255)
    drone.setEyeDefaultMode(Mode.BLINKING)
    # drone.resetDefaultLED()

    sleep(3)  # check time

    print("eye : Blinking Green, arm : DoubleBlinking Green")
    drone.setEyeMode(Mode.BLINKING)

    sleep(3)  # check time

    drone.setEyeRGB(0, 255, 0)

    sleep(3)  # check time

    drone.setArmMode(Mode.DOUBLE_BLINK)

    sleep(3)  # check time

    drone.setArmRGB(0, 255, 0)

    sleep(3)  # check time

    print("eye : Mix, arm : Flow red")
    drone.setEyeMode(Mode.MIX)
    drone.setEyeRGB(255, 0, 0)

    drone.setArmMode(Mode.FLOW)
    drone.setArmRGB(255, 0, 0)

    # for seeing color
    sleep(3)

    print("eye : Reverse Flow Green, arm : Pulsing Green")
    drone.setEyeMode(Mode.PULSING)
    drone.setEyeRGB(0, 255, 0)

    drone.setArmMode(Mode.REVERSE_FLOW)
    drone.setArmRGB(0, 255, 0)

    # for seeing color
    sleep(3)

    print("eye : Off , arm : Hold Blue")
    drone.setEyeMode(Mode.OFF)
    drone.setArmMode(Mode.HOLD)
    drone.setArmRGB(0, 0, 255)

    sleep(3)
    print("all green!! ")
    drone.setAllRGB(0, 255, 0)

