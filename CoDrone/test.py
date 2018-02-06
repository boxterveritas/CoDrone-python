from codrone import *
flag = 0
battery = 0
IR = 0


def colorTest(drone):
    """
    print("eye : Green, arm : Blue")
    # set the eye color to Green
    drone.setEyeRGB(0,255,0)
    sleep(0.03)
    # set the arm color to Blue
    drone.setArmRGB(0,0,255)
    sleep(0.03)

    print("eye : Blinking Blue, arm : DoubleBlinking Blue")
    #drone.setLEDMode(Mode.Blinking)
    #drone.setEyeRGB(0,0,255)
    drone.setLEDMode(Mode.DoubleBlink)
    drone.setArmRGB(0,0,255)
    sleep(3)

    print("eye : Mix red, arm : Flow red")
    drone.setLEDMode(Mode.Mix)
    drone.setEyeRGB(255,0,0)
    sleep(3)
    drone.setLEDMode(Mode.Flow)
    drone.setArmRGB(255,0,0)
    sleep(3)

    print("eye : Reverse Flow Green, arm : Pulsing Green")
    drone.setLEDMode(Mode.ReverseFlow)
    drone.setEyeRGB(0,255,0)
    sleep(3)
    drone.setLEDMode(Mode.Pulsing)
    drone.setArmRGB(0,255,0)
    sleep(3)

    print("eye : Off , arm : Blue")
    drone.setLEDMode(Mode.Off)
    drone.setArmRGB(0,0,255)
    sleep(5)
    """

    #drone.resetLED()
    #sleep(5)
    for i in range(10):
        print("Blue All")
        drone.setAllRGB(0,0,255)
        sleep(2)

        print("green All")
        drone.setAllRGB(0,255,0)
        sleep(2)

        print("red All")
        drone.setAllRGB(255,0,0)
        sleep(2)

def dataTest(drone):
    print("---- START Data Test")
    for i in range(2):
        print(drone.getHeight())
        print(drone.getBatteryPercentage())
        print(drone.getBatteryVoltage())
        temp = drone.getAccelerometer()
        print(temp.X, temp.Y, temp.Z)
        temp = drone.getAngularSpeed()
        print(temp.Roll, temp.Pitch, temp.Yaw)
        print(drone.getPressure())
        print(drone.getState())
        temp = drone.getTrim()
        print(temp.Roll, temp.Pitch, temp.Yaw, temp.Throttle)
        print(drone.getDroneTemp())
        temp = drone.getGyroAngles()
        print(temp.Roll, temp.Pitch, temp.Yaw)
        temp = drone.getOptFlowPosition()
        print(temp.X, temp.Y)
        sleep(2)
    print("---- DONE Data Test")

def printDataTime(func):
    startTime = time.time()
    h = func()
    endTime = time.time() - startTime
    print(func, ":", endTime, "## ", h)
    return h

def flightTest(drone):
    print("---- START Flight Test")
    print("take off for 5 sec")
    drone.takeoff()

    print("hovering for 3 sec")
    drone.hover(3)

    sleep(3)

    print("landing")
    drone.land()
    sleep(5)

    print("take off for 5 sec")
    drone.takeoff()

    print("emergency stop")
    drone.emergencyStop()

    print("---- DONE Flight Test")

def moveTest(drone):
    print("---- START Move Test")

    print("down for 2sec")
    drone.sendControl(0, 0, 0, -30)
    sleep(2)

    print("forward for 2sec")
    drone.sendControlDuration(0, 30, 0, 0, 2)

    print("hovering for 2sec")
    sleep(2)

    print("backward for 2sec")
    drone.sendControlDuration(0,-30,0,0,2)

    print("hovering for 2sec")
    sleep(2)

    print("up for 2sec")
    drone.setThrottle(30)
    drone.move()
    sleep(2)

    print("down for 2sec")
    drone.setThrottle(-30)
    drone.move(2)

    print("hovering for 2sec")
    sleep(2)

    print("right for 2sec")
    drone.move(2, 30, 0, 0, 0)

    print("hovering for 2sec")
    sleep(2)

    print("left for 2sec")
    drone.move(2, -30, 0, 0, 0)

    print("hovering for 2sec")
    sleep(2)

    print("---- DONE Move Test")


def goTest(drone):
    print("---- START Go Test")

    print("go left for 2 sec")
    drone.go(Direction.Left, 2, 50)

    print("go right for 2 sec")
    drone.go(Direction.Right, 2, 50)

    print("go up for 2 sec")
    drone.go(Direction.Up, 2, 50)

    print("go down for 2 sec")
    drone.go(Direction.Down, 2, 50)

    print(drone.getRoll(), drone.getPitch(), drone.getYaw(), drone.getThrottle())
    drone.setRoll(50)
    drone.setPitch(50)
    drone.setYaw(50)
    drone.setThrottle(50)
    print(drone.getRoll(), drone.getPitch(), drone.getYaw(), drone.getThrottle())

    print("---- DONE Go Test")

def combineTest(drone):
    drone.takeoff()
    height = drone.getHeight()
    while(height < 1300):
        drone.go(Direction.Up)
        height = drone.getHeight()
    while(height > 500):
        drone.go(Direction.Down)
        height = drone.getHeight()
    drone.land()

def testDrone():
    drone = CoDrone()

    while(not drone.isConnected()):
        print("printing")
        drone.connect()
        sleep(3)

    #flightTest(drone)
    #drone.takeoff()
    #drone.hover(0)
    colorTest(drone)

    moveTest(drone)
    goTest(drone)

    drone.takeoff()
    drone.go(Direction.Up)
    print("hover")
    drone.hover(2)
    dataTest(drone)
    drone.land()
    drone.close()

def testCoDrone():
    drone = CoDrone()

    while not drone.isConnected():
        print("printing")
        drone.connect()
        sleep(3)


testDrone()
