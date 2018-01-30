from codrone import *

flag = 0
battery = 0
IR = 0


def colorTest(drone):
    drone.setEyeRGB(0,255,0)
    sleep(2)
    drone.setArmRGB(0,0,255)
    sleep(3)

def dataTest(drone):
    printData(drone.getHeight)
    printData(drone.getBatteryPercentage)
    printData(drone.getBatteryVoltage)
    printData(drone.getAccelerometer)
    printData(drone.getAngles)
    printData(drone.getPressure)
    printData(drone.getState)
    printData(drone.getTrim)
    printData(drone.getTemperature)
    for i in range(100):
        printData(drone.getAngles)
        sleep(1)
def printData(func):
    startTime = time.time()
    h = func()
    endTime = time.time() - startTime
    print(func, ":", h, endTime)

def func(fuck):
    return (1,2,3,4)

def moveTest(drone):
    print("move 3")
    drone.sendControl(0,0,0,-30)
    sleep(2)
    drone.sendControlDuration(0,30,0,0,2)
    drone.sendControlDuration(0,-30,0,0,2)
    drone.setThrottle(30)
    drone.move()
    sleep(3)
    print("move 3")
    drone.setThrottle(-30)
    drone.move(3)
    print("move 3")
    drone.move(3,0,0,0,30)
    print(drone.getThrottle())

def goTest(drone):
    print("go left for 2 sec")
    drone.go(Direction.Left, 2, 50)
    print("go right for 2 sec")
    drone.go(Direction.Right, 2, 50)
    print("go up for 2 sec")
    drone.go(Direction.Up, 2, 50)
    print("go down for 2 sec")
    drone.go(Direction.Down, 2, 50)
    print(drone.getRoll(), drone.getPitch(), drone.getYaw(), drone.getThrottle())

def combineTest(drone):
    drone.takeoff()
    height = drone.getHeight()
    while(height < 1300):
        print(height)
        drone.go(Direction.Up)
        height = drone.getHeight()
    while(height > 500):
        print(height)
        drone.go(Direction.Down)
        height = drone.getHeight()
    drone.land()

def testDrone():
    drone = CoDrone()

    while(not drone.isConnected()):
        print("printing")
        drone.connect()
        sleep(3)

    colorTest(drone)
    drone.close()

def testCoDrone():
    drone = CoDrone()

    while(not drone.isConnected()):
        print("printing")
        drone.connect()
        sleep(3)

    drone.sendTakeOff()
    sleep(3)
    drone.sendControl(0,0,0,-30)
    sleep(2)
    drone.sendControl(0,0,0,30)
    sleep(2)

testDrone()