from codrone import *
import matplotlib.pyplot as plt
import math
flag = 0
battery = 0
IR = 0


def colorTest(drone):
    print("START Color Test ----")

    print("Default setting")
    drone.setArmDefaultRGB(0,0,255)
    drone.setArmDefaultMode(Mode.Flow)
    drone.setEyeDefaultRGB(0,0,255)
    drone.setEyeDefaultMode(Mode.Blinking)
    #drone.resetDefaultLED()

    sleep(3)     #check time

    print("eye : Blinking Green, arm : DoubleBlinking Green")
    drone.setEyeMode(Mode.Blinking)

    sleep(3)     #check time

    drone.setEyeRGB(0,255,0)

    sleep(3)     #check time

    drone.setArmMode(Mode.DoubleBlink)

    sleep(3)     #check time

    drone.setArmRGB(0,255,0)

    sleep(3)     #check time

    print("eye : Mix, arm : Flow red")
    drone.setEyeMode(Mode.Mix)
    drone.setEyeRGB(255,0,0)

    drone.setArmMode(Mode.Flow)
    drone.setArmRGB(255,0,0)

    # for seeing color
    sleep(3)

    print("eye : Reverse Flow Green, arm : Pulsing Green")
    drone.setEyeMode(Mode.Pulsing)
    drone.setEyeRGB(0,255,0)

    drone.setArmMode(Mode.ReverseFlow)
    drone.setArmRGB(0,255,0)

    # for seeing color
    sleep(3)

    print("eye : Off , arm : Hold Blue")
    drone.setEyeMode(Mode.Off)
    drone.setArmMode(Mode.Hold)
    drone.setArmRGB(0,0,255)

    sleep(3)
    print("all green!! ")
    drone.setAllRGB(0,255,0)


def dataTest(drone):
    print("---- START Data Test")
    for i in range(2):
        print("Height :" , drone.getHeight())
        print("battery percentage :" ,drone.getBatteryPercentage())
        print("battery voltage :" ,drone.getBatteryVoltage())
        temp = drone.getAccelerometer()
        print("accel :" ,temp.X, temp.Y, temp.Z)
        temp = drone.getAngularSpeed()
        print("angular speed :" ,temp.Roll, temp.Pitch, temp.Yaw)
        print("pressure :" ,drone.getPressure())
        print("state :" ,drone.getState())
        temp = drone.getTrim()
        print("trim :" ,temp.Roll, temp.Pitch, temp.Yaw, temp.Throttle)
        print("temperature :" ,drone.getDroneTemp())
        temp = drone.getGyroAngles()
        print("gyro :" ,temp.Roll, temp.Pitch, temp.Yaw)
        temp = drone.getOptFlowPosition()
        print("image flow :" ,temp.X, temp.Y)
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
    """
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
    """
    print("up for 2sec")
    drone.setThrottle(50)
    drone.move()
    sleep(2)

    print("down for 2sec")
    drone.setThrottle(-50)
    drone.move(2)

    print("hovering for 2sec")
    sleep(2)

    print("right for 2sec")
    drone.move(1, 30, 0, 0, 0)

    print("hovering for 2sec")
    sleep(2)

    print("left for 2sec")
    drone.move(1, -30, 0, 0, 0)

    print("hovering for 2sec")
    sleep(2)

    print("---- DONE Move Test")


def goTest(drone):
    print("---- START Go Test")

    print("go left for 2 sec")
    drone.go(Direction.Left, 2, 20)

    print("go right for 2 sec")
    drone.go(Direction.Right, 2, 20)

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

def onTest(drone):
    def blueArm():
        drone.setArmRGB(0,0,255)
        print("blueArm, takeoff")
    def blueEye():
        drone.setEyeRGB(0,0,255)
        print("blueEye, crash")
    def doubleBlinkEye():
        drone.setEyeMode(Mode.DoubleBlink)
        print("doubleBlinkEye, ready")
    def greenArm():
        drone.setArmRGB(0,255,0)
        print("greenArm, emergency")
    def greenEye():
        drone.setEyeRGB(0,255,0)
        print("greenEye, upsidedown")
    def doubleBlinkArm():
        drone.setArmMode(Mode.DoubleBlink)
        print("doubleBlinkArm, fly")
    def RedBlinkArm():
        drone.setArmRGB(255,0,0)
        drone.setArmMode(Mode.Blinking)
        print("redArm, low Battery")

    while(not drone.isConnected()):
        print("printing")
        drone.connect()
        sleep(3)

    #drone.onTakeoff(blueArm)
    #drone.onCrash(blueEye)
    #drone.onReady(doubleBlinkEye)
    #drone.onEmergencyStop(greenArm)
    #drone.onUpsideDown(greenEye)
    #drone.onFlying(doubleBlinkArm)
    drone.onLowBattery(RedBlinkArm)

"""
for (float z = -1.0; z <= 1.0; z += intervalZ) {
		for (float d = 0; d <= M_PI2; d += intervalD) {
			float y = sqrt(1.0 - pow(z, 2)) * cos(d);
			float x = sqrt(1.0 - pow(z, 2)) * sin(d);
"""
def testCircle(drone):
    x_list = []
    y_list = []
    num = 8
    interval = (math.pi * 2) / num
    for d in [interval * i for i in range(num)]:
        x_list.append(int((math.cos(d)*100-100)/2))
        y_list.append(int(math.sin(d)*100/2))

    for i in range(1,num):
        x_list[i-1] = x_list[i] - x_list[i-1]
        y_list[i-1] = y_list[i] - y_list[i-1]
    x_list[num-1] = -x_list[num-1]
    y_list[num-1] = -y_list[num-1]
    print(x_list, y_list)
    drone.takeoff()
    for i in range(num):
        drone.move(0.005,y_list[i],x_list[i],0,0)
    drone.hover(5)
    drone.land()

def testL(drone):
    drone.takeoff()
    drone.move(0.1, 0,-80,0,0)
    drone.move(0.1, 30,0,0,0)
    drone.hover(3)
    drone.land()

def testSquare(drone):
    drone.takeoff()
    drone.move(1, 30,0,0,0)
    drone.move(1, 0,30,0,0)
    drone.move(1, -30,0,0,0)
    drone.move(1, 0,-30,0,0)


    drone.land()

def testZigZag(drone):
    drone.takeoff()
    drone.move(1, -30,20,0,0)
    drone.move(1, 30,20,0,0)
    drone.move(1, -30,20,0,0)
    drone.move(1, 30,20,0,0)
    drone.land()

def testHeight(drone):
    drone.takeoff()
    if(drone.getHeight() < 1500):
        drone.move(0.2, 0,0,0,50)
    drone.land()

def testTurn(drone):
    drone.takeoff()
    for i in range(10,100, 10):
        print(i)
        drone.sendControlDuration(0, 0, i, 0, 1)
        drone.hover(3)

def testDrone():
    drone = CoDrone()
    while not drone.isConnected():
        drone.connect("COM10", "PETRONE 6187")
        sleep(3)
    print(drone.getBatteryPercentage())
    #print(drone.getHeight())
    #onTest(drone)
    #colorTest(drone)
    #testCircle(drone)
    #sleep(3)
    #testL(drone)
    #sleep(3)
    #testSquare(drone)
    #sleep(3)
    #testHeight(drone)
    #sleep(3)
    testZigZag(drone)
    sleep(3)
    #flightTest(drone)
    #drone.takeoff()
    #drone.hover(0)
    #colorTest(drone)
    #moveTest(drone)
    #goTest(drone)

    drone.close()

def testCoDrone():
    drone = CoDrone()

    while not drone.isConnected():
        print("printing")
        drone.connect()
        sleep(3)


testDrone()