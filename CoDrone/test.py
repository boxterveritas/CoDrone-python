from codrone import *

flag = 0
battery = 0
IR = 0

def testBasic(drone):
    drone.takeoff()
    drone.sendControlWhile(0,0,0,10,3)
    drone.land()
    print("finish")
    sleep(3)

def testColor(drone):
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

def testData(drone):
    print("---- START Data Test")
    drone.takeoff()
    for i in range(10):
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

def testFlight(drone):
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

def testMove(drone):
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

def testGo(drone):
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

def testOn(drone):
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
getHeight() test: 
take off from ground, 
read the height from the ground. 
Fly above a table, 
read the height from the table.
"""
def testGetHeight(drone):
    drone.takeoff()
    drone.go(Direction.Up, 1, 30)
    sleep(1)
    print("ground height :" , drone.getHeight())
    drone.go(Direction.Forward, 1)
    sleep(2)
    print("table height :" , drone.getHeight())
    print("table height :", drone.getHeight())
    drone.land()
    height = drone.getHeight()
    while(height < 10):
        height = drone.getHeight()
    sleep(1)
"""
Spiral test:
Do an outward spiral, then land
"""
def testSpiral(drone):
    drone.takeoff()
    for i in range(5):
        drone.sendControl(10 + i*2, 0, -50, -i*2)
        sleep(1)
    drone.hover(1)
    drone.land()

"""
batteryLow() test
Flash the eyes red when the drone is below 50% battery
"""
def testBatteryLow(drone):
    def RedFlashEyes():
        drone.setEyeMode(Mode.Blinking)
        drone.setEyeRGB(255,0,0)
        print("redEye, low Battery")
        sleep(1)

    drone.onLowBattery(RedFlashEyes)
    print(drone.getBatteryPercentage())

    drone.takeoff()
    while(drone.getBatteryPercentage() > 50):
        drone.go(Direction.Left,1,70)
        drone.go(Direction.Right,1, 30)
        print(drone.getBatteryPercentage())

    sleep(1)
    print(drone.getBatteryPercentage())
    sleep(1)
    drone.land()

"""
    Fly in a circle: 
    Just fly in a circle that will fit in our room. 
    Okay to use yaw + roll. 
    Just needs to see it ends where it started.
"""
def testCircle(drone):
    drone.takeoff()
    power = -30
    yaw = drone.getAngularSpeed().Yaw
    degree = -360 + yaw

    start_time = time.time()
    drone.sendControl(5, 0, 0, 0)
    while (start_time - time.time()) < 15:
        if abs(yaw - drone._data.attitude.Yaw) > 180:
            degree += 360
        yaw = drone._data.attitude.Yaw
        if degree < yaw:
            drone.sendControl(10+int(abs(yaw - degree)/100), 0, power, 0)
            sleep(0.1)
        else:
            break
    drone.hover(1)
    sleep(0.5)
    drone.land()

"""
    L shape: take off, 
    fly forward for 3 seconds, 
    turn, 
    fly right for 3 seconds
"""
def testL(drone):
    drone.takeoff()

    drone.go(Direction.Forward, 2,30)
    drone.turnDegree(Direction.Right, Degree.ANGLE_90)
    drone.go(Direction.Forward, 2,30)

    drone.land()
    sleep(3)

"""
    Fly in a square: 
    Fly in a square large enough for it to fit in our room
"""
def testSquare(drone):
    drone.takeoff()

    drone.go(Direction.Right, 2,30)
    drone.go(Direction.Forward, 2,30)
    drone.go(Direction.Left, 2,30)
    drone.go(Direction.Backward, 2,30)

    drone.land()
    sleep(3)

"""
    3-part zig zag: take off,
    45 degrees right for 2 seconds,
    90 degrees left 2 seconds,
    90 degrees right for 2 seconds
"""
def testZigZag(drone):
    drone.takeoff()

    sleep(1)
    drone.move(1, 30,30,0,0)
    drone.move(1, -30,30,0,0)
    drone.move(1, 30,30,0,0)
    drone.move(1, -30,30,0,0)

    drone.land()
    sleep(3)

"""
    goToHeight() test: 
    Fly forward for 2 seconds, 
    go to 500 mm, 
    then fly forward for 2 seconds, 
    go to 1000 mm
"""
def testHeight(drone):
    drone.takeoff()

    print(500)
    drone.go(Direction.Forward, 1, 50)
    drone.goToHeight(500)
    sleep(1)
    print(1000)
    drone.go(Direction.Forward, 1, 50)
    drone.goToHeight(1000)
    sleep(5)
    drone.land()
    sleep(3)

def testTurn(drone):
    drone.takeoff()
    drone.turnDegree(Direction.Right, Degree.ANGLE_30)
    drone.hover(3)
    drone.go(Direction.Forward, 1)
    drone.turnDegree(Direction.Left, Degree.ANGLE_120)
    drone.go(Direction.Forward, 1)
    drone.turnDegree(Direction.Left, Degree.ANGLE_120)
    drone.go(Direction.Forward, 1)

    sleep(1)
    drone.land()
    sleep(3)

def testDrone():
    drone = CoDrone()
    while not drone.isConnected():
        drone.connect("COM4", "PETRONE 2455")
        sleep(3)
        print("?")

    drone.takeoff()

    drone.sendLinkDisconnect()
    sleep(3)
    drone.close()

testDrone()
