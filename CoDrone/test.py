from codrone import *

flag = 0
battery = 0
IR = 0

def eventUpdataBattery(data):
    global battery
    global flag
    battery = data.batteryPercent
    flag = 1

def eventUpdateIR(data):
    global IR
    global flag
    IR = data.Range

def dataTest(drone):
    global flag

    print("request")
    drone.setEventHandler(DataType.Battery, eventUpdataBattery)
    drone.sendRequest(DataType.Battery)
    print(battery)
    while(1):
        if(flag == 1):
            flag = 0
            print(battery)
            drone.sendRequest(DataType.Battery)

def goTest(drone):
    drone.turn("right", 100, 2)
    drone.turn("left", 100, 2)
    drone.goDirection("right", 100, 2)
    drone.goDirection("left", 100, 2)
    print(drone.getRoll(), drone.getPitch(), drone.getYaw(), drone.getThrottle())

    """
    drone.setThrottle(100)
    print(drone.getThrottle())

    drone.setThrottle(1000)
    print(drone.getThrottle())

    drone.go(3)
    """

drone = CoDrone()

while(not drone.isConnected()):
    print("printing")
    drone.connect()
    sleep(3)

#dataTest(drone)
goTest(drone)
drone.close()