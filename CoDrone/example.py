from codrone import CoDrone
from protocol import *

from time import sleep

def test_setRoll(drone, i):
    # Drone goes right for 1 second with 50 power
    drone.setRoll(i)
    drone.move(1)
    print(drone.getRoll())
    # Drone goes left indefinitely with 50 power
    drone.setRoll(-i)
    drone.move()
    print(drone.getRoll())

def test_setPitch(drone, i):
    # Drone goes forward for 1 second with 50 power
    drone.setPitch(i)
    drone.move(1)
    print(drone.getPitch())
    # Drone goes backward indefinitely with 50 power
    drone.setPitch(-i)
    drone.move()
    print(drone.getPitch())

def test_setYaw(drone, i):
    # Drone turns clockwise for 1 second with 50 power
    drone.setYaw(i)
    drone.move(1)
    print(drone.getYaw())
    # Drone turns counterclockwise indefinitely with 50 power
    drone.setYaw(-i)
    drone.move()
    print(drone.getYaw())

def test_setThrottle(drone, i):
    # Drone goes upward for 1 second with 50 power
    drone.setThrottle(i)
    drone.move(1)
    print(drone.getThrottle())
    # Drone goes downward indefinitely with 50 power
    drone.setThrottle(-i)
    drone.move()
    print(drone.getThrottle())


def main_test():
    drone = CoDrone(1,1,1)

    while not drone.isConnected():
        drone.connect()
        sleep(3)

    drone.connect("COM4", "1928")

    drone.takeoff()
    print(drone._data.takeoffFuncFlag)



    drone.sendLinkDisconnect()
    drone.close()

if __name__ == "__main__":
    main_test()