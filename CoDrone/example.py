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
    drone.move(1, 0,0,0,10)
    drone.move(1, 0,0,0,-10)

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

    drone.turn(Direction.LEFT,3)
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
    print("100 -- {}".format(time.time()-start_time))

    start_time = time.time()
    drone.goToHeight(300)
    print("300 -- {}".format(time.time()-start_time))

    start_time = time.time()
    drone.goToHeight(300)
    print("500 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(300)
    print("1000 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(300)
    print("1500 -- {}".format(time.time() - start_time))

    start_time = time.time()
    drone.goToHeight(300)
    print("2000 -- {}".format(time.time() - start_time))

### SENSORS

def main_test():
    drone = CoDrone(1,1,1)

    drone.connect()
    drone.takeoff()
    test_rotate180(drone)
    #test_STARTSTOP(drone)
    drone.land()
    drone.sendLinkDisconnect()
    drone.close()

if __name__ == "__main__":
    main_test()