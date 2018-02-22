from codrone import *
import math

drone = CoDrone()
while not drone.isConnected():
    drone.connect("COM15", "PETRONE 2455")
    sleep(3)
    print("Connected Bad")


drone.takeoff()
drone.go(Direction.Forward, 1)
while(drone.getHeight()< 500):
    drone.sendControl(0,0,0,20)
    sleep(0.5)
while(drone.getHeight()> 500):
    drone.sendControl(0,0,0,-20)
    sleep(0.5)
drone.go(Direction.Forward, 1)
drone.hover(5)
while(drone.getHeight()< 1000):
    drone.sendControl(0,0,0,20)
    sleep(0.5)
while(drone.getHeight()> 1000):
    drone.sendControl(0,0,0,-20)
    sleep(0.5)
drone.hover(3)
drone.land()
