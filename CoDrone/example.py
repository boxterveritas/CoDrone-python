from codrone import *

def ex_getHeight(drone):
    # fly between 500mm and 1000mm for 20sec
    drone.takeoff()
    for i in range(200):
        height = drone.getHeight()
        if height > 1000:
            drone.go(Direction.DOWN)
        elif height < 500:
            drone.go(Direction.UP)
        sleep(0.1)
    drone.land()

def ex_getPressure(drone):
    # print the pressure
    pressure = drone.getPressure()
    print(pressure)

def ex_getDroneTemp(drone):
    ## print the temperature of drone
    temperature = drone.getDroneTemp()
    print(temperature)

def ex_getAngularSpeed(drone):
    # print the angular speed of drone
    Gyrodata = drone.getAngularSpeed()
    print(Gyrodata.ROLL, Gyrodata.PITCH, Gyrodata.YAW)

def ex_getGyroAngles(drone):
    # print the angles of drone
    GyroAngles = drone.getGyroAngles()
    print(GyroAngles.ROLL, GyroAngles.PITCH, GyroAngles.YAW)

def ex_getAccelerometer(drone):
    # print the acceleration of drone
    Acceleration = drone.getAccelerometer()
    print(Acceleration.X, Acceleration.Y, Acceleration.Z)

def ex_getOptFlowPosition(drone):
    # print the optical flow position of drone.
    position = drone.getOptFlowPosition()
    print(position.X, position.Y)

def ex_getState(drone):
    # take off the drone if state is not on flight
    state = drone.getState()
    if state != "Flight":
        drone.takeoff()

def ex_getBatteryPercentage(drone):
    # stop the drone if battery is lower than 10 percent.
    drone.takeoff()
    battery = drone.getBatteryPercentage()
    if battery < 10:
        drone.emergencyStop()

def ex_getBatteryVoltage(drone):
    # print the battery voltage of drone.
    battery = drone.getBatteryVoltage()
    print(battery)

def ex_getTrim(drone):
    trim = drone.getTrim()
    print(trim.ROLL, trim.PITCH, trim.YAW, trim.THROTTLE)

def ex_setRGB(drone):
    for i in range(5):
        drone.setAllRGB(0,i*50,0)
        sleep(1)

def main_test():
    drone = CoDrone(1, 1, 1)
    drone.connect()
    ex_setRGB(drone)
    drone.close()


if __name__ == "__main__":
    main_test()