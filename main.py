from src.vehicle import Vehicle
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
conn = "/dev/ttyACM0"

v = Vehicle(conn)

v.arm_and_takeoff(1)
time.sleep(3)
v.vehicle.mode = VehicleMode("LAND")
time.sleep(5)