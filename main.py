from src.vehicle import Vehicle

conn = "/dev/ttyACM0"

v = Vehicle(conn)

v.arm_and_takeoff(1)