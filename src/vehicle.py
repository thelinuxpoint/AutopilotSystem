# AUTOPILOT SYSTEM
# CREATED BY PRAKASH CHOUDHARY (choudharyprakash0660@gmail.com)

from pymavlink import mavutil
import time


class Vehicle(object):
	"""
	@Vehicle
		class for providing drone instruction
	"""
	def __init__(self):
		self.connection = mavutil.mavlink_connection('/dev/ttyACM0')
		self.connection.wait_heartbeat()		
		print("Heartbeat from system (system %u component %u)" % (self.connection.target_system, self.connection.target_component))
    #EOF

	def arm(self):
		mode = 'GUIDED'
		mode_id = self.connection.mode_mapping()[mode]
		print("MODE ID - ",mode_id)

		print(f"APS|> Setting Mode To \x1b[32m{mode}\x1b[0m ")
		self.connection.mav.set_mode_send(
    		self.connection.target_system,
    		1,
    		mode_id)
		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)

		self.connection.arducopter_arm()
		# wait till motor arms
		print("APS|> Drone is Arming ... ")
		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
		self.connection.motors_armed_wait()
		print("APS|> Drone is \x1b[31mARMED!\x1b[0m " + str(msg))
		# check the status message
	# EOF

	def disarm(self):

		self.connection.arducopter_disarm()
		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
		self.connection.motors_disarmed_wait()
		print("APS|> Drone is \x1b[32mDISARMED!\x1b[0m " + str(msg))
	# EOF


	def takeoff(self,alt):
		print("APS|> Calibrating Accelerometer ... ",end="")
		self.connection.calibrate_level()
		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
		print(str(msg))

		print("APS|> Calibrating Barometer ... ",end="")
		self.connection.calibrate_pressure()
		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
		print(str(msg))

		self.connection.mav.command_long_send(self.connection.target_system,self.connection.target_component,
		
		mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,alt)
		
		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
		print("APS|> Drone is Taking OFF ... "+str(msg))

	#  EOF

	def land(self):

		print("APS|> Drone is \x1b[33mLANDING\x1b[0m... "+ str(msg))
		self.connection.mav.command_long_send(self.connection.target_system,self.connection.target_component,
		mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,0,0)
		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)

	def get_altitude(self):
		self.connection.location().alt

v = Vehicle()
v.arm()
v.takeoff(10)
v.disarm()
