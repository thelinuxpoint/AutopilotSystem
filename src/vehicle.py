# AUTOPILOT SYSTEM
# CREATED BY PRAKASH CHOUDHARY (choudharyprakash0660@gmail.com)

# from pymavlink import mavutil
# import time


# class Vehicle(object):
# 	"""
# 	@Vehicle
# 		class for providing drone instruction
# 	"""
# 	def __init__(self):

# 		self.connection = mavutil.mavlink_connection('/dev/ttyACM0')
# 		self.connection.wait_heartbeat()		
# 		print("Heartbeat from system (system %u component %u)" % (self.connection.target_system, self.connection.target_component))
#     #EOF

# 	def arm(self):
# 		mode = 'GUIDED'
# 		mode_id = self.connection.mode_mapping()[mode]
# 		print("MODE ID - ",mode_id)

# 		print(f"APS|> Setting Mode To \x1b[32m{mode}\x1b[0m ... ",end="",flush=True)
# 		self.connection.mav.set_mode_send(
#     		self.connection.target_system,
#     		1,
#     		mode_id)

		
# 		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
# 		print(str(msg))
# 		# time.sleep(1)

		
# 		self.connection.arducopter_arm()
# 		# wait till motor arms
# 		print("APS|> Drone is Arming ... ")
# 		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
# 		self.connection.motors_armed_wait()
# 		print("APS|> Drone is \x1b[31mARMED!\x1b[0m " + str(msg))
# 		# check the status message
# 	# EOF

# 	def disarm(self):

# 		self.connection.arducopter_disarm()
# 		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
# 		self.connection.motors_disarmed_wait()
# 		print("APS|> Drone is \x1b[32mDISARMED!\x1b[0m " + str(msg))
# 	# EOF


# 	def takeoff(self,alt):
# 		print("APS|> Calibrating Accelerometer ... ",end="")
# 		self.connection.calibrate_level()
# 		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
# 		print(str(msg))

# 		print("APS|> Calibrating Barometer ... ",end="")
# 		self.connection.calibrate_pressure()
# 		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
# 		print(str(msg))

# 		self.connection.mav.command_long_send(self.connection.target_system,self.connection.target_component,
		
# 		mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,alt)
		
# 		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
# 		print("APS|> Drone is Taking OFF ... "+str(msg))

# 	#  EOF

# 	def land(self):

# 		self.connection.mav.command_long_send(self.connection.target_system,self.connection.target_component,
# 		mavutil.mavlink.MAV_CMD_NAV_LAND,0,0,0,0,0,0,0,0)
# 		msg = self.connection.recv_match(type='COMMAND_ACK',blocking=True)
# 		print("APS|> Drone is \x1b[33mLANDING\x1b[0m... "+ str(msg))

# 	def get_altitude(self):
# 		self.connection.location().alt

# 	def close(self):
# 		self.connection.close()


from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


class Vehicle(object):
	
	def __init__(self,conn):
		self.vehicle = connect(conn, wait_ready=True)



	def arm(self):
		self.vehicle.armed = True
		print("APS|> ARMING MOTORS ...",end="")
		time.sleep(1)
		while not self.vehicle.armed:
			print(".",end="")
			time.sleep(1)
		print(" OK")



	def arm_and_takeoff(self,altitude):

		#---------------------- CHANGE MODE --------------------
		mode = "GUIDED"
		print(f"APS|> Setting Mode To \x1b[32m{mode}\x1b[0m ... ",end="\n",flush=True)
		self.vehicle.mode = VehicleMode(mode)
		time.sleep(0.5)
		print("OK")

		#---------------------- ARM MOTORS --------------------
		self.vehicle.armed = True
		print("APS|> ARMING MOTORS ...",end="")
		while not self.vehicle.armed:
			print(".",end="")
			time.sleep(1)
		print(" OK")
		print("APS|> \x1b[32mTAKING OFF\x1b[0m!")
		self.vehicle.simple_takeoff(altitude)  # Take off to target altitude

		#---------------------- CHECK ALTITUDE --------------------
		# while True:
		# 	print("APS|> Altitude: ", self.vehicle.location.global_relative_frame.alt)
		# 	self.turn(180,True,"CW")
		# 	# Break and return from function just below target altitude.
		# 	if self.vehicle.location.global_relative_frame.alt >= altitude * 0.95:
		# 		print("Reached target altitude")
		# 		break

	def turn(self,heading, relative, direction):

		print("Current Heading: %s" % self.vehicle.heading)
		if relative:
			is_relative = 1 # yaw relative to direction of travel
			print("Turning", direction, heading, "degrees relative to current heading.")
			if direction == "CW":
				newHeading = self.vehicle.heading+heading
				direction = 1
			else: 
				newHeading = self.vehicle.heading-heading
				direction = -1
			if newHeading>360:
				newHeading-360
			if newHeading<0:
				newHeading+360
			print("New Heading: %s" % newHeading)
		else:
			is_relative = 0 #yaw is an absolute angle
			print("Turning to %s degrees absolute." % heading)
		# create the CONDITION_YAW command using command_long_encode()
		msg = self.vehicle.message_factory.command_long_encode(
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
			0, #confirmation
			heading,    # param 1, yaw in degrees
			0,          # param 2, yaw speed deg/s
			direction,          # param 3, direction -1 ccw, 1 cw
			is_relative, # param 4, relative offset 1, absolute angle 0
			0, 0, 0)    # param 5 ~ 7 not used
		# send command to vehicle
		self.vehicle.send_mavlink(msg)

		while True:
			print("Current heading: %s" % self.vehicle.attitude)
			if direction == 1 and self.vehicle.heading>=newHeading:
				print("New heading reached")
				break
			elif direction == -1 and self.vehicle.heading<=newHeading:
				print("New heading reached")
				break
			time.sleep(0.5)

