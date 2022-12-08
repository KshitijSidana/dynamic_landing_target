#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL

import random

class UGV:
	def __init__(self) -> None:
		
		rospy.init_node("ground_vehicle_node_py")

		self.current_state = State()
		self.state_sub = rospy.Subscriber("ugv/mavros/state", State, callback = self.state_cb)

		self.local_pos_pub = rospy.Publisher("ugv/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		
		rospy.wait_for_service("/ugv/mavros/cmd/arming")
		self.arming_client = rospy.ServiceProxy("ugv/mavros/cmd/arming", CommandBool)    
		self.arm_cmd = CommandBoolRequest()

		# Mode service
		rospy.wait_for_service("/ugv/mavros/set_mode")
		self.set_mode_client = rospy.ServiceProxy("ugv/mavros/set_mode", SetMode)
		self.offb_set_mode = SetModeRequest()
		self.offb_set_mode.custom_mode = 'OFFBOARD'

		# Landing service
		self.landing_client = rospy.ServiceProxy('/ugv/mavros/cmd/land', CommandTOL)
		
		# Takeoff service 
		self.takeoff_client = rospy.ServiceProxy('/ugv/mavros/cmd/takeoff', CommandTOL)

		# Setpoint publishing MUST be faster than 2Hz
		self.rate = rospy.Rate(20)

		# Wait for Flight Controller connection
		while(not rospy.is_shutdown() and not self.current_state.connected):
			print("Wait for Flight Controller connection")
			for _ in range(10) :
				self.rate.sleep()

		# message type [?afak?]
		self.pose = PoseStamped()

		# 
		self.last_req = rospy.Time.now()

		# 
		self.check()
		



	def check(self):
		if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
			if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
				rospy.loginfo("OFFBOARD enabled")
			
			self.last_req = rospy.Time.now()
		else:
			if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
				if(self.arming_client.call(self.arm_cmd).success == True):
					rospy.loginfo("Vehicle armed")
			
				self.last_req = rospy.Time.now()

	def state_cb(self, msg):
		# print("state callback fn")
		self.current_state = msg
		return

	def arm(self, state_arm = True):
		if state_arm:
			print ("Arm")
			self.arm_cmd.value = True
			#  loop untill armed
			while(not self.arming_client.call(self.arm_cmd).success and not rospy.is_shutdown()):
				for _ in range(10) :
					self.rate.sleep()

			if(self.arming_client.call(self.arm_cmd).success == True):
				rospy.loginfo("Vehicle armed")
		else:
			print ("Disarm")
			self.arm_cmd.value = False
			#  loop untill disarmed
			while(not self.arming_client.call(self.arm_cmd).success and not rospy.is_shutdown()):
				for _ in range(10) :
					self.rate.sleep()

			if(self.arming_client.call(self.arm_cmd).success == True):
				rospy.loginfo("Vehicle disarmed")
		return

	def take_off(self, altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0):
		print ("Taking off")
		rospy.wait_for_service('/ugv/mavros/cmd/takeoff')
		try:
			response = self.takeoff_client(altitude, latitude, longitude, min_pitch, yaw)
			rospy.loginfo(response)
		except rospy.ServiceException as e:
			print(f"Takeoff failed: {e}")

		# TODO Wait untill 98% of take_off altitude is reached
 		# while(check_altitude<=0.98*altitude and not rospy.is_shutdown()):
		# 	self.rate.sleep()
		
		return

	def land(self, altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0):
		print ("Landing")
		rospy.wait_for_service('/ugv/mavros/cmd/land')
		try:
			response = self.landing_client(altitude, latitude, longitude, min_pitch, yaw)
			rospy.loginfo(response)
		except rospy.ServiceException as e:
			print(f"Takeoff failed: {e}")

		return

	def go_to(self, x: float, y: float, z: float):
		print (f"go_to {x}, {y}, {z}")

		self.pose.pose.position.x = x
		self.pose.pose.position.y = y
		self.pose.pose.position.z = z

		# # Send a few setpoints before starting
		# for i in range(100):   
		# 	if(rospy.is_shutdown()):
		# 		break

		# 	self.local_pos_pub.publish(self.pose)
		# 	self.rate.sleep()

		self.check()
		self.local_pos_pub.publish(self.pose)

		for _ in range(20):
			self.rate.sleep()

		return
		

def call_a_fn_to_get_xyz(count: float):
	if (count>= 20):
		return "L", 10*random.random(), 10*random.random(),3
	else:
		return "F", 10*random.random(), 10*random.random(),3

if __name__ == "__main__":
	
	ugv = UGV()
	ugv.last_req = rospy.Time.now()
	
	count = 0
	status = 'F' # F: Fly || L: Land
	
	if not rospy.is_shutdown():
		ugv.check()
		ugv.arm(True)
		ugv.take_off()
	else:
		print("master not running | Exiting")
		exit()

	# loop runs untill ros is shut down or asked to land
	while(not rospy.is_shutdown()):
		count+=1

		ugv.check()

		# TODO : replace with call to coordinator fn that returns -> status, x,y,z 
		status, x,y,z = call_a_fn_to_get_xyz(count)
		
		if status == 'L':
			break

		ugv.go_to(10,10,0)	


	if not rospy.is_shutdown():
		ugv.land()
		ugv.arm(False)
	else:
		print("master not running | Exiting")
		exit()