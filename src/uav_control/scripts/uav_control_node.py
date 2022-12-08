#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL

import random

class UAV:
	def __init__(self) -> None:
		
		self.current_state = State()

		rospy.init_node("uav_control_node_py")

		self.state_sub = rospy.Subscriber("mavros/state", State, callback = self.state_cb)

		self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
		
		rospy.wait_for_service("/mavros/cmd/arming")
		self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    
		self.arm_cmd = CommandBoolRequest()

		rospy.wait_for_service("/mavros/set_mode")
		self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
		# Landing service
		self.landing_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
		# Takeoff service 
		self.takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

		# Mode service
		self.offb_set_mode = SetModeRequest()
		self.offb_set_mode.custom_mode = 'OFFBOARD'

		# Setpoint publishing MUST be faster than 2Hz
		self.rate = rospy.rate(20)

		# Wait for Flight Controller connection
		while(not rospy.is_shutdown() and not self.current_state.connected):
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
		self.current_state
		self.current_state = msg

	def arm(self, state_arm = True):
		if state_arm:
			self.arm_cmd.value = True
			if(self.arming_client.call(self.arm_cmd).success == True):
				rospy.loginfo("Vehicle armed")
		return

	def take_off(self, altitude=10, latitude=0, longitude=0, min_pitch=0, yaw=0):
		print ("Taking off")
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:
			response = self.takeoff_cl(altitude, latitude, longitude, min_pitch, yaw)
			rospy.loginfo(response)
		except rospy.ServiceException as e:
			print(f"Takeoff failed: {e}")

		return

	def land(self, altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0):
		print ("Landing")
		rospy.wait_for_service('/mavros/cmd/land')
		try:
			response = self.landing_cl(altitude, latitude, longitude, min_pitch, yaw)
			rospy.loginfo(response)
		except rospy.ServiceException as e:
			print(f"Takeoff failed: {e}")

		return

	def go_to(self, x: float, y: float, z: float):

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

		return
		

def call_a_fn_to_get_xyz(count: float):
	if (count>= 20):
		return "L", 10*random.random(), 10*random.random(),3
	else:
		return "F", 10*random.random(), 10*random.random(),3

if __name__ == "__main__":
	
	uav = UAV()
	uav.last_req = rospy.Time.now()
	
	count = 0
	status = 'F' # F: Fly || L: Land
	
	if not rospy.is_shutdown():
		uav.check()
		uav.arm()
		uav.take_off()
	else:
		print("master not running | Exiting")
		exit()

	# loop runs untill ros is shut down or asked to land
	while(not rospy.is_shutdown()):
		count+=1

		uav.check()
		state, x,y,z = call_a_fn_to_get_xyz(count)
		
		if status == 'L':
			break

		uav.go_to(x,y,z)	


	if not rospy.is_shutdown():
		uav.land()
	else:
		print("master not running | Exiting")
		exit()