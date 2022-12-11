#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL
from gazebo_msgs.msg import ModelStates
from get_traj import Coordinator
import random

class UAV:
	def __init__(self) -> None:
		
		rospy.init_node("uav_control_node_py")
		self.model_name = 'iris'

		self.current_state = State()
		self.state_sub = rospy.Subscriber("uav/mavros/state", State, callback = self.state_cb)

		self.local_pos_pub = rospy.Publisher("uav/mavros/setpoint_position/local", PoseStamped, queue_size=10)

		# Get current position
		# self.local_pos_sub = rospy.Subscriber("uav/mavros/local_position/pose", PoseStamped, callback = self.set_uav_pos)
		self.model_states = rospy.Subscriber("/gazebo/model_states", ModelStates, callback = self.set_uav_pos)
		self.x_uav = 0.0
		self.y_uav = 0.0
		self.z_uav = 0.0
		
		rospy.wait_for_service("/uav/mavros/cmd/arming")
		self.arming_client = rospy.ServiceProxy("uav/mavros/cmd/arming", CommandBool)    
		self.arm_cmd = CommandBoolRequest()

		# Mode service
		rospy.wait_for_service("/uav/mavros/set_mode")
		self.set_mode_client = rospy.ServiceProxy("uav/mavros/set_mode", SetMode)
		self.offb_set_mode = SetModeRequest()
		self.offb_set_mode.custom_mode = 'OFFBOARD'

		# Landing service
		self.landing_client = rospy.ServiceProxy('/uav/mavros/cmd/land', CommandTOL)
		
		# Takeoff service 
		self.takeoff_client = rospy.ServiceProxy('/uav/mavros/cmd/takeoff', CommandTOL)

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
		# while(self.current_state.mode != "OFFBOARD"):
		# 	self.rate.sleep()
		# 	if((rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
		# 		if(self.set_mode_client.call(self.offb_set_mode).mode_sent == True):
		# 			rospy.loginfo("OFFBOARD enabled")
		# 			break		
		# 		self.last_req = rospy.Time.now()
		# 	else:
		# 		if(not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
		# 			if(self.arming_client.call(self.arm_cmd).success == True):
		# 				rospy.loginfo("Vehicle armed")
		# 			self.last_req = rospy.Time.now()
		
		# if(self.current_state.mode == "OFFBOARD" and not self.current_state.armed and (rospy.Time.now() - self.last_req) > rospy.Duration(5.0)):
		# 	if(self.arming_client.call(self.arm_cmd).success == True):
		# 		rospy.loginfo("Vehicle armed")
		# 	self.last_req = rospy.Time.now()

	def state_cb(self, msg):
		# print("state callback fn")
		self.current_state = msg
		return

	def set_uav_pos(self, msg):
		for idx, name in enumerate(msg.name):
			if name == self.model_name:
				break
		self.x_uav = msg.pose[idx].position.x
		self.y_uav = msg.pose[idx].position.y
		self.z_uav = msg.pose[idx].position.z

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
		rospy.wait_for_service('/uav/mavros/cmd/takeoff')
		try:
			response = self.takeoff_client(altitude, latitude, longitude, min_pitch, yaw)
			rospy.loginfo(response)
			# Wait untill take_off altitude is reached
			while(self.z_uav<=4 and not rospy.is_shutdown()):
				self.go_to(0,0,5)
		
		except rospy.ServiceException as e:
			print(f"Takeoff failed: {e}")

		
		return

	def land(self, altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0):
		print ("Landing")
		rospy.wait_for_service('/uav/mavros/cmd/land')
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

		# for _ in range(20):
		# 	self.rate.sleep()

		return
		

def call_a_fn_to_get_xyz(count: float):
	if (count>= 20):
		return "L", 10*random.random(), 10*random.random(),3
	else:
		return "F", 10*random.random(), 10*random.random(),3

if __name__ == "__main__":
	
	uav = UAV()
	coordinator = Coordinator()
	uav.last_req = rospy.Time.now()
	
	count = 0
	status = 'F' # F: Fly || L: Land
	
	if not rospy.is_shutdown():
		uav.check()
		uav.arm(True)
		uav.take_off()
	else:
		print("master not running | Exiting")
		exit()

	# loop runs untill ros is shut down or asked to land
	while(not rospy.is_shutdown()):
		count+=1
		uav.check()
		
		status, x,y,z = coordinator.result()
		
		if status == 'L':
			break

		uav.go_to(x,y,z)	

	if not rospy.is_shutdown():
		uav.land()
		uav.arm(False)
	else:
		print("master not running | Exiting")
		exit()