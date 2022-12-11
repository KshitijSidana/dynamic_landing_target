#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL
from gazebo_msgs.msg import ModelStates

import random
from gen_way_points import PointsInCircum, infinityCircum

# Helper fn 
def is_between(a, x, b)-> bool:
	return min(a, b) < x < max(a, b)



class UGV:
	def __init__(self) -> None:
		
		rospy.init_node("ground_vehicle_node_py")
		self.model_name = 'rover1'
		# self.model_name = 'r1_rover'
		self.current_state = State()
		self.state_sub = rospy.Subscriber("ugv/mavros/state", State, callback = self.state_cb)

		self.local_pos_pub = rospy.Publisher("ugv/mavros/setpoint_position/local", PoseStamped, queue_size=10)
		self.pose = PoseStamped()
		self.pose.pose.position.x = 0
		self.pose.pose.position.y = 0
		self.pose.pose.position.z = 0

		# self.local_vel_pub = rospy.Publisher("ugv/mavros/setpoint_velocity/local", PoseStamped, queue_size=10)
		# self.local_vel_pub = rospy.Publisher("ugv/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=10)
		# self.vel = Twist()
		# self.vel.linear.x = 0
		# self.vel.linear.y = 0
		# self.vel.linear.z = 0


		self.vel = PositionTarget()
		self.local_vel_pub = rospy.Publisher("/ugv/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
		# self.local_pos_sub = rospy.Subscriber("ugv/mavros/local_position/pose", PoseStamped, callback = self.set_uav_pos)
		self.model_states = rospy.Subscriber("/gazebo/model_states", ModelStates, callback = self.set_ugv_pos)
		self.x_ugv = 0.0
		self.y_ugv = 0.0
		self.z_ugv = 0.0

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
	
	def set_ugv_pos(self, msg):
		for idx, name in enumerate(msg.name):
			if name == self.model_name:
				break
		self.x_ugv = msg.pose[idx].position.x
		self.y_ugv = msg.pose[idx].position.y
		self.z_ugv = msg.pose[idx].position.z

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

			# Send a few setpoints before starting
			for _ in range(10):   
				if(rospy.is_shutdown()):
					break
				self.local_pos_pub.publish(self.pose)
				self.rate.sleep()
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

	def take_off(self, altitude=0.01, latitude=0, longitude=0, min_pitch=0, yaw=0):
		print ("Taking off")
		rospy.wait_for_service('/ugv/mavros/cmd/takeoff')
		try:
			response = self.takeoff_client(altitude, latitude, longitude, min_pitch, yaw)
			rospy.loginfo(response)
			for _ in range(10):
				self.rate.sleep()
		except rospy.ServiceException as e:
			print(f"Takeoff failed: {e}")

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

	def set_vel (self, Vx=0.10, Vy=0.10, Vz = 0.00):
		
		
		pass

	
	def go_to(self, x: float, y: float, z: float):
		print (f"go_to {x}, {y}, {z}")

		self.pose.pose.position.x = x
		self.pose.pose.position.y = y
		self.pose.pose.position.z = z

		self.check()
		self.local_pos_pub.publish(self.pose)

		# try untill pos is reached or untill max_tries reached
		tries = 0
		while (not is_between(0.9*x, self.x_ugv, 1.1*x) and not is_between(0.9*y, self.y_ugv, 1.1*y) and tries <=20):
			print ("waiting for vehicle to reach setpoint")		
			tries+=1
			for _ in range(5):
				self.rate.sleep()
			self.local_pos_pub.publish(self.pose)

		return
		
	def set_vel(self, x:float, y:float):
		print (f"get_vel {x}, {y}")
		self.vel.coordinate_frame = 8
		self.vel.type_mask = 3527

		
		self.vel.velocity.x = x
		self.vel.velocity.y = y
		self.vel.velocity.z = 0
		
		# self.vel.angular.x = 0
		# self.vel.angular.y = 0
		# self.vel.angular.z = 0

		self.check()
		self.local_vel_pub.publish(self.vel)
		for _ in range(5):
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
	
	if not rospy.is_shutdown():
		ugv.check()
		ugv.arm(True)
		ugv.take_off()
	else:
		print("master not running | Exiting")
		exit()
	
	count = 0
	status = 'F' # F: Fly/Drive || L: Land/Park

	cap = 100
	vel = 0.80
	# loop runs untill ros is shut down or asked to land
	while(not rospy.is_shutdown()):
		count+=1
		ugv.check()

		# for x,y in PointsInCircum(r=20, n=250):
		# 	ugv.check()
		# 	ugv.go_to(x,y,0)
		# 	ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# 	# ugv.rate.sleep()
		# # break
		# status, x,y,z = call_a_fn_to_get_xyz(count)
		while count<cap and not rospy.is_shutdown():
			ugv.set_vel (vel, 0.0)
			count+=1
		count = 0
		while count<cap and not rospy.is_shutdown():
			ugv.set_vel (0.0, vel)
			count+=1
		count = 0
		while count<cap and not rospy.is_shutdown():
			ugv.set_vel (-1*vel, 0.0)
			count+=1
		count = 0
		while count<cap and not rospy.is_shutdown():
			ugv.set_vel (0.0, -1*vel)
			count+=1
		count = 0
		
		if status == 'L':
			break


	if not rospy.is_shutdown():
		ugv.land()
		ugv.arm(False)
	else:
		print("master not running | Exiting")
		exit()