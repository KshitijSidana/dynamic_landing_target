#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL


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


        # Setpoint publishing MUST be faster than 2Hz
        self.rate = rospy.rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not self.current_state.connected):
            self.rate.sleep()

        self.pose = PoseStamped()
        
    def state_cb(self, msg):
        self.current_state
        self.current_state = msg

    def arm(self, state: bool):
        self.arm_cmd.value = True
        if(self.arming_client.call(self.arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")
        return

    def take_off(self):

        pass

    def go_to(self):

        self.pose.pose.position.x = 0
        self.pose.pose.position.y = 0
        self.pose.pose.position.z = 2

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            self.local_pos_pub.publish(self.pose)
            self.rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        

        last_req = rospy.Time.now()

        pts = list()
        pts.append([-2,-2])
        pts.append([ 2,-2])
        pts.append([ 2, 2])
        pts.append([-2, 2])
        pts.append([3,3,0.3])
        i = 0

        while(not rospy.is_shutdown()):
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")
                
                last_req = rospy.Time.now()
            else:
                if(not self.current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(self.arming_client.call(self.arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")
                
                    last_req = rospy.Time.now()

            self.pose.pose.position.x = pts[i%4][0]
            self.pose.pose.position.y = pts[i%4][1]
            self.pose.pose.position.z = 2
            i+=1
            
            if i%15 == 0:
                rospy.wait_for_service('/mavros/cmd/land')
                response = self.landing_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
                rospy.loginfo(response)
                # self.local_pos_pub.publish(self.pose)
                for _ in range (105):
                    self.rate.sleep()

            self.local_pos_pub.publish(self.pose)
            for _ in range (15):
                self.rate.sleep()


if __name__ == "__main__":
    pass