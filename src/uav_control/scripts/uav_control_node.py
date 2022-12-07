#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL

current_state = State()



def state_cb(msg):
    global current_state
    current_state = msg




if __name__ == "__main__":
    rospy.init_node("uav_control_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    # Landing service
    landing_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    pts = list()
    pts.append([-2,-2])
    pts.append([ 2,-2])
    pts.append([ 2, 2])
    pts.append([-2, 2])
    pts.append([3,3,0.3])
    i = 0

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        pose.pose.position.x = pts[i%4][0]
        pose.pose.position.y = pts[i%4][1]
        pose.pose.position.z = 2
        i+=1
        
        if i%15 == 0:
            rospy.wait_for_service('/mavros/cmd/land')
            response = landing_cl(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
            rospy.loginfo(response)
            # local_pos_pub.publish(pose)
            for _ in range (105):
                rate.sleep()

        local_pos_pub.publish(pose)
        for _ in range (15):
            rate.sleep()
