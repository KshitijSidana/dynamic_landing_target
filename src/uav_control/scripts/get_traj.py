import math
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates


class Coordinator:
    
    def __init__(self) -> None:
        # rospy.init_node('coordinator', anonymous = True)
        # rospy.Subscriber('/uav/mavros/local_position/pose', PoseStamped, self.uav_pose)
        # rospy.Subscriber('/ugv/mavros/local_position/pose', PoseStamped, self.ugv_pose)
        self.model_states = rospy.Subscriber("/gazebo/model_states", ModelStates, callback = self.update_pose)

        self.x_uav = 0.0
        self.y_uav = 0.0
        self.z_uav = 0.0
        self.x_ugv = 0.0
        self.y_ugv = 0.0
        self.z_ugv = 0.0

    # def result(self,min_cap = -5.5, max_cap = 5.5):
    #     M = 2
    #     del_x = M*(self.x_ugv-self.x_uav)
    #     del_x = max(min_cap, del_x)
    #     del_x = min(max_cap, del_x)
        
    #     del_y = M*(self.y_ugv-self.y_uav)
    #     del_y = max(min_cap, del_y)
    #     del_y = min(max_cap, del_y)

    #     dist = math.sqrt( (self.x_ugv-self.x_uav)**2 + (self.y_ugv-self.y_uav)**2 )
        
    #     result_z = max(self.z_ugv, 5 - 1/(10*dist))
    #     res_z = max(0.5, result_z)
    #     prsc = 4
    #     print(f"dist = {round(dist,ndigits=prsc)}, (1/dist)/10 = {round(1/(10*dist), ndigits=prsc)} dx: {round(del_x,ndigits=prsc)}, dy: {round(del_y,ndigits=prsc)}, z: {round(res_z,ndigits=prsc)}")
    #     if (dist>0.01):
    #         return "F", self.x_uav+del_x, self.y_uav+del_y, res_z
    #     return     "L", self.x_uav+del_x, self.y_uav+del_y, res_z

    def result(self,min_cap = -5.5, max_cap = 5.5):
        

        dist = (self.x_ugv-self.x_uav)**2 + (self.y_ugv-self.y_uav)**2 
        
        result_z = max( 0, 5 - 2/(dist) )
        # res_z = max(0.5, result_z)
        prsc = 4
        # print(f"dist = {round(dist,ndigits=prsc)}, (1/dist)/10 = {round(1/(10*dist), ndigits=prsc)} dx: {round(del_x,ndigits=prsc)}, dy: {round(del_y,ndigits=prsc)}, z: {round(res_z,ndigits=prsc)}")
        if (dist>0.015):
            return "F", self.x_ugv, self.y_ugv, result_z
        return     "L", self.x_ugv, self.y_ugv, result_z

    def uav_pose(self, msg):
        self.x_uav = msg.pose.position.x
        self.y_uav = msg.pose.position.y
        self.z_uav = msg.pose.position.z

    def ugv_pose(self, msg):
        self.x_ugv = msg.pose.position.x
        self.y_ugv = msg.pose.position.y
        self.z_ugv = msg.pose.position.z

    def update_pose(self, msg):
        for idx, name in enumerate(msg.name):
            if name == 'iris': 
                self.x_uav = msg.pose[idx].position.x
                self.y_uav = msg.pose[idx].position.y
                self.z_uav = msg.pose[idx].position.z
            if name == 'rover1' or name == 'r1_rover':
                self.x_ugv = msg.pose[idx].position.x
                self.y_ugv = msg.pose[idx].position.y
                self.z_ugv = msg.pose[idx].position.z