#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf import transformations
import tf2_ros
from copy import deepcopy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from geometry_msgs.msg import PoseStamped, Pose,Point,Quaternion,WrenchStamped,Wrench
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger






class follower:
    def __init__(self):
        rospy.init_node("wrench_subscriber")
        rospy.loginfo("wrench subscriber is running")
        self.config()
        rospy.Subscriber(self.wrench_topic,WrenchStamped,self.callback_wrench)
        # self.broad_cast()
        self.flag = 0
        rospy.spin()


    def config(self):
        self.wrench_topic = rospy.get_param('~robot_wrench_topic', '/mur620a/UR10_r/wrench')
        rospy.loginfo(self.wrench_topic)
  

    def callback_wrench(self,data):
        # test = WrenchStamped()
        # test.wrench.force.

        self.wrench_Fx = data.wrench.force.x
        self.wrench_Fy = data.wrench.force.y
        self.wrench_Fz = data.wrench.force.z
        self.wrench_Tx = data.wrench.torque.x
        self.wrench_Ty = data.wrench.torque.y
        self.wrench_Tz = data.wrench.torque.z
        
        rospy.loginfo(f"Fx: {self.wrench_Fx},Fy: {self.wrench_Fy},Fz: {self.wrench_Fz},Tx: {self.wrench_Tx},Ty: {self.wrench_Ty},Tz: {self.wrench_Tz}")


        # if self.flag == 0:        
        #     rosservice.call_service("/mur620a/UR10_r/ur_hardware_interface/zero_ftsensor","")
        #     rospy.loginfo("service call")
        #     self.flag =1
        
        
        # self.pose.orientation = data.orientation
        # # rospy.loginfo("pose_received")
        # br = tf2_ros.TransformBroadcaster()
        # t = TransformStamped()
        # t.header.stamp = rospy.Time.now()
        # t.header.frame_id = "map"
        # t.child_frame_id = self.follower_pose_topic.split("/")[1]+"/"+"mur_pose"
        # t.transform.translation = self.pose.position
        # t.transform.rotation = self.pose.orientation
        # br.sendTransform(t)
        # rospy.loginfo("running")

    def broad_cast(self):   
        pass
      


        return 


if __name__ == "__main__":
    instance = follower()
