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
        self.filtered_wrench_pub = rospy.Publisher(self.robot_name+"/filtered_WrenchStamped", WrenchStamped, queue_size=1)
        # self.broad_cast()
        self.flag = 0
        rate = rospy.Rate(100)
        
        self.wrench_Fx = None
        self.wrench_Fy = None
        self.wrench_Fz = None
        self.wrench_Tx = None
        self.wrench_Ty = None
        self.wrench_Tz = None

        self.alpha = 0.6 #smoothing factor





        rospy.spin()


    def config(self):
        self.wrench_topic = rospy.get_param('~robot_wrench_topic', '/mur620a/UR10_r/wrench')
        self.robot_name = rospy.get_param('~robot_name', '/mur620a/UR10_r')
        rospy.loginfo(self.wrench_topic)
  

    def callback_wrench(self,data):
        # test = WrenchStamped()
        # test.wrench.force.


        previous_wrench_Fx = self.wrench_Fx
        previous_wrench_Fy = self.wrench_Fy
        previous_wrench_Fz = self.wrench_Fz
        previous_wrench_Tx = self.wrench_Tx
        previous_wrench_Ty = self.wrench_Ty
        previous_wrench_Tz = self.wrench_Tz

        self.wrench_Fx = data.wrench.force.x
        self.wrench_Fy = data.wrench.force.y
        self.wrench_Fz = data.wrench.force.z
        self.wrench_Tx = data.wrench.torque.x
        self.wrench_Ty = data.wrench.torque.y
        self.wrench_Tz = data.wrench.torque.z
        
        # rospy.loginfo(f"{self.wrench_Fx==previous_wrench_Fx}")
        # rospy.loginfo(f"Fx: {self.wrench_Fx},Fy: {self.wrench_Fy},Fz: {self.wrench_Fz},Tx: {self.wrench_Tx},Ty: {self.wrench_Ty},Tz: {self.wrench_Tz}")
        
        #filtering
        if previous_wrench_Fx is not None:
            filtered_wrench = WrenchStamped()
            filtered_wrench_Fx = self.alpha*self.wrench_Fx + (1-self.alpha)*previous_wrench_Fx
            filtered_wrench_Fy = self.alpha*self.wrench_Fy + (1-self.alpha)*previous_wrench_Fy
            filtered_wrench_Fz = self.alpha*self.wrench_Fz + (1-self.alpha)*previous_wrench_Fz
            filtered_wrench_Tx = self.alpha*self.wrench_Tx + (1-self.alpha)*previous_wrench_Tx
            filtered_wrench_Ty = self.alpha*self.wrench_Ty + (1-self.alpha)*previous_wrench_Ty
            filtered_wrench_Tz = self.alpha*self.wrench_Tz + (1-self.alpha)*previous_wrench_Tz

            filtered_wrench.header.stamp = rospy.Time.now()
            filtered_wrench.header.frame_id = data.header.frame_id
            filtered_wrench.wrench.force.x = filtered_wrench_Fx
            filtered_wrench.wrench.force.y = filtered_wrench_Fy
            filtered_wrench.wrench.force.z = filtered_wrench_Fz
            filtered_wrench.wrench.torque.x = filtered_wrench_Tx
            filtered_wrench.wrench.torque.y = filtered_wrench_Ty
            filtered_wrench.wrench.torque.z = filtered_wrench_Tz

            self.filtered_wrench_pub.publish(filtered_wrench)





        # rospy.loginfo(f"Fx: {filtered_wrench_Fx},Fy: {filtered_wrench_Fy},Fz: {filtered_wrench_Fz},Tx: {filtered_wrench_Tx},Ty: {filtered_wrench_Ty},Tz: {filtered_wrench_Tz}")
 


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
