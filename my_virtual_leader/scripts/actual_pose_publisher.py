#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from tf import transformations
import tf2_ros
from copy import deepcopy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from geometry_msgs.msg import PoseStamped, Pose,Point,Quaternion
from nav_msgs.msg import Odometry





class follower:
    def __init__(self):
        self.config()
        rospy.init_node("actual_pose_publishe")
        rospy.loginfo("actual_pose_publiser is running")
        rospy.Subscriber(self.follower_pose_topic,Pose,self.callback_follower_pose)
        # self.broad_cast()
        rospy.spin()


    def config(self):
        self.follower_pose_topic = rospy.get_param('~robot_pose_topic', '/mur620d/mir_pose_simple')
  

    def callback_follower_pose(self,data):
        self.pose = Pose()
        self.pose.position = data.position
        self.pose.orientation = data.orientation
        rospy.loginfo("pose_received")
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "mur_pose"
        t.transform.translation = self.pose.position
        t.transform.rotation = self.pose.orientation
        br.sendTransform(t)
        rospy.loginfo("running")

    def broad_cast(self):   
        pass
      


        return 


if __name__ == "__main__":
    instance = follower()

