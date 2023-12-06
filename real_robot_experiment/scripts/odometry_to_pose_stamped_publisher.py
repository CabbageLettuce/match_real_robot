#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose,Point,Quaternion
from nav_msgs.msg import Odometry


pose_stamped_publisher = None

def odometry_callback(msg):
    global pose_stamped_publisher
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header = msg.header
    pose_stamped_msg.pose = msg.pose.pose
    pose_stamped_publisher.publish(pose_stamped_msg)
    rospy.loginfo('publish posestamped')   

if __name__ == "__main__":
    try:
        rospy.init_node("odometry_to_pose_stamped_publisher",anonymous=True)
        pose_stamped_publisher = rospy.Publisher("pose_stamped_from_odometry",PoseStamped,queue_size=10)
        odometry_subscriber = rospy.Subscriber("/mur620a/odom",Odometry,odometry_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass