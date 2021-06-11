#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


def publish_odom_frame(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.position.x
    t.transform.translation.y = msg.position.y
    t.transform.translation.z = msg.position.z
    
    t.transform.rotation.x = msg.orientation.x
    t.transform.rotation.y = msg.orientation.y
    t.transform.rotation.z = msg.orientation.z
    t.transform.rotation.w = msg.orientation.w

    br.sendTransform(t)
    
    """ 
    t2 = geometry_msgs.msg.TransformStamped()
    
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_link"
    t2.child_frame_id = "base_laser"
    t2.transform.translation.x = 0
    t2.transform.translation.y = 0
    t2.transform.translation.z = 0
    
    t2.transform.rotation.x = 0
    t2.transform.rotation.y = 0
    t2.transform.rotation.z = 0
    t2.transform.rotation.w = 1
    br.sendTransform(t2)
    
    t3 = geometry_msgs.msg.TransformStamped()
    
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "map"
    t3.child_frame_id = "odom"
    t3.transform.translation.x = 0
    t3.transform.translation.y = 0
    t3.transform.translation.z = 0
    
    t3.transform.rotation.x = 0
    t3.transform.rotation.y = 0
    t3.transform.rotation.z = 0
    t3.transform.rotation.w = 1
    br.sendTransform(t3)
    """
    
if __name__ == '__main__':
    rospy.init_node('odometry_frame_publisher')
    rospy.Subscriber('odometry_frame', geometry_msgs.msg.Pose, publish_odom_frame)
    rospy.spin()
