#! /usr/bin/env python

## import ros stuff
import rospy
import time
from std_srvs.srv import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist, Pose, PoseStamped

import math

pub_move_base=None
sub_odom=None
sub_position=None
pub_twist=None
pub_odom=None

position=Odometry()



target_position=Point()
#Initialize a value for the target position, it will be uploaded in the callback
target_position.x=8
target_position.y=6

actual_pos=Pose()





def positionCallback(msg):
    global actual_pos, pub_odom, position
    actual_pos=msg.position

    position.header.frame_id='odom'
    position.pose.pose.position=msg.position
    position.pose.pose.orientation=msg.orientation

    print(position)
    pub_odom.publish(position)
  #  print("\nPosition callback")
    return



def clbk_target(pos):
    global target_position

    target_position=pos.pose.position
    print("\ntarget settato")
    moveRobot()

    return


def moveRobot():
    global target_position, pub_move_base

    move_goal = MoveBaseActionGoal()
    move_goal.goal.target_pose.header.frame_id="map"
    move_goal.goal.target_pose.pose.orientation.w=1

    move_goal.goal.target_pose.pose.position.x = target_position.x
    move_goal.goal.target_pose.pose.position.y = target_position.y
    move_goal.goal.target_pose.pose.position.z = target_position.z
    
    pub_move_base.publish(move_goal)  
    print(move_goal) 
    print("\nMove Robot done")

    return

def distance():
    global target_position, actual_pos
    
    dist_x= actual_pos.x-target_position.x
    dist_y= actual_pos.y-target_position.y

    dist=math.sqrt(pow(dist_x, 2)+pow(dist_y, 2))
    print("\nCalcolo distanza")
	
    return dist





def main():
    global pub_move_base, pub_twist, sub_position, sub_odom, pub_odom
	
    rospy.init_node('navigation')
    
    
    ## Subscribe to Odom service to get the robot position
    sub_odom=rospy.Subscriber("/odometry_frame", Pose, positionCallback)
    
   
   #Pubblisher to pubblsh the poition in wich the robot has to go and the velocity to set
    pub_move_base=rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)  
    pub_odom=rospy.Publisher('/odom', Odometry, queue_size=1)      
    sub_position=rospy.Subscriber('move_base_simple/goal', PoseStamped, clbk_target)
    pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
   # sub_odom=rospy.Subscriber('/odom', Odometry, odom_clbk)

    rate = rospy.Rate(20)

    rospy.spin()

        

    

if  __name__ == '__main__':
	## Since there are other scripts to be launched at the beginning this program waits 2 seconds, so the logs of the scripts aren't printed together
    time.sleep(2)
    main()