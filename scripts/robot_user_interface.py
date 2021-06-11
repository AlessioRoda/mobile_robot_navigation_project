#! /usr/bin/env python

## import ros stuff
import rospy
import time
from std_srvs.srv import *
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from mobile_robot_navigation_project.srv import RandomPosition
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist, Pose, PoseStamped

import math

actual_position=Point()

## Variables to save the value of the target robot has to reach
desired_position_=Point()

## Variable to generate a message of reached position in the positionCallback
notify=False

set_target=False

## Initialize the distance to the target
dist=0


## Initialize the publishers
pub_move_base=None
pub_twist=None

## Initialize the odom subscriber
sub_odom=None



## Callback to constantly get the robot position
def positionCallback(msg):
	
    global actual_position, goal_x, goal_z, notify
    actual_position=msg.pose.pose.position
    
    if (distance()<=0.4 and notify==True):
	    print("\nTarget reached!")
	    notify=False
    
def clbk_position(pos):
    global desired_position_

    desired_position_.x=pos.pose.position.x
    desired_position_.y=pos.pose.position.y
    desired_position_.z=pos.pose.position.z	
    
    
## Function to move in the direction received from the server     
def move_randomly():
	global srv_client_wall_follower, srv_pos, actual_position
	
	
	print("\nActual robot position: "+str(actual_position))

	resp=srv_pos()
	
	## Call the generic funtion to set the new target position
	set_target_position(resp.x, resp.z)
	
	return
    
	

## To stop the robot the linear velocity values are set to 0, evenmore the target to reach is set to the position in wich robot is, so that to make sure it won't try to reach another position
def stop_robot():
	
	global set_target, pub_move_base, srv_client_wall_follower, pub_twist	
	
	## Set the value of velocity to 0	
	velocity=Twist()
	velocity.linear.x=0
	velocity.linear.y=0
		
	
	pub_twist.publish(velocity)
	
	return
	
	
	
## This function get two parameters, target_x and target_y and set the target position in wich robot has to go
def set_target_position(target_x, target_z):
	
	global resp, srv_client_user_interface, set_target, pub_move_base
	global desired_position_, notify
	
	## Initialize a MoveBaseActionGoal target to move my robot
	move_goal = MoveBaseActionGoal()
	move_goal.goal.target_pose.header.frame_id="map"
	move_goal.goal.target_pose.pose.orientation.w=1

	move_goal.goal.target_pose.pose.position.x = desired_position_.x
	move_goal.goal.target_pose.pose.position.y = desired_position_.y
	move_goal.goal.target_pose.pose.position.z = desired_position_.z
	
	pub_move_base.publish(move_goal)
	
	print("\n Let's reach the position x="+str(target_x)+" z="+str(target_z))
	
	## Set the goal position as the one I received 
	goal_x=target_x
	goal_z=target_z
	
	## Set the variable notify to true to be allerted when the robot reaches the target
	notify=True
	
	## Print the distance beteen the robot and the target
	print ("\n Distance to the target: "+str(distance()))
	
	
	return
    
	
## Function to estimate the value of the distance between the robot positon and the target it has to reach
def distance():
	
	global goal_x, goal_z, actual_position
	
	dist_x= actual_position.x-goal_x
	dist_z= actual_position.z-goal_z
	
	## To estimate the distance it's used the vector distance from the coordinate of the position to the actual robot position
	dist=math.sqrt(pow(dist_x, 2)+pow(dist_z, 2))
	
	return dist
	
		


def main():
	
    global srv_client_wall_follower, srv_pos, pub_move_base, pub_twist, set_target
	
    rospy.init_node('robot_user_interface')
    
    
    ## Subscribe to Odom service to get the robot position
    sub_odom=rospy.Subscriber("/odometry_frame", Pose, positionCallback)
    
   
   #Pubblisher to pubblsh the poition in wich the robot has to go and the velocity to set
    pub_move_base=rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=1)
	sub_position=rospy.Subscriber('move_base_simple/goal', PoseStamped, clbk_postion)
    pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
   
    ## Connect to the server_second_assignment node, the server that provides to give the random position
    srv_pos= rospy.ServiceProxy('/position', RandomPosition)
    
    
    rate = rospy.Rate(20)
    
    ## For the entire execution of the program it's shown an user interface to obtain the command the robot has to execute:
    ## robot can move in a random position, move in a specific position, follow the walls or stop in the last position.
    while not rospy.is_shutdown():

        ## Initialize variable command to 
		#  
		     
	    command=int(input())
		
	    if(command==1):
			  
		    move_randomly()

		
		

    
		
			

if __name__ == '__main__':
	## Since there are other scripts to be launched at the beginning this program waits 2 seconds, so the logs of the scripts aren't printed together
    time.sleep(2)
    main()
