#include "ros/ros.h"
#include "mobile_robot_navigation_project/RandomPosition.h"
#include <ctime>

/**Function to obtain a random position*/
bool myrandom (mobile_robot_navigation_project::RandomPosition::Request &req, mobile_robot_navigation_project::RandomPosition::Response &res){
	
    float x_values[]={-3.7, -7.87, 5.63, -5.55, 7.12, -4.85};
	float y_values[]={8.41, 8.4, -0.19, -3.94, -6.76, -8.27};
	
	srand((unsigned) time(0));
	
	/**Generate a random number*/
	int index=rand()%6;  
	
	    /**Use the random number as an index for the array*/
	    res.x = x_values[index];
        res.y = y_values[index];
        ROS_INFO("\nServer position: x[%f] y[%f]", res.x, res.y);
    return true;
}



int main(int argc, char **argv)
{
   ros::init(argc, argv, "position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position", myrandom);
   ROS_INFO("Server activated \n");
   ros::spin();

   return 0;
}
