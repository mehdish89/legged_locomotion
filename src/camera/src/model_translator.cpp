#include "ros/ros.h"
#include <gazebo_msgs/ModelStates.h>



ros::Publisher pub;
ros::Subscriber sub;

using namespace std;


void update(const gazebo_msgs::ModelStates::ConstPtr& msg)
{

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "model_translate");
	ros::NodeHandle n;
	pub = n.advertise<gazebo_msgs::ModelStates>("/gazebo/model_states", 1);
	sub = n.subscribe("/tag_detections", 1, update);
	ros::Rate rate(10.0);
	ros::spin();
}


