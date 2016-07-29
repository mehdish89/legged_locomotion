#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace std;


ros::Publisher pub;

void update(const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg)
{
	tf::TransformListener listener;

	gazebo_msgs::ModelStates out;
	out.name.push_back("first"); 
	out.name.push_back("daedalus");
	out.pose.push_back(geometry_msgs::Pose());
	out.pose.push_back(geometry_msgs::Pose());
	//cout << msg->detections.size() << endl;

	tf::StampedTransform transformed;
	try {
		string destination_frame = "/tag_1";
		string original_frame = "/tag_0";
		listener.waitForTransform(destination_frame, original_frame, ros::Time(0), ros::Duration(1));
		listener.lookupTransform(destination_frame, original_frame, ros::Time(0), transformed);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		//ros::Duration(1.0).sleep();
		return;
	}
	/*
	int count = 0;
		
	for(int i=0;i<msg->detections.size();i++)
		if(msg->detections[i].id==0 || msg->detections[i].id==0)
			count++;
	*/	
	{
		out.pose[1].position.x = transformed.getOrigin().x();
		out.pose[1].position.y = transformed.getOrigin().y();
		out.pose[1].position.z = transformed.getOrigin().z();
		
		out.pose[1].orientation.x = transformed.getRotation().x();
		out.pose[1].orientation.y = transformed.getRotation().y();
		out.pose[1].orientation.z = transformed.getRotation().z();
		out.pose[1].orientation.w = transformed.getRotation().w();
		pub.publish(out);
	}

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "model_translate");
	ros::NodeHandle n;
	pub = n.advertise<gazebo_msgs::ModelStates>("/gazebo/model_states", 1);
	ros::Subscriber sub = n.subscribe("/tag_detections", 1, update);
	
	ros::Rate rate(10.0);
	ros::spin();
}


