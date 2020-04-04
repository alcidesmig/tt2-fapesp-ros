// Código teste: pegar posição do drone

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped pos;

void print_pos(const geometry_msgs::PoseStamped::ConstPtr & msg) {
	pos = *msg;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_drone_position");
	ros::NodeHandle nh;

	ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
	                          ("mavros/local_position/pose", 10, print_pos);

	ros::Rate rate(20.0);

	ros::spin();

	return 0;
}

