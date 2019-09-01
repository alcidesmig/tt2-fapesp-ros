/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped pos;
void get_pos(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pos = *msg;
}


int status = 0;
double start_zero = -1;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, get_pos);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pos.pose.position.x;
    pose.pose.position.y = pos.pose.position.y;
    pose.pose.position.z = pos.pose.position.z + 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
	ROS_INFO_STREAM("ST\n" << status);
        if(status == 0) {
		if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            		if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                		ROS_INFO("Offboard enabled\n");
            			start_zero = ros::Time::now().toSec();
			}
			
            		last_request = ros::Time::now();
        	} else {
            		if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                		if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    			ROS_INFO("Vehicle armed\n");
                		}
                		last_request = ros::Time::now();
            		}
        	}
		if(start_zero != -1 && ros::Time::now().toSec() - start_zero > ros::Duration(10).toSec()) {
			status = 1;
		}
        	local_pos_pub.publish(pose);
	} else if (status == 1 && current_state.mode != "AUTO.MISSION") {
		offb_set_mode.request.custom_mode = "AUTO.MISSION";
		if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                	ROS_INFO("Collect data: ok\nMission enabled");
                	status = 2;
		}
		
		
	}

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

