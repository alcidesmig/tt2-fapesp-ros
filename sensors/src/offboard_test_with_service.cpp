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
#include <mavros_msgs/WaypointReached.h>
#include <stdlib.h>
#include <string.h>
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped pos;
void get_pos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos = *msg;
}

mavros_msgs::WaypointReached waypoint_num;
void get_waypoint(const mavros_msgs::WaypointReached::ConstPtr &msg)
{
    waypoint_num = *msg;
}


int status = -1;
double offboard_enabled = -1;
int last_waypoint = -1;
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
    ros::Subscriber waypoint_sub = nh.subscribe<mavros_msgs::WaypointReached>
                                   ("mavros/mission/reached", 10, get_waypoint);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);


    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pos.pose.position.x;
    pose.pose.position.y = pos.pose.position.y;
    pose.pose.position.z = pos.pose.position.z + 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode.request.base_mode = 0;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    last_waypoint = waypoint_num.wp_seq;
    ROS_INFO_STREAM("First value for waypoint_num:" << waypoint_num.wp_seq);

    while(ros::ok())
    {
        if(last_waypoint != waypoint_num.wp_seq && status == -1)
        {
            last_waypoint = waypoint_num.wp_seq;
            status = 0;
            ROS_INFO_STREAM("Waypoint reached:" << waypoint_num.wp_seq);

            pose.pose.position.x = pos.pose.position.x;
            pose.pose.position.y = pos.pose.position.y;
            pose.pose.position.z = pos.pose.position.z + 2;
            local_pos_pub.publish(pose);

        }
        if(status == 0)
        {
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if(current_state.mode != "OFFBOARD" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Trying to switch to Offboard mode");
            }
            if(current_state.mode == "OFFBOARD" && offboard_enabled == -1)
            {
                ROS_INFO("Offboard enabled");
                offboard_enabled = ros::Time::now().toSec();
            }
            if(offboard_enabled != -1 && ros::Time::now().toSec() - offboard_enabled > 10)
            {
                status = 1;
            }
        }
        else if (status == 1 && current_state.mode != "AUTO.MISSION")
        {
            offb_set_mode.request.custom_mode = "AUTO.MISSION";
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Collect data: ok");
                ROS_INFO("AUTO.PILOT = MISSION enabled");
                status = -1;
            }
            offboard_enabled = -1;
        }

        for(int i = 100; ros::ok() && i > 0; --i)
        {
            local_pos_pub.publish(pose);
            //ros::spinOnce();
            rate.sleep();
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

/*   //send a few setpoints before starting

       for(int i = 100; ros::ok() && i > 0; --i)
       {
           local_pos_pub.publish(pose);
           ros::spinOnce();
           rate.sleep();
       }*/