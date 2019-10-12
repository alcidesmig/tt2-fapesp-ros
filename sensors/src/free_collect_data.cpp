/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>

#include <std_msgs/Float64.h>


#include <stdlib.h>
#include <string.h>


#define FILENAME "/tmp/data.txt"

std_msgs::Float64 compass;
void get_compass_value(const std_msgs::Float64::ConstPtr &msg)
{
    compass = *msg;
}

std_msgs::Float64 temperature;
void get_temperature_value(const std_msgs::Float64::ConstPtr &msg)
{
    temperature = *msg;
}

std_msgs::Float64 airspeed;
void get_airspeed_value(const std_msgs::Float64::ConstPtr &msg)
{
    airspeed = *msg;
}


FILE *fp;
int cont = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_node");

    // Publicação do Pose
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 1);
    // Sensor de temperatura
    ros::Subscriber temperature_sub = nh.subscribe<std_msgs::Float64>
                                      ("temperature", 1, get_temperature_value);
    // Sensor de airspeed
    ros::Subscriber airspeed_sub = nh.subscribe<std_msgs::Float64>
                                   ("airspeed", 1, get_airspeed_value);
    // Dados da bússola
    ros::Subscriber compass_sub = nh.subscribe<std_msgs::Float64>
                                  ("mavros/global_position/compass_hdg", 1, get_compass_value);

    ros::Rate rate(100.0); 

    while(ros::ok())
    {
        cont++;
        if(cont > 100) {
            fclose(fp);
            fp = fopen(FILENAME, "ab+");
            if(fp == NULL) {
                return;
            }
        }
        fprintf(fp, "%f;%f;%f\n", temperature.data, airspeed.data, compass.data);
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
