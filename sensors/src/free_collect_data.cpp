/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>

#include <std_msgs/Float64.h>


#include <stdlib.h>
#include <string.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>

#define FILENAME "/mnt/pendrive/data.txt"

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

std_msgs::Float64 temperature_airspeed;
void get_temperature_airspeed_value(const std_msgs::Float64::ConstPtr &msg)
{
    temperature_airspeed = *msg;
}

std_msgs::Float64 airspeed;
void get_airspeed_value(const std_msgs::Float64::ConstPtr &msg)
{
    airspeed = *msg;
}

sensor_msgs::NavSatFix gps;
void get_gps_value(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    gps = *msg;
}

sensor_msgs::FluidPressure pressure_ambient;
void get_pressure_value(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    pressure_ambient = *msg;
}

std_msgs::Float64 humidity;
void get_hum_temperature_value(const std_msgs::Float64::ConstPtr &msg)
{
    humidity = *msg;
}

static float CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C = 1.225;
static float CONSTANTS_AIR_GAS_CONST = 287.1;
static float CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15;
float get_air_density(float static_pressure, float temperature_celsius)
{
    return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
}
float calc_true_airspeed_from_indicated(float speed_indicated, float pressure_ambient, float temperature_celsius)
{

    return speed_indicated * sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / get_air_density(pressure_ambient,
                                   temperature_celsius));
}
FILE *fp;
int cont = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_node");

    ros::NodeHandle nh;

    // Sensor de temperatura
    ros::Subscriber temperature_sub = nh.subscribe<std_msgs::Float64>
                                      ("temperature", 1, get_temperature_value);
    // Sensor de temperatura ~ umidade
    ros::Subscriber humidity_sub = nh.subscribe<std_msgs::Float64>
                                      ("humidity_temperature", 1, get_hum_temperature_value);
    // Sensor de airspeed
    ros::Subscriber airspeed_sub = nh.subscribe<std_msgs::Float64>
                                   ("airspeed", 1, get_airspeed_value);
    // Dados da bússola
    ros::Subscriber compass_sub = nh.subscribe<std_msgs::Float64>
                                  ("mavros/global_position/compass_hdg", 1, get_compass_value);
    // Temperatura (sensor airspeed)
    ros::Subscriber temperature_airspeed_sub = nh.subscribe<std_msgs::Float64>
                                      ("temperature_airspeed", 1, get_temperature_airspeed_value);
    // Pressão ambiente para o cálculo do true airspeed
    ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>
                                   ("/mavros/imu/static_pressure", 1, get_pressure_value);
                                       // Dados de GPS
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
                              ("mavros/global_position/global", 1, get_gps_value);

    fp = fopen(FILENAME, "a+");
    if(fp != NULL) fprintf(fp, "Zero do sensor de airspeed: indicado(%f) true(%f)", airspeed.data, calc_true_airspeed_from_indicated(airspeed.data, pressure_ambient.fluid_pressure, temperature_airspeed.data));
    fclose(fp);

    ros::Rate rate(10.0); 

    while(ros::ok())
    {
        try{
            cont++;
            if(cont > 100) {
                fclose(fp);
                fp = fopen(FILENAME, "a+");
                if(fp == NULL) {
                    return 0;
                }
            }
            float indicated_airspeed = airspeed.data;
            float true_airspeed = calc_true_airspeed_from_indicated(indicated_airspeed, pressure_ambient.fluid_pressure, temperature_airspeed.data);
            if(fp != NULL) {
                fprintf(fp, 
                    "%f;%f;%f;%f;%f;%f;%f;%d\n", 
                    temperature.data, indicated_airspeed, true_airspeed, humidity, compass.data, gps.latitude, gps.longitude, (int) time(NULL)
                );
            }
        }catch(...){
            ROS_INFO("ERROR - COLLECTING");
        }
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
