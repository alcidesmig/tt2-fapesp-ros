/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <math.h>

#include <stdlib.h>
#include <string.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>

#include <collect_data/HDC1050.h>
#include <collect_data/MS4525.h>

#include <sensor_msgs/MagneticField.h>

#define FILENAME "/tmp/data.txt"

std_msgs::Float64 compass;
void get_compass_value(const std_msgs::Float64::ConstPtr &msg)
{
    compass = *msg;
}

collect_data::HDC1050 hdc1050;
void get_hdc1050_data(const collect_data::HDC1050::ConstPtr &msg)
{
    hdc1050 = *msg;
}

collect_data::MS4525 ms4525;
void get_ms4525_data(const collect_data::MS4525::ConstPtr &msg)
{
    ms4525 = *msg;
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

sensor_msgs::MagneticField mag_compass;
void get_mag_compass_value(const sensor_msgs::MagneticField::ConstPtr &msg)
{
    mag_compass = *msg;
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

double mag_to_compass(float x, float y) {
    return atan2(y, x) * 180 / M_PI;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_node");

    ros::NodeHandle nh;

    ros::Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField>("mavros/imu/mag", 1, get_mag_compass_value);

    // Sensor de temperatura
    ros::Subscriber hdc1050_sub = nh.subscribe<collect_data::HDC1050>
                                  ("hdc1050", 1, get_hdc1050_data);
    // Sensor de airspeed
    ros::Subscriber ms4525_sub = nh.subscribe<collect_data::MS4525>
                                 ("ms4525", 1, get_ms4525_data);
    // Dados da bússola
    ros::Subscriber compass_sub = nh.subscribe<std_msgs::Float64>
                                  ("mavros/global_position/compass_hdg", 1, get_compass_value);
    // Pressão ambiente para o cálculo do true airspeed
    ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>
                                   ("mavros/imu/static_pressure", 1, get_pressure_value);
    // Dados de GPS
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
                              ("mavros/global_position/raw/fix", 1, get_gps_value);

    fp = fopen(FILENAME, "a+");
    if(fp != NULL)
    {
        fprintf(fp, "Zero do sensor de airspeed: indicado(%f) true(%f) - Dados válidos: %d", ms4525.indicated_airspeed, calc_true_airspeed_from_indicated(ms4525.indicated_airspeed, pressure_ambient.fluid_pressure, ms4525.temperature), ms4525.valid);
    }
    fclose(fp);

    ros::Rate rate(10.0);

    while(ros::ok())
    {
        
	try
        {
            cont++;
            if(cont > 100)
            {
                fclose(fp);
                fp = fopen(FILENAME, "a+");
                if(fp == NULL)
                {
                    return 0;
                }
            }
            float indicated_airspeed = ms4525.indicated_airspeed;
            float true_airspeed = calc_true_airspeed_from_indicated(indicated_airspeed, pressure_ambient.fluid_pressure, ms4525.temperature);
            float compass_mag = mag_to_compass(mag_compass.magnetic_field.x, mag_compass.magnetic_field.y);
	    ROS_INFO("%f;%f;%f;%f;%lf;%f;%f;%f;%d\n",
                            hdc1050.temperature, indicated_airspeed, true_airspeed, hdc1050.humidity, compass_mag, gps.latitude, gps.longitude, gps.altitude, (int) time(NULL)
                           );
	    if(fp != NULL)
            {
                if(!ms4525.valid)
                {
                    fprintf(fp, "Dados do MS4525 inválidos\n");
                }
                if(!hdc1050.valid)
                {
                    fprintf(fp, "Dados do HDC1050 inválidos\n");
                }
                if(hdc1050.valid && ms4525.valid)
                {
                    fprintf(fp,
                            "%f;%f;%f;%f;%lf;%f;%f;%f;%d\n",
                            hdc1050.temperature, indicated_airspeed, true_airspeed, hdc1050.humidity, compass_mag, gps.latitude, gps.longitude, gps.altitude, (int) time(NULL)
                           );
                }
            } else {
	    	ROS_INFO("Erro FP");
	    }
        }
        catch(...)
        {
            ROS_INFO("ERROR - COLLECTING");
        }
        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
