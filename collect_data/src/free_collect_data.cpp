// Código teste: coleta livre de dados

#include <ros/ros.h>

#include <std_msgs/Float64.h>


#include <stdlib.h>
#include <string.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/PoseStamped.h>

#include <collect_data/HDC1050.h>
#include <collect_data/MS4525.h>

#define FILENAME "/mnt/pendrive/data.txt"

std_msgs::Float64 compass;
void get_compass_value(const std_msgs::Float64::ConstPtr &msg)
{
    compass = *msg;
}

std_msgs::Float64 rel_alt;
void get_rel_alt(const std_msgs::Float64::ConstPtr &msg)
{
    rel_alt = *msg;
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

geometry_msgs::PoseStamped pos;
void get_pos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos = *msg;
}

sensor_msgs::Range lidar;
void get_lidar(const sensor_msgs::Range::ConstPtr &msg)
{
    lidar = *msg;
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
                                   ("/mavros/imu/static_pressure", 1, get_pressure_value);
    // Dados de GPS
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
                              ("mavros/global_position/global", 1, get_gps_value);
    ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1, get_rel_alt);

    // Pegar a posição do drone
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, get_pos);
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::Range>("mavros/distance_sensor/lidarlite_pub", 1, get_lidar);

    fp = fopen(FILENAME, "a+");
    if (fp != NULL)
    {
        fprintf(fp, "Zero do sensor de airspeed: indicado(%f) true(%f) - Dados válidos: %d", ms4525.indicated_airspeed, calc_true_airspeed_from_indicated(ms4525.indicated_airspeed, pressure_ambient.fluid_pressure, ms4525.temperature), ms4525.valid);
    }
    fclose(fp);

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        try
        {
            cont++;
            if (cont > 100)
            {
                fclose(fp);
                fp = fopen(FILENAME, "a+");
                if (fp == NULL)
                {
                    return 0;
                }
            }
            float indicated_airspeed = ms4525.indicated_airspeed;
            float true_airspeed = calc_true_airspeed_from_indicated(indicated_airspeed, pressure_ambient.fluid_pressure, ms4525.temperature);
            if (fp != NULL)
            {
                if (!ms4525.valid)
                {
                    fprintf(fp, "Dados do MS4525 inválidos\n");
                }
                if (!hdc1050.valid)
                {
                    fprintf(fp, "Dados do HDC1050 inválidos\n");
                }
                if (hdc1050.valid && ms4525.valid)
                {
                    fprintf(fp,
                            "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d\n",
                            pos.pose.position.z, rel_alt, lidar.range, hdc1050.temperature, indicated_airspeed, true_airspeed, hdc1050.humidity, compass.data, gps.latitude, gps.longitude, (int) time(NULL)
                           );
                }
            }
        }
        catch (...)
        {
            ROS_INFO("ERROR - COLLECTING");
        }

        ROS_INFO("Distance\n\tLidar: %f\n\tPose: %f\n\tRel_alt: %f", lidar.range, pos.pose.position.z, rel_alt);

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
