#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <numeric>

const float CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C = 1.225f;
const float CONSTANTS_AIR_GAS_CONST = 287.1f;
const float CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15f;
static int fd;

class AirSpeedSensor
{
public:
    const float P_min = -1.0f;
    const float P_max = 1.0f;
    const float PSI_to_Pa = 6894.757f;
    AirSpeedSensor(ros::NodeHandle *nh)
    {
        // Initialize airspeed, i2c and ROS publisher
        airspeed = 0.0;
        airspeedPublisher =
            nh->advertise<std_msgs::Float64>("/airspeed", 10);
    }

    float calc_indicated_airspeed(float differential_pressure)
    {

        if (differential_pressure > 0.0f)
        {
            return sqrtf((2.0f * differential_pressure) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

        }
        else
        {
            return -sqrtf((2.0f * fabsf(differential_pressure)) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
        }

    }
    float calc_true_airspeed_from_indicated(float speed_indicated, float pressure_ambient, float temperature_celsius)
    {
        float get_air_density(float static_pressure, float temperature_celsius)
        {
            return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
        }
        return speed_indicated * sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / get_air_density(pressure_ambient,
                                       temperature_celsius));
    }
    double readAirSpeedSensorData()
    {
        try
        {
            unsigned char bytes[4] = {0, 0, 0, 0};
            ioctl(fd, I2C_SLAVE, 0x28);
            read(fd, bytes, 4);
            char status = (bytes[0] & 0xC0) >> 6;
            int dp_raw = 0, dT_raw = 0;
            dp_raw = (bytes[0] << 8) + bytes[1];
            dp_raw = 0x3FFF & dp_raw;
            dT_raw = (bytes[2] << 8) + bytes[3];
            dT_raw = (0xFFE0 & dT_raw) >> 5;
            float temperature = ((200.0f * dT_raw) / 2047) - 50; //calculate temperature for future usages
            float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
            float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
            airspeed = calc_indicated_airspeed(diff_press_pa_raw);
            return true;
        }
        catch (int e)
        {
            return false;
        }
    }
    void publishAirspeed()
    {
        if(readAirSpeedSensorData())
        {
            std_msgs::Float64 msg;
            msg.data = airspeed;
            airspeedPublisher.publish(msg);
        } else {
            std_msgs::Float64 msg;
            msg.data = -1f;
            airspeedPublisher.publish(msg);
        }
    }
private:
    double airspeed;
    ros::Publisher airspeedPublisher;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "airspeed");
    ros::NodeHandle nh;
    // Create an instance of Temperature sensor
    AirSpeedSensor airspeedSensor(&nh);
    // Create a ROS timer for reading data
    ros::Timer timerReadAirspeed =
        nh.createTimer(ros::Duration(1.0 / 100.0),
                       std::bind(&AirSpeedSensor::readAirSpeedSensorData, airspeedSensor));
    // Create a ROS timer for publishing temperature
    ros::Timer timerPublishAirspeed =
        nh.createTimer(ros::Duration(1.0 / 10.0),
                       std::bind(&AirSpeedSensor::publishAirspeed, airspeedSensor));
    ros::spin();
}