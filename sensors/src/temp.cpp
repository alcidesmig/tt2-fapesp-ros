// edit of https://roboticsbackend.com/roscpp-timer-with-ros-publish-data-at-a-fixed-rate/
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <numeric>
#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <pthread.h>
#include <boost/bind.hpp>

static int fd;

unsigned short i2c_read(unsigned char addr, unsigned char reg, int delay)
{
    static struct i2c_msg msgs[1];
    int r;

    struct i2c_rdwr_ioctl_data msgset = { msgs, sizeof(msgs) / sizeof(*msgs) };
    unsigned char buf[4];

    buf[0] = reg;
    msgs[0].addr = addr;
    msgs[0].flags = 0;
    msgs[0].buf = buf;
    msgs[0].len = 1;

    r = ioctl(fd, I2C_RDWR, &msgset);
    if (r < 0) return 0xffff;

    if (delay) usleep(delay);

    msgs[0].addr = addr;
    msgs[0].flags = I2C_M_RD;
    msgs[0].buf = buf;
    msgs[0].len = 2;

    r = ioctl(fd, I2C_RDWR, &msgset);
    if (r < 0) return 0xffff;

    return buf[0] * 256 + buf[1];
}



class TemperatureSensor
{
public:
    TemperatureSensor(ros::NodeHandle *nh)
    {
        // Initialize temperature, i2c and ROS publisher
        temperature = 0.0;
        temperaturePublisher =
            nh->advertise<std_msgs::Float64>("/temperature", 10);
        fd = open("/dev/i2c-1", O_RDWR);
    }
    bool readTemperatureSensorData()
    {
        try
        {
            double i2c_value = i2c_read(0x40, 0, 20000); // 0 = temperature, 1 = humidity
            temperature = i2c_value * (165.0 / 65536.0) - 40; //transferir umidade
            return true;
        }
        catch (int e)
        {
            return false;
        }
    }
    void publishTemperature()
    {
        if(readTemperatureSensorData())
        {
            std_msgs::Float64 msg;
            msg.data = temperature;
            temperaturePublisher.publish(msg);
        } else {
            std_msgs::Float64 msg;
            msg.data = -1;
            temperaturePublisher.publish(msg);
        }
    }
private:
    double temperature;
    ros::Publisher temperaturePublisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "temperature");
    ros::NodeHandle nh;
    // Create an instance of Temperature sensor
    TemperatureSensor temperatureSensor(&nh);
    // Create a ROS timer for reading data
    ros::Timer timerReadTemperature =
        nh.createTimer(ros::Duration(1.0 / 100.0),
                       boost::bind(&TemperatureSensor::readTemperatureSensorData, temperatureSensor));
    // Create a ROS timer for publishing temperature
    ros::Timer timerPublishTemperature =
        nh.createTimer(ros::Duration(1.0 / 10.0),
                       boost::bind(&TemperatureSensor::publishTemperature, temperatureSensor));
    ros::spin();
}