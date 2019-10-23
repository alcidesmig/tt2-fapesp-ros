// edit of https://roboticsbackend.com/roscpp-timer-with-ros-publish-data-at-a-fixed-rate/
#include <ros/ros.h>

#include <std_msgs/Float64.h>

#include <vector>
#include <numeric>

#include <sys/ioctl.h>
#include <sys/types.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <boost/bind.hpp>

#include <collect_data/HDC1050.h>

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

float temperature, humidity;
int valid;

class SensorHDC1050
{
public:
    SensorHDC1050(ros::NodeHandle *nh)
    {
        // Initialize temperature, i2c and ROS publisher
        temperature = 0;
        humidity = 0;
        valid = 0;
        dataPublisher =
            nh->advertise<collect_data::HDC1050>("/hdc1050", 10);
        fd = open("/dev/i2c-1", O_RDWR);
    }
    bool readSensorData()
    {
        try
        {
	    double i2c_value = i2c_read(0x40, 0, 20000); // 0 = temperature, 1 = humidity
	    temperature = i2c_value * (165.0 / 65536.0) - 40;
	    i2c_value  = i2c_read(0x40,1,20000);
            humidity = i2c_value  * (100.0/65536.0);
            valid = 1;
        }
        catch (...)
        {
            valid = 0;
        }
    }
    void publishData()
    {
        if(valid)
        {
            msg.temperature = temperature;
            msg.humidity = humidity;
            msg.valid = 1;
            dataPublisher.publish(msg);
        } else {
            msg.temperature = temperature;
            msg.humidity = humidity;
            msg.valid = 0;
            dataPublisher.publish(msg);
        }
    }
private:
    collect_data::HDC1050 msg;
    ros::Publisher dataPublisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hdc1050");
    ros::NodeHandle nh;
    // Cria uma instância do sensor
    SensorHDC1050 sensor(&nh);
    // Cria um ROS timer para ler dados
    ros::Timer timerReadData =
    	nh.createTimer(ros::Duration(1.0 / 100.0),
                       boost::bind(&SensorHDC1050::readSensorData, sensor));
    // Cria um ROS timer para publicar dados
    ros::Timer timerPublishData =
        nh.createTimer(ros::Duration(1.0 / 10.0),
                       boost::bind(&SensorHDC1050::publishData, sensor));
    ros::spin();
}
