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
#include <string.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <boost/bind.hpp>

#include <collect_data/MS4525.h>

float CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C = 1.225;
float CONSTANTS_AIR_GAS_CONST = 287.1;
float CONSTANTS_ABSOLUTE_NULL_CELSIUS = -273.15;
float P_min = -1.0;
float P_max = 1.0;
float PSI_to_Pa = 6894.757;

double airspeed;
double temp;
double moving_avg;
double avg[20];
double sum_avg;
int current_avg_index;
int valid;

static int fd;

class SensorMS4525
{
public:

    SensorMS4525(ros::NodeHandle *nh)
    {
        // Inicializa variáveis e ROS publisher
        publisher =
            nh->advertise<collect_data::MS4525>("/ms4525", 10);
        fd = open("/dev/i2c-1", O_RDWR);
        airspeed = 0.0;
        current_avg_index = 0;
        sum_avg = 0;
        valid = 0;
        memset(avg, -1024, sizeof(avg));
    }

    float calc_indicated_airspeed(float differential_pressure)
    {

        if (differential_pressure > 0.0)
        {
            return sqrtf((2.0 * differential_pressure) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

        }
        else
        {
            return -sqrtf((2.0 * fabsf(differential_pressure)) / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);
        }

    }
    float get_air_density(float static_pressure, float temperature_celsius)
    {
        return static_pressure / (CONSTANTS_AIR_GAS_CONST * (temperature_celsius - CONSTANTS_ABSOLUTE_NULL_CELSIUS));
    }
    float calc_true_airspeed_from_indicated(float speed_indicated, float pressure_ambient, float temperature_celsius)
    {

        return speed_indicated * sqrtf(CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C / get_air_density(pressure_ambient,
                                       temperature_celsius));
    }

    void readSensorData()
    {
        try
        {
            unsigned char bytes[4] = {0, 0, 0, 0};
            ioctl(fd, I2C_SLAVE, 0x28);
            size_t returned = read(fd, bytes, 4);
            char status = (bytes[0] & 0xC0) >> 6;

	    if(returned == -1 || returned == 0) {
		    valid = 0;
		    return;
	    }

            if(status == 0)
            {
                int dp_raw = 0, dT_raw = 0;
                dp_raw = (bytes[0] << 8) + bytes[1];
                dp_raw = 0x3FFF & dp_raw;
                dT_raw = (bytes[2] << 8) + bytes[3];
                dT_raw = (0xFFE0 & dT_raw) >> 5;

                float temperature = ((200.0f * dT_raw) / 2047) - 50; //calculate temperature for future usages
                float diff_press_PSI = -((dp_raw - 0.1f * 16383) * (P_max - P_min) / (0.8f * 16383) + P_min);
                float diff_press_pa_raw = diff_press_PSI * PSI_to_Pa;
                double airspeed_aux = calc_indicated_airspeed(diff_press_pa_raw);

		if(airspeed_aux >= 0)
		{

                	float airspeed_to_remove = avg[current_avg_index]; // Pega o valor para remover da soma para média móvel
	                sum_avg -= airspeed_to_remove;                     // Remove o valor da soma da média
	                avg[current_avg_index] = airspeed_aux;             // Coloca o novo valor no vetor 
	                sum_avg += airspeed_aux;                           // Adiciona o novo valor na soma da média
	                current_avg_index = (current_avg_index + 1) % 20;  // Calcula o novo índice de remoção do vetor de valores
	
	                airspeed = sum_avg / 20;
	                temp = temperature;
	                valid = 1;
		}        
	    } else {
                valid = 0;
            }
        }
        catch (...)
        {
            valid = 0;
        }
    }
    void publishData()
    {
        if(valid && avg[19] != 1024)
        {
            msg.indicated_airspeed = airspeed;
            msg.temperature = temp;
            msg.valid = 1;
            publisher.publish(msg);
        }
        else
        {
            msg.indicated_airspeed = airspeed;
            msg.temperature = temp;
            msg.valid = 0;
            publisher.publish(msg);
        }
    }
private:
    collect_data::MS4525 msg;

    ros::Publisher publisher;
    ros::Publisher temperature_publisher;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ms4525");
    ros::NodeHandle nh;
    // Instância do sensor
    SensorMS4525 sensor(&nh);
    // ROS timer para ler dados do sensor
    ros::Timer timerReadData =
        nh.createTimer(ros::Duration(1.0 / 100.0),
                       boost::bind(&SensorMS4525::readSensorData, sensor));
    // ROS timer para publicar os dados lidos
    ros::Timer timerPublishData =
        nh.createTimer(ros::Duration(1.0 / 10.0),
                       boost::bind(&SensorMS4525::publishData, sensor));
    ros::spin();
}
