// (ok) gravar indic e true airspeed, humidade, lat, long, timestamp
// fazer checagem dos sensores, do arquivo dentro do nó master, acender led = ok, piscar led = !ok
// (ok) mudar para 10s a rotação
// (ok) quaternion para determinar parada, quando bater 2pi + offset
// (ok) arrumar arquivo = segfault
// (ok) tratar excessao do arquivo para n crashar ros
// (ok) gravar o 0 do sensor de airspeed (true, indicated) para fazer compensação (tapar tubo quando ligar o sistema)


// to do: serviço para calcular o true airspeed

// quarta manhã 8h usp2

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

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>

#include <std_msgs/Float64.h>

#include <stdlib.h>
#include <string.h>

#include <wiringPi.h> // http://wiringpi.com/download-and-install/

#include <math.h>

#include <time.h>

#include <collect_data/HDC1050.h>
#include <collect_data/MS4525.h>

#define PIN 4

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_drone_system");

    ros::NodeHandle nh;

    wiringPiSetup();
    pinMode(PIN, OUTPUT);

    pullUpDnControl(4, PUD_DOWN);


    //    ms4525.valid = 0;
    //    hdc1050.valid = 0;

    //digitalWrite(PIN, LOW);           E
    digitalWrite(PIN, 1); // Invertido por conta da NOT

    // Status do drone
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 1, state_cb);
    // Sensor de temperatura
    ros::Subscriber hdc1050_sub = nh.subscribe<collect_data::HDC1050>
                                  ("hdc1050", 1, get_hdc1050_data);
    // Sensor de airspeed
    ros::Subscriber ms4525_sub = nh.subscribe<collect_data::MS4525>
                                 ("ms4525", 1, get_ms4525_data);
    // Espera conexão com a PX
    while(!current_state.connected)
    {
        ROS_INFO("Status: NOT OK. Waiting PX4 Connection.");
        sleep(1);
    }

    // Abre arquivo dos parâmetros
    FILE *fp = fopen("/home/pi/parameters.txt", "r");

    // Formato: [velocidade rotação (2*pi/360/VALOR_PARAMETRO)] [altitude coleta minima] [altitude coleta máxima]
    float alt_1, alt_2, alt_3, vel;
    int return_scanf = fscanf(fp, "%f %f %f %f", &vel, &alt_1, &alt_2, &alt_3);
    fclose(fp);

    fp = fopen("/mnt/pendrive/guarantee", "w");
    // Variável que indica que os parâmetros estão OK.
    bool parameters_ok = fp != NULL && return_scanf == 4 && (alt_1 >= 1 && alt_1 <= 100 && alt_2 >= 1 && alt_2 <= 100 && alt_3 >= 1 && alt_3 <= 100 && vel > 0 && vel <= 50);
    fclose(fp);
    sleep(10);

    // Verifica se os dados dos sensores são válidos
    if(parameters_ok && (hdc1050.valid == 1) && (ms4525.valid == 1) && current_state.connected)
    {
        //digitalWrite(PIN, HIGH);
        digitalWrite(PIN, 0); // Acende o LED caso tudo ok
    }
    else
    {
        //digitalWrite(PIN, LOW);
        digitalWrite(PIN, 1); // Garantia do LED apagado
    }

    fclose(fp);

    return 0;
}
