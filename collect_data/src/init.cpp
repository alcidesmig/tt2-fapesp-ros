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

//#include <wiringPi.h> // http://wiringpi.com/download-and-install/

#include <math.h>

#include <time.h>

#include <collect_data/HDC1050.h>
#include <collect_data/MS4525.h>

#define PIN 25

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

    ros::init(argc, argv, "drone_node");

    ros::NodeHandle nh;



    // Arquivo de status do sistema
    FILE * fp2;


  //  wiringPiSetup();
  //  pinMode(PIN, OUTPUT);

  //  digitalWrite(PIN, LOW);

    // Status do drone
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 1, state_cb);
    // Sensor de temperatura
    ros::Subscriber hdc1050_sub = nh.subscribe<collect_data::HDC1050>
                                  ("hdc1050", 1, get_hdc1050_data);
    // Sensor de airspeed
    ros::Subscriber ms4525_sub = nh.subscribe<collect_data::MS4525>
                                 ("ms4525", 1, get_ms4525_data);
    fp2 = fopen("/home/pi/status.txt", "a+");
    // Espera conexão com a PX
    while(!current_state.connected)
    {
        fprintf(fp2, "Status: NOT OK. Waiting PX4 Connection.");
	sleep(1);
    }

    fclose(fp2);

    // Abre arquivo dos paramêtros
    FILE *fp = fopen("/home/pi/parameters.txt", "r");

    // Formato: [velocidade rotação (2*pi/360/VALOR_PARAMETRO)] [altitude coleta minima] [altitude coleta máxima]
    float alt_min, alt_max, vel;
    fscanf(fp, "%f %f %f", &vel, &alt_min, &alt_max);
    fclose(fp);
    // Variável que indica que os parâmetros estão OK.
    bool parameters_ok = (alt_min >= 1 && alt_max <= 50 && vel > 0 && vel <= 10);
    sleep(10);
    fp2 = fopen("/home/pi/status.txt", "w");

    // Verifica se os dados dos sensores são válidos
    if(parameters_ok && (hdc1050.valid == 1) && (ms4525.valid == 1) && current_state.connected)
    {
      //  digitalWrite(PIN, HIGH); // Acende o LED caso sim
	fprintf(fp2, "Status: OK");
    }
    else
    {
      //  digitalWrite(PIN, LOW); // Garantia
	fprintf(fp2, "Status: NOT OK");
    }

    fclose(fp);
    fclose(fp2);

    return 0;
}
