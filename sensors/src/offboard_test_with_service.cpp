// gravar indic e true airspeed, humidade, lat, long, timestamp  
// fazer checagem dos sensores, do arquivo dentro do nó master, acender led = ok, piscar led = !ok
// (ok) mudar para 10s a rotação 
// quaternion para determinar parada, quando bater 2pi + offset
// arrumar arquivo = segfault
// tratar excessao do arquivo para n crashar ros
// gravar o 0 do sensor de airspeed (true, indicated) para fazer compensação (tapar tubo quando ligar o sistema)

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

#include <std_msgs/Float64.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <string.h>

#include <pthread.h>

#include <math.h>

#define FILENAME "/tmp/data.txt"

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped pos;
void get_pos(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pos = *msg;
}

mavros_msgs::WaypointReached waypoint_num;
void get_waypoint(const mavros_msgs::WaypointReached::ConstPtr &msg)
{
    waypoint_num = *msg;
}

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


double offboard_enabled = -1;
int value_quaternion, collecting, cont_spin, collecting_5m, last_waypoint = -1, status = -1, can_compare_for_loop = 0;
double value_quat2 = 0.00872664625;
tf::Quaternion aux_rotate_tf;
geometry_msgs::Quaternion aux_rotate_geometry;
geometry_msgs::PoseStamped pose;
double compass_diff = 0;
double yaw_compass_start_value;
FILE *fp = NULL;


void *thread_func_set_pos(void *args)
{
    ros::NodeHandle nh;
    // Publicação do Pose
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 1);
    // Sensor de temperatura
    ros::Subscriber temperature_sub = nh.subscribe<std_msgs::Float64>
                                      ("temperature", 1, get_temperature_value);
    // Sensor de airspeed
    ros::Subscriber airspeed_sub = nh.subscribe<std_msgs::Float64>
                                   ("airspeed", 1, get_airspeed_value);
    ros::Rate rate(72.0);
    while(ros::ok())
    {
        if(collecting)
        {
            ROS_INFO("Collecting");
            value_quat2 += 0.00872664625; // 2*pi / 360 / 2
            aux_rotate_tf = tf::createQuaternionFromYaw(value_quat2); // Valor (Quaternion) para rotação
            quaternionTFToMsg(aux_rotate_tf, aux_rotate_geometry); // Conversão de tf::Quaternion para geometry_msgs::Quaternion
            pose.pose.orientation = aux_rotate_geometry; // Seta o valor da rotação para o Pose, para ser enviado para o drone
            if(fprintf != NULL) fprintf(fp, "%f;%f;%f\n", temperature.data, airspeed.data, compass.data); // gravar indic e true airspeed, humidade, lat, long, timestamp
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
}

double diff(double x, double y)
{
    return abs(x - y);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "drone_node");

    ros::NodeHandle nh;

    // Publicação do Pose
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);

    // Status do drone
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    // Armar o drone
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    // Mudar o modo do drone
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");
    // Pegar a posição do drone
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                              ("mavros/local_position/pose", 10, get_pos);
    // Acompanhar a chegada em waypoints
    ros::Subscriber waypoint_sub = nh.subscribe<mavros_msgs::WaypointReached>
                                   ("mavros/mission/reached", 10, get_waypoint);
    // Dados da bússola
    ros::Subscriber compass_sub = nh.subscribe<std_msgs::Float64>
                                  ("mavros/global_position/compass_hdg", 1, get_compass_value);


    // Setpoint publishing rate (precisa ser > 2Hz)
    ros::Rate rate(100.0); // 360/5 = 72

    // Espera conexão
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    // Atualiza os dados do pose

    pose.pose.position.x = pos.pose.position.x;
    pose.pose.position.y = pos.pose.position.y;
    pose.pose.position.z = pos.pose.position.z;

    // Inicia a thread para envio da posição para o ROS

    pthread_t thread_pose;
    pthread_create(&(thread_pose), NULL, thread_func_set_pos, NULL);

    // Drone mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode.request.base_mode = 0;

    // Armar o drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Último request
    ros::Time last_request = ros::Time::now();

    // Último waypoint -> comparar com o atual para mudança de estado
    last_waypoint = waypoint_num.wp_seq;
    ROS_INFO_STREAM("First value for waypoint_num:" << waypoint_num.wp_seq);

    while(ros::ok())
    {
        switch(status)
        {
        case -1:
            // Verifica se chegou em um waypoint comparando o último que foi registrado com o atual
            if(last_waypoint != waypoint_num.wp_seq)
            {
                last_waypoint = waypoint_num.wp_seq;
                status = 0;
                ROS_INFO_STREAM("Waypoint reached:" << waypoint_num.wp_seq);
                pose.pose.position.x = pos.pose.position.x;
                pose.pose.position.y = pos.pose.position.y;
                fp = fopen(FILENAME, "a+");
                if(fp != NULL) fprintf(fp, "Waypoint: %d\n", last_waypoint);
            }
            break;
        case 0:
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if(current_state.mode != "OFFBOARD" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) // Se não estiver no OFFBOARD -> tente colocar
            {
                ROS_INFO("Trying to switch to Offboard mode");
            }
            if(current_state.mode == "OFFBOARD" && offboard_enabled == -1)
            {
                ROS_INFO("Offboard enabled");
                offboard_enabled = ros::Time::now().toSec(); // Horário em que o modo offboard foi habilitado
                pose.pose.position.z = 2; // Muda o valor da posição enviada para 2 metros (altura) para coleta de dados
                collecting = 0; // Garantir o valor correto para variável
            }

            if(current_state.mode == "OFFBOARD")
            {

                // Se chegou a 2m de altura (tolerância = 0.3m), começa a coletar os dados
                if(!collecting_5m && pos.pose.position.z - 2 < 0.3 && pos.pose.position.z - 2 > -0.3 && !collecting && current_state.mode == "OFFBOARD")
                {
                    ROS_INFO("Chegou 2m");
                    value_quat2 = 0; // Zera o valor do quaternion utilizado na thread
                    yaw_compass_start_value = compass.data; // Posição de início de coleta de dados
                    value_quaternion = 0; // Valor para ser usado para calcular o Quaternion
                    cont_spin = 0; // Quantidade de vezes que passou pelo mesmo ponto
                    collecting_5m = 0; // Garantir o valor correto para variável
                    can_compare_for_loop = 0; // Variável que permite saber quando a comparação para conhecimento da volta pode ser realizada
                    collecting = 1; // Coletando = SIM
                }

                ROS_INFO("Compass: %f Diff: %f", compass.data, compass.data - compass_diff);
                compass_diff = compass.data;



                // Compara a posição (bússola) atual com a posição de início caso o drone já tenha dado meia volta
                if(collecting && can_compare_for_loop && diff(yaw_compass_start_value, compass.data) < 3)
                {
                    collecting = 0; // Para de coletar
                    if(collecting_5m)   // Se coletou dados nos 5m de altura
                    {
                        status = 1; // Finaliza a coleta de dados
                        collecting_5m = 0;
                        fclose(fp);
                        fp = NULL;
                    }
                    if(!collecting_5m && status != 1)
                    {
                        pose.pose.position.z = 5; // Enviar o drone para 5m de altura
                        collecting_5m = 1; // Flag para saber se está na coleta de dados na altura de 5m ou 2m
                        can_compare_for_loop = 0;
                    }
                }

                // Se chegou a 5m de altura (tolerância = 0.3m), começa a coletar os dados
                if(collecting_5m && (pos.pose.position.z - 5 < 0.3 && pos.pose.position.z - 5 > -0.3))
                {
                    if(!collecting)
                    {
                        yaw_compass_start_value = compass.data;
                        collecting = 1; // Inicia a coleta de dados (5m de altura)
                    }
                }

                // Verifica se o drone já fez meia volta, para saber se pode começar a comparar o valor atual com o valor de início da rotação
                if(collecting && diff(yaw_compass_start_value, fmod(compass.data + 180.0, 360.0)) < 10.0)
                {
                    can_compare_for_loop = 1;
                    ROS_INFO("Now can compare for loop");

                }

                // collecting agora é utilizado pela thread_func_set_pos

                /*
                if(collecting)
                {
                    ROS_INFO("Collecting");
                    value_quat2 += 0.01745329251; // 2*pi / 360
                    aux_rotate_tf = tf::createQuaternionFromYaw(value_quat2); // Valor (Quaternion) para rotação
                    quaternionTFToMsg(aux_rotate_tf, aux_rotate_geometry); // Conversão de tf::Quaternion para geometry_msgs::Quaternion
                    pose.pose.orientation = aux_rotate_geometry; // Seta o valor da rotação para o Pose, para ser enviado para o drone
                }
                */
                /*
                if(offboard_enabled != -1 && ros::Time::now().toSec() - offboard_enabled > 10)
                {
                    status = 1;
                }
                */

            }
            break;
        case 1:
            // Seta o modo de voo como AUTO.MISSION (seguir a missão carregada)
            if (current_state.mode != "AUTO.MISSION")
            {
                offb_set_mode.request.custom_mode = "AUTO.MISSION";
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Collect data: ok");
                    ROS_INFO("AUTO.PILOT = MISSION enabled");
                    status = -1;
                }
                offboard_enabled = -1;
            }
            break;
        default:
            break;

        }

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
