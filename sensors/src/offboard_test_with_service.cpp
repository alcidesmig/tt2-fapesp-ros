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

int status = -1;
double offboard_enabled = -1;
int last_waypoint = -1;
int value_quaternion, collecting, cont_spin, collecting_5m;
tf::Quaternion aux_rotate_tf;
geometry_msgs::Quaternion aux_rotate_geometry;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
                                ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
                                       ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
                                         ("mavros/set_mode");
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
                              ("mavros/local_position/pose", 10, get_pos);
    ros::Subscriber waypoint_sub = nh.subscribe<mavros_msgs::WaypointReached>
                                   ("mavros/mission/reached", 10, get_waypoint);
    ros::Subscriber compass_sub = nh.subscribe<std_msgs::Float64>
                                  ("mavros/global_position/compass_hdg", 10, get_compass_value);




    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);


    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pos.pose.position.x;
    pose.pose.position.y = pos.pose.position.y;
    pose.pose.position.z = pos.pose.position.z + 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode.request.base_mode = 0;
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    last_waypoint = waypoint_num.wp_seq;
    ROS_INFO_STREAM("First value for waypoint_num:" << waypoint_num.wp_seq);

    while(ros::ok())
    {
        if(last_waypoint != waypoint_num.wp_seq && status == -1) // Verifica se chegou em um waypoint comparando o último que foi registrado com o atual
        {
            last_waypoint = waypoint_num.wp_seq;
            status = 0;
            ROS_INFO_STREAM("Waypoint reached:" << waypoint_num.wp_seq);

            pose.pose.position.x = pos.pose.position.x;
            pose.pose.position.y = pos.pose.position.y;
            pose.pose.position.z = /*pos.pose.position.z + */2;
            local_pos_pub.publish(pose);

        }
        if(status == 0)
        {
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


                if(pos.pose.position.z - 2 < 0.3 && pos.pose.position.z - 2 > -0.3 && !collecting && current_state.mode == "OFFBOARD")   // Se chegou a 2m de altura (tolerância = 0.3m), começa a coletar os dados
                {
                    ROS_INFO("Chegou 2m");
                    collecting = 1; // Coletando = SIM
                    value_quaternion = 0; // Valor para ser usado para calcular o Quaternion
                    cont_spin = 0; // Quantidade de vezes que passou pelo mesmo ponto
                    collecting_5m = 0; // Garantir o valor correto para variável
                    //inicial_compass = compass.data; // Posição de início de coleta de dados
                }
                ROS_INFO("Compass: %f", compass.data);
                if(collecting && compass.data < 3)   // Verifica o ponto que está passando
                {
                    cont_spin++; // Conta a quantidade de vezes que passou por um ponto < 3
                    if(cont_spin == 2 && !collecting_5m)   // Se passou duas vezes pelo mesmo ponto, deu pelo menos uma volta coletando dados
                    {
                        collecting = 0; // Para de coletar
                        cont_spin = 0; // Zera a contagem
                        pose.pose.position.z = 5; // Enviar o drone para 5m de altura
                        collecting_5m = 1; // Flag para saber se está na coleta de dados na altura de 5m ou 2m
                    }
                    if(cont_spin == 2 && collecting_5m)   // Se coletou dados nos 5m de altura
                    {
                        status = 1; // Finaliza a coleta de dados
                        collecting = 0;
                    }
                }

                if(collecting_5m && (pos.pose.position.z - 5 < 0.3 && pos.pose.position.z - 5 > -0.3))   // Se chegou a 5m de altura (tolerância = 0.3m), começa a coletar os dados
                {
                    collecting = 1; // Inicia a coleta de dados (5m de altura)
                }

                if(collecting)
                {
                    ROS_INFO("Collecting");
                    aux_rotate_tf = tf::createQuaternionFromYaw((++value_quaternion / 25)); // Valor (Quaternion) para rotação
                    quaternionTFToMsg(aux_rotate_tf, aux_rotate_geometry); // Conversão de tf::Quaternion para geometry_msgs::Quaternion
                    pose.pose.orientation = aux_rotate_geometry; // Seta o valor da rotação para o Pose, para ser enviado para o drone
                    local_pos_pub.publish(pose);

                }

                /*
                if(offboard_enabled != -1 && ros::Time::now().toSec() - offboard_enabled > 10)
                {
                    status = 1;
                }
                */

            }
        }
        else if (status == 1 && current_state.mode != "AUTO.MISSION")
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


        if(!collecting)
        {
            for(int i = 100; ros::ok() && i > 0; --i)
            {
                local_pos_pub.publish(pose);
                //ros::spinOnce();
                rate.sleep();
            }
        }

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}

/*   //send a few setpoints before starting

       for(int i = 100; ros::ok() && i > 0; --i)
       {
           local_pos_pub.publish(pose);
           ros::spinOnce();
           rate.sleep();
       }*/