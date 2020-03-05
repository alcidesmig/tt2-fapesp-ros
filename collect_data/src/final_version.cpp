// to do: serviço para calcular o true airspeed

#include <ros/ros.h>

#include <string>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/WaypointReached.h>
#include <mavros_msgs/PositionTarget.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/Range.h>

#include <std_msgs/Float64.h>

#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <stdlib.h>
#include <string.h>

#include <pthread.h>

#include <math.h>

#include <time.h>

#include <collect_data/HDC1050.h>
#include <collect_data/MS4525.h>

// Includes para o servidor HTTP
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <err.h>

#define FILENAME "/mnt/pendrive/data.txt"

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

sensor_msgs::TimeReference time_reference;
void get_time_reference(const sensor_msgs::TimeReference::ConstPtr &msg)
{
    time_reference = *msg;
}

sensor_msgs::Range lidar;
void get_lidar_data(const sensor_msgs::Range::ConstPtr &msg)
{
    lidar = *msg;
}

std_msgs::Float64 rel_alt;
void get_rel_alt(const std_msgs::Float64::ConstPtr &msg)
{
    rel_alt = *msg;
}

double offboard_enabled = -1;
int value_quaternion, collecting, cont_spin, collecting_2_point = 0, collecting_3_point = 0, last_waypoint = -1, status = -1, can_compare_for_loop = 0;
double value_quat2 = 0.0;
double const_sum_quat = 0.01745329251; // 2*pi / 360
tf::Quaternion aux_rotate_tf;
geometry_msgs::Quaternion aux_rotate_geometry;
geometry_msgs::PoseStamped pose;
double compass_diff = 0;
double yaw_compass_start_value;
float alt_1_point = 2, alt_2_point = 5, alt_3_point = 10;
FILE *fp = NULL;
int alt_point = -1;
char data[3][256];
double max_airspeed[3] = {-1, -1, -1};
int change_z = 0;
float pos_x = 0, pos_y = 0, pos_z = 0;
int lidar_ok = 0, lidar_sincronized = 0;
float offset_lidar_barometer = 0;
int try_sync_done = 0;
char data_file_name[128];
char file_name_regina[128];
double altitude = 0;
double zero_i_airspeed = -1.0, zero_t_airspeed = -1.0;
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

// https://stackoverflow.com/questions/3756323/how-to-get-the-current-time-in-milliseconds-from-c-in-linux
char *get_complete_timestamp()
{
    long            ms; // Milliseconds
    time_t          s;  // Seconds
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s  = spec.tv_sec;
    ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
    if (ms > 999)
    {
        s++;
        ms = 0;
    }


    char *string = (char *) malloc(sizeof(char) * 50);
    sprintf(string, "%jd.%03ld", (intmax_t)s, ms);
    return string;
}

char * response;

// Créditos: https://rosettacode.org/wiki/Hello_world/Web_server#C.2B.2B
void *thread_func_server_http(void *args) {
  int one = 1, client_fd;
  struct sockaddr_in svr_addr, cli_addr;
  socklen_t sin_len = sizeof(cli_addr);
  ROS_INFO("Starting server");
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    err(1, "Erro ao abrir o socket");
    ROS_INFO("Socket");
  }
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(int));
 
  int port = 8080;
  svr_addr.sin_family = AF_INET;
  svr_addr.sin_addr.s_addr = INADDR_ANY;
  svr_addr.sin_port = htons(port);
 
  if (bind(sock, (struct sockaddr *) &svr_addr, sizeof(svr_addr)) == -1) {
    close(sock);
    err(1, "Erro, falha no bind");
    ROS_INFO("Bind");
  }
 
  listen(sock, 5);
  while (1) {
    client_fd = accept(sock, (struct sockaddr *) &cli_addr, &sin_len);
 	
    if(client_fd == 1) ROS_INFO("Accepted");

    if (client_fd == -1) {
      perror("Can't accept");
      ROS_INFO("Cant accept");
      continue;
    }
 
    write(client_fd, response, sizeof(response) - 1); /*-1:'\0'*/
    close(client_fd);
  }
	
}

std::string parse_for_json(char * data_1, char * data_2, char * data_3) {
	FILE * fp_id = fopen("/home/pi/id_coleta", "r");
	int id_coleta;
        fscanf(fp_id, "%d", &id_coleta);
	fclose(fp_id);
	fp_id = fopen("/home/pi/id_coleta", "w+");
	fprintf(fp_id, "%d", id_coleta + 1);
	fclose(fp_id);

	//  hdc1050.temperature, indicated_airspeed, indicated_airspeed - zero_i_airspeed, true_airspeed, true_airspeed - zero_t_airspeed, hdc1050.humidity, fmod((compass.data + 180), 360), gps.latitude, gps.longitude, gps.altitude, rel_alt.data, lidar.range, altitude, time_reference.time_ref.sec, complete_timestamp

	std::string temp_1 = strtok(data_1, ";");
	strtok(NULL, ";");
	std::string indicated_airspeed_1 = strtok(NULL, ";");
	strtok(NULL, ";");
	std::string true_airspeed_1 = strtok(NULL, ";");
	std::string humidity_1 = strtok(NULL, ";");
	std::string compass_1 = strtok(NULL, ";");
	std::string lat_1 = strtok(NULL, ";");
	std::string long_1 = strtok(NULL, ";");
	std::string alt_1 = strtok(NULL, ";");

	std::string temp_2 = strtok(data_2, ";");
        strtok(NULL, ";");
        std::string indicated_airspeed_2 = strtok(NULL, ";");
        strtok(NULL, ";");
        std::string true_airspeed_2 = strtok(NULL, ";");
        std::string humidity_2 = strtok(NULL, ";");
        std::string compass_2 = strtok(NULL, ";");
        std::string lat_2 = strtok(NULL, ";");
        std::string long_2 = strtok(NULL, ";");
        std::string alt_2 = strtok(NULL, ";");

	std::string temp_3 = strtok(data_3, ";");
        strtok(NULL, ";");
        std::string indicated_airspeed_3 = strtok(NULL, ";");
        strtok(NULL, ";");
        std::string true_airspeed_3 = strtok(NULL, ";");
        std::string humidity_3 = strtok(NULL, ";");
        std::string compass_3 = strtok(NULL, ";");
        std::string lat_3 = strtok(NULL, ";");
        std::string long_3 = strtok(NULL, ";");
        std::string alt_3 = strtok(NULL, ";");

	char id_coleta_[20];
	sprintf(id_coleta_, "%d", id_coleta);
	std::string id_coleta_s = id_coleta_;

	std::string ret = ""
	"{" 
		"\t\"id\": \"" + id_coleta_s + "\","
		"\t\"temp_1\": \"" + temp_1 + "\""
		"\t\"indicated_airspeed_1\": \"" + indicated_airspeed_1 + "\","
		"\t\"true_airspeed_1\": \"" + true_airspeed_1 + "\","
		"\t\"humidity_1\": \"" + humidity_1 + "\","
		"\t\"compass_1\": \"" + compass_1 + "\","
		"\t\"lat_1\": \"" + lat_1 + "\","
		"\t\"long_1\": \"" + long_1 + "\","
		"\t\"alt_1\": \"" + alt_1 + "\","
		"\t\"temp_2\": \"" + temp_2 + "\","
                "\t\"indicated_airspeed_2\": \"" + indicated_airspeed_2 + "\","
                "\t\"true_airspeed_2\": \"" + true_airspeed_2 + "\","
                "\t\"humidity_2\": \"" + humidity_2 + "\","
                "\t\"compass_2\": \"" + compass_2 + "\","
                "\t\"lat_2\": \"" + lat_2 + "\","
                "\t\"long_2\": \"" + long_2 + "\","
                "\t\"alt_2\": \"" + alt_2 + "\","
		"\t\"temp_3\": \"" + temp_3 + "\","
                "\t\"indicated_airspeed_3\": \"" + indicated_airspeed_3 + "\","
                "\t\"true_airspeed_3\": \"" + true_airspeed_3 + "\","
                "\t\"humidity_3\": \"" + humidity_3 + "\","
                "\t\"compass_3\": \"" + compass_3 + "\","
                "\t\"lat_3\": \"" + lat_3 + "\","
                "\t\"long_3\": \"" + long_3 + "\","
                "\t\"alt_3\": \"" + alt_3 + "\""
	"}";
	return ret;

}

void *thread_func_set_pos(void *args)
{
    ros::NodeHandle nh;
    // Publicação do Pose
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
                                   ("mavros/setpoint_position/local", 1);
    // Pressão ambiente para o cálculo do true airspeed
    ros::Subscriber pressure_sub = nh.subscribe<sensor_msgs::FluidPressure>
                                   ("mavros/imu/static_pressure", 1, get_pressure_value);

    ros::Subscriber time_reference_sub = nh.subscribe<sensor_msgs::TimeReference>
                                         ("mavros/time_reference", 1, get_time_reference);

    ros::Rate rate_thread(72.0);


    while(ros::ok())
    {
        if(collecting)
        {
            try
            {
                value_quat2 += const_sum_quat; // 2*pi / 360 / 2
                ROS_INFO("Collecting quaternion = %f", value_quat2);
                aux_rotate_tf = tf::createQuaternionFromYaw(value_quat2); // Valor (Quaternion) para rotação
                quaternionTFToMsg(aux_rotate_tf, aux_rotate_geometry); // Conversão de tf::Quaternion para geometry_msgs::Quaternion
                pose.pose.orientation = aux_rotate_geometry; // Seta o valor da rotação para o Pose, para ser enviado para o drone
                float indicated_airspeed = ms4525.indicated_airspeed;
                float temperature_airspeed = ms4525.temperature;
                float true_airspeed = calc_true_airspeed_from_indicated(indicated_airspeed, pressure_ambient.fluid_pressure, temperature_airspeed);

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
                        char * complete_timestamp = get_complete_timestamp();
                        fprintf(fp,
                                "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d;%s\n",
                                hdc1050.temperature, indicated_airspeed, indicated_airspeed - zero_i_airspeed, true_airspeed, true_airspeed - zero_t_airspeed, hdc1050.humidity, fmod((compass.data + 180), 360), gps.latitude, gps.longitude, gps.altitude, rel_alt.data, lidar.range, altitude, time_reference.time_ref.sec, complete_timestamp
                               );
                        if(alt_point != -1 && true_airspeed/*indicated_airspeed*/ > max_airspeed[alt_point - 1])
                        {
                            strcpy(data[alt_point - 1], "");
                            sprintf(data[alt_point - 1],
                                    "%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%d;%s\n",
                                hdc1050.temperature, indicated_airspeed, indicated_airspeed - zero_i_airspeed, true_airspeed, true_airspeed - zero_t_airspeed, hdc1050.humidity, fmod((compass.data + 180), 360), gps.latitude, gps.longitude, gps.altitude, rel_alt.data, lidar.range, altitude, time_reference.time_ref.sec, complete_timestamp
                               );
                            max_airspeed[alt_point - 1] = /*indicated_airspeed;*/true_airspeed;
                        }
                        free(complete_timestamp);
                    }
                }
            }
            catch(...)
            {
                ROS_INFO("ERROR - COLLECTING");
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate_thread.sleep();
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

    ros::Rate rate(25.0);

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
                              ("mavros/local_position/pose", 1, get_pos);
    // Acompanhar a chegada em waypoints
    ros::Subscriber waypoint_sub = nh.subscribe<mavros_msgs::WaypointReached>
                                   ("mavros/mission/reached", 10, get_waypoint);
    // Dados da bússola
    ros::Subscriber compass_sub = nh.subscribe<std_msgs::Float64>
                                  ("mavros/global_position/compass_hdg", 1, get_compass_value);
    // Dados de GPS
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>
                              ("mavros/global_position/raw/fix", 1, get_gps_value);
    // Sensor de temperatura
    ros::Subscriber hdc1050_sub = nh.subscribe<collect_data::HDC1050>
                                  ("hdc1050", 1, get_hdc1050_data);
    // Sensor de airspeed
    ros::Subscriber ms4525_sub = nh.subscribe<collect_data::MS4525>
                                 ("ms4525", 1, get_ms4525_data);
    // Sensor lidar: altitude
    ros::Subscriber range_lidar_sub = nh.subscribe<sensor_msgs::Range>
                                      ("mavros/distance_sensor/lidarlite_pub", 1, get_lidar_data);
    // Altitude pelo sensor barométrico
    ros::Subscriber rel_alt_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1, get_rel_alt);

    // Mensagem que controla a velocidade do drone (x, y, z) ~ (frente, lado, cima)
    ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);


    // Abre arquivo e lê o valor da velocidade da rotação e grava na constante de soma
    FILE *file_rotate = fopen("/home/pi/parameters.txt", "r");
    int return_scanf = fscanf(file_rotate, "%f %f %f", &alt_1_point, &alt_2_point, &alt_3_point);
    fclose(file_rotate);
    ROS_INFO_STREAM("Return scanf e parameters" << return_scanf << alt_1_point << alt_2_point << alt_3_point);
    if(return_scanf != 3)
    {
        alt_1_point = 2;
        alt_2_point = 5;
        alt_3_point = 10;
    }

/*
    // Espera conexão
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
*/
    mavros_msgs::PositionTarget pos_target;
    pos_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

    pos_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                           mavros_msgs::PositionTarget::IGNORE_PZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    /*  pos_target.position.x = 0.0f;
      pos_target.position.y = 0.0f;
      pos_target.position.z = 50.0f;
      pos_target.acceleration_or_force.x = 0.0f;
      pos_target.acceleration_or_force.y = 0.0f;
      pos_target.acceleration_or_force.z = 0.0f;
    */
    pos_target.velocity.x = 0.0f;
    pos_target.velocity.y = 0.0f;
    pos_target.velocity.z = -0.5;

    // Atualiza os dados do pose
    pose.pose.position.x = pos.pose.position.x;
    pose.pose.position.y = pos.pose.position.y;
    pose.pose.position.z = pos.pose.position.z;

    // Inicia a thread para envio da posição para o ROS

    pthread_t thread_pose[2];
    pthread_create(&(thread_pose[0]), NULL, thread_func_set_pos, NULL);
    pthread_create(&(thread_pose[1]), NULL, thread_func_server_http, NULL);	

    // Drone mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offb_set_mode.request.base_mode = 0;

    // Armar o drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Último request
    ros::Time last_request = ros::Time::now();


    while(time_reference.time_ref.sec == NULL or time_reference.time_ref.sec == 0) sleep(1);

    sprintf(data_file_name, "/mnt/pendrive/data_%d.txt", time_reference.time_ref.sec);
    sprintf(file_name_regina, "/mnt/pendrive/regina_%d.txt", time_reference.time_ref.sec);



    // Gravar o 0 do sensor de airspeed
    fp = fopen(data_file_name, "a+");
 //   while(fp == NULL && !ms4525.valid); // Possível erro no while // Espera chegar algum dado do sensor de ms4525 para gravar o 0 do sensor de airspeed.
    sleep(5); // Para dados consistentes do sensor
    zero_i_airspeed = ms4525.indicated_airspeed;
    zero_t_airspeed = calc_true_airspeed_from_indicated(ms4525.indicated_airspeed, pressure_ambient.fluid_pressure, ms4525.temperature);
    fprintf(fp, "Zero do sensor de airspeed: indicado(%f) true(%f) - Dados válidos: %d\n\n", zero_i_airspeed, zero_t_airspeed, ms4525.valid);
    fclose(fp);
    fp = NULL;

    fp = fopen(file_name_regina, "a+");
    fprintf(fp, "temperatura;raw indicated airspeed;indicated airspeed (-offset);raw true airspeedtrue airspeed (-offset);umidade;bússola;latitude;longitude;altitude gps;rel_alt;lidar;altitude calculada;timestamp;timestamp_nsecs\n");
    fclose(fp);

    response = (char *) malloc(sizeof(strlen("HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n\r\n")) + 1);
    strcpy(response, "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n\r\n");

    // Último waypoint -> comparar com o atual para mudança de estado
    last_waypoint = waypoint_num.wp_seq;
    ROS_INFO_STREAM("First value for waypoint_num:" << waypoint_num.wp_seq);

    while(ros::ok())
    {
        // int valid_lidar = lidar.range >= 0.1 && lidar.range <= 10;

        switch(status)
        {
        case -1:
            // Verifica se chegou em um waypoint comparando o último que foi registrado com o atual. Obs: o primeiro é descartado
            if(last_waypoint != waypoint_num.wp_seq && waypoint_num.wp_seq != 0)
            {
                last_waypoint = waypoint_num.wp_seq;
                status = 0;
                ROS_INFO_STREAM("Waypoint reached:" << waypoint_num.wp_seq);
                pose.pose.position.x = pos.pose.position.x;
                pose.pose.position.y = pos.pose.position.y;
                fp = fopen(data_file_name, "a+");
                float latitude = gps.latitude;
                float longitude = gps.longitude;

                if(fp != NULL) fprintf(fp, "Waypoint: %d at Latitude: %f Longitude: %f\ntemperatura;raw indicated airspeed;indicated airspeed (-offset);raw true airspeedtrue airspeed (-offset);umidade;bússola;latitude;longitude;altitude gps;rel_alt;lidar;altitude calculada;timestamp;timestamp_nsecs\n",
                                           last_waypoint, latitude, longitude);
            }
            break;
        case 0:
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if(current_state.mode != "OFFBOARD" && set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) // Se não estiver no OFFBOARD -> tente colocar
            {
                ROS_INFO("Trying to switch to Offboard mode");
            }
            if(current_state.mode == "OFFBOARD" && !try_sync_done)
            {
                ROS_INFO("Offboard enabled Lidar: %d", lidar_ok);
                offboard_enabled = ros::Time::now().toSec(); // Horário em que o modo offboard foi habilitado
                // if(valid_lidar) pose.pose.position.z = pos.pose.position.z - lidar.range + alt_1_point; // Muda o valor da posição enviada para alt_1_point metros (altura) para coleta de dados
                // else pose.pose.position.z = pos.pose.position.z - rel_alt.data + alt_1_point;
                // pose.pose.position.z = alt_1_point; // Muda o valor da posição enviada para alt_1_point metros (altura) para coleta de dados
                collecting = 0; // Garantir o valor correto para variável

                if(lidar_ok && !lidar_sincronized) // Se o lidar estiver ok (já tiver registrado algum valor válido) e a altura não foi sincronizada (offset calculado), faz a sincronização
                {

                    if(pos.pose.position.z > 7 && (lidar.range < 1 || lidar.range > 7)) // Manda o drone para baixo até que atinja uma altura na qual o lidar é valido
                    {
                        pos_x = pos.pose.position.x;
                        pos_y = pos.pose.position.y;
                        pos_target.velocity.z = -0.5 * 2;
                        change_z = 1;
                    }
                    if(lidar.range > 3 && lidar.range < 7) // Se entrou num range válido do lidar, faz o cálculo do offset e manda o drone para a primeira altura de coleta
                    {
                        offset_lidar_barometer = rel_alt.data - lidar.range;
                        lidar_sincronized = 1;
                        if(rel_alt.data - offset_lidar_barometer > alt_1_point) // Manda o drone para cima ou para baixo, a depender da posição relativa em relação à altura de coleta
                        {
                            pos_target.velocity.z = -0.5 * 2;
                        }
                        else
                        {
                            pos_target.velocity.z = 0.5 * 2;
                        }
                        pos_x = pos.pose.position.x;
                        pos_y = pos.pose.position.y;
                        change_z = 1;
                        try_sync_done = 1;
                    }
                }
                if(!lidar_ok)   // Se o lidar não estiver ok, não faz a sincronização
                {
                    try_sync_done = 1;
                }
            }



            if(current_state.mode == "OFFBOARD" && try_sync_done)
            {


                if(!lidar_ok) altitude = rel_alt.data; // Calcula a altitude com base nos dados do lidar
                else altitude = rel_alt.data - offset_lidar_barometer;

                ROS_INFO("Lidar ok: %d, altitude: %f", lidar_ok, altitude);
                // Se chegou a alt_1_point metros de altura (tolerância = 0.3m), começa a coletar os dados
                if(!collecting_2_point && !collecting_3_point && altitude - alt_1_point < 0.3 && altitude - alt_1_point > -0.3 && !collecting && current_state.mode == "OFFBOARD")
                {
                    change_z = 0; // Variável que ativa a publicação da velocidade do eixo z
                    value_quat2 = 0; // Zera o valor do quaternion utilizado na thread
                    yaw_compass_start_value = compass.data; // Posição de início de coleta de dados
                    value_quaternion = 0; // Valor para ser usado para calcular o Quaternion
                    cont_spin = 0; // Quantidade de vezes que passou pelo mesmo ponto
                    collecting_2_point = 0; // Garantir o valor correto para variável
                    collecting_3_point = 0; // Garantir o valor correto para variável
                    can_compare_for_loop = 0; // Variável que permite saber quando a comparação para conhecimento da volta pode ser realizada
                    collecting = 1; // Coletando = SIM
                    alt_point = 1;
                    pos_x = pos.pose.position.x; // Valores de sincronização para evitar ruidos na rotação
                    pos_y = pos.pose.position.y;
                    pos_z = pos.pose.position.z;
                }

                //  ROS_INFO("Compass: %f Diff: %f", compass.data, compass.data - compass_diff);
                compass_diff = compass.data;

                // Compara a posição (bússola) atual com a posição de início caso o drone já tenha dado meia volta
                if(collecting && ((can_compare_for_loop && diff(yaw_compass_start_value, compass.data) < 3) || value_quat2 > 6.28318530718) )
                {
                    ROS_INFO_STREAM("Can compare for loop" << can_compare_for_loop);
                    ROS_INFO_STREAM("Yaw Diff" << diff(yaw_compass_start_value, compass.data));
                    ROS_INFO_STREAM("Value quat" << value_quat2);

                    collecting = 0; // Para de coletar
                    if(collecting_3_point) // Se coletou dados no 3 ponto de coleta
                    {
                        status = 1; // Finaliza a coleta de dados
                        collecting_3_point = 0;
                        fclose(fp);
                        ROS_INFO("Finalizou coleta");
                        fp = NULL;
                        alt_point = -1;
                        FILE *regina = fopen(file_name_regina, "a");
                        fprintf(regina, "Waypoint %d\n", last_waypoint);
                        fprintf(regina, "%s\n", data[0]);
                        fprintf(regina, "%s\n", data[1]);
                        fprintf(regina, "%s\n", data[2]);
                        fclose(regina);
                        
			response = (char*) realloc(response, 2048);
			
			char aux_[2048] = "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=UTF-8\r\n\r\n";
			strcat(aux_, parse_for_json(data[0], data[1], data[2]).c_str());
			
			strcpy(response, aux_);

			max_airspeed[0] = -1;
                        max_airspeed[1] = -1;
                        max_airspeed[2] = -1;
                    }
                    else if(collecting_2_point) // Se coletou dados no 2 ponto de coleta
                    {
                        collecting_2_point = 0; // Flag para saber em que ponto está na coleta de dados
                        can_compare_for_loop = 0;
                        value_quat2 = 0;
                        alt_point = 3;
                        collecting_3_point = 1;
                        // if(valid_lidar) pose.pose.position.z = pos.pose.position.z - lidar.range + alt_3_point; // Muda o valor da posição enviada para alt_3_point metros (altura) para coleta de dados
                        // else pose.pose.position.z = pos.pose.position.z - rel_alt.data + alt_3_point;

                        if(altitude > alt_3_point) // Manda o drone para cima ou para baixo, a depender da posição relativa em relação à altura de coleta
                        {
                            pos_target.velocity.z = -0.5 * 2;
                        }
                        else
                        {
                            pos_target.velocity.z = 0.5 * 2;
                        }
                        pos_x = pos.pose.position.x; // Sincronização dos valores x e y para não haver conflito na publicação
                        pos_y = pos.pose.position.y;
                        change_z = 1; // Variável que ativa a publicação da velocidade do eixo z

                        ROS_INFO("Vai p 3 ponto");
                    }
                    else if(!collecting_2_point && !collecting_3_point && status != 1)
                    {
                        collecting_2_point = 1; // Flag para saber em que ponto está na coleta de dados
                        // if(valid_lidar) pose.pose.position.z = pos.pose.position.z - lidar.range + alt_2_point; // Muda o valor da posição enviada para alt_2_point metros (altura) para coleta de dados
                        // else pose.pose.position.z = pos.pose.position.z - rel_alt.data + alt_2_point;
                        if(altitude > alt_2_point)
                        {
                            pos_target.velocity.z = -0.5 * 2;
                        }
                        else
                        {
                            pos_target.velocity.z = 0.5 * 2;
                        }
                        pos_x = pos.pose.position.x; // Sincronização dos valores x e y para não haver conflito na publicação
                        pos_y = pos.pose.position.y;
                        change_z = 1; // Variável que ativa a publicação da velocidade do eixo z

                        can_compare_for_loop = 0;
                        alt_point = 2;
                        value_quat2 = 0;
                        ROS_INFO("Vai p 2 ponto");
                    }
                }

                // Se chegou a 5m de altura (tolerância = 0.3m), começa a coletar os dados
                if(collecting_2_point && (altitude - alt_2_point < 0.3 && altitude - alt_2_point > -0.3))
                {
                    if(!collecting)
                    {
                        yaw_compass_start_value = compass.data;
                        collecting = 1; // Inicia a coleta de dados no segundo ponto
                        pos_x = pos.pose.position.x; // Valores de sincronização para evitar ruidos na rotação
                        pos_y = pos.pose.position.y;
                        pos_z = pos.pose.position.z;
                    }
                    change_z = 0; // Variável que ativa a publicação da velocidade do eixo z
                }

                // Se chegou a 5m de altura (tolerância = 0.3m), começa a coletar os dados
                if(collecting_3_point && (altitude - alt_3_point < 0.3 && altitude - alt_3_point > -0.3))
                {
                    if(!collecting)
                    {
                        yaw_compass_start_value = compass.data;
                        collecting = 1; // Inicia a coleta de dados no terceiro ponto
                        pos_x = pos.pose.position.x; // Valores de sincronização para evitar ruidos na rotação
                        pos_y = pos.pose.position.y;
                        pos_z = pos.pose.position.z;
                    }
                    change_z = 0; // Variável que ativa a publicação da velocidade do eixo z
                }

                // Verifica se o drone já fez meia volta, para saber se pode começar a comparar o valor atual com o valor de início da rotação
                if(collecting && diff(yaw_compass_start_value, fmod(compass.data + 180.0, 360.0)) < 10.0)
                {
                    can_compare_for_loop = 1;
                    ROS_INFO("Now can compare for loop");
                    yaw_compass_start_value = -500;
                }

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
                offboard_enabled = -1; // Zera as variáveis para a proxima entrada no estado
                lidar_sincronized = 0;
                try_sync_done = 0;
            }
            break;
        default:
            break;

        }

        if(change_z) // Caso seja para publicar o pos_target
        {
            pos_target.position.x = pos_x; // Sincroniza os valores de x e y
            pos_target.position.y = pos_y;
            set_vel_pub.publish(pos_target); // Publica a mensagem
            pose.pose.position.x = pos.pose.position.x; // Sincroniza os valores
            pose.pose.position.y = pos.pose.position.y;
            pose.pose.position.z = pos.pose.position.z;
        }
        else
        {
            pose.pose.position.x = pos_x; // Sincroniza os valores
            pose.pose.position.y = pos_y;
            pose.pose.position.z = pos_z;
        }

        if(!lidar_ok && lidar.range > 2 && lidar.range < 8) // Faz a verificação constante para ver se em algum momento o lidar coletou valores válidos
        {
            lidar_ok = 1;
        }


        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}

