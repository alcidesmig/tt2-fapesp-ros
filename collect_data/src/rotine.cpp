// Código teste: subir e rotacionar

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

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

std_msgs::Float64 rot;
void get_cmd_vel(const std_msgs::Float64::ConstPtr &msg)
{
    rot = *msg;
}

float value_sum_quaternion = 0.01745329251;
float value_now_quaternion = 0;

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


    ros::Subscriber rotat = nh.subscribe<std_msgs::Float64>
                            ("mavros/global_position/compass_hdg", 10, get_cmd_vel);




    ros::Rate rate(72.0); // 360 / 5s = 72

    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pos.pose.position.x;
    pose.pose.position.y = pos.pose.position.y;
    pose.pose.position.z = pos.pose.position.z + 2;

    // geometry_msgs::Twist rotation;
    // rotation.linear.x = 10;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        //rotat.publish(rotation);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    double i = 0;
    int flag = 1;


    // Lê o valor da velocidade da rotação e aplica no valor a ser somado
    FILE *file_rotate = fopen("/home/pi/parameters.txt", "r");
    float divisor;
    int return_scanf = fscanf(file_rotate, "%f", &divisor);
    fclose(file_rotate);
    if (return_scanf != 1) {
        divisor = 1;
    }
    value_sum_quaternion /= divisor;

    while (ros::ok())
    {
        if ( current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if ( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if ( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if ( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }


        tf::Quaternion x = tf::createQuaternionFromYaw(value_now_quaternion);
        value_now_quaternion += value_sum_quaternion;
        geometry_msgs::Quaternion y;
        ROS_INFO("%f", rot.data);
        quaternionTFToMsg(x, y);
        if (flag)
        {
            pose.pose.orientation = y;
        }
        local_pos_pub.publish(pose);
        //rotat.publish(rotation);


        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}