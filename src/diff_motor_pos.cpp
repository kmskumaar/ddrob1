#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <rc/encoder_eqep.h>

//global variables
int g_left_encoder;      // param default 1
int g_right_encoder;     // param default 2
double g_loop_rate;

int main (int argc, char **argv){

    ros::init(argc,argv,"diff_motor_pos");
    ros::NodeHandle n;

    ros::param::param("~left_encoder", g_left_encoder, 1);
    ros::param::param("~right_encoder", g_right_encoder, 2);
    ros::param::param("~encoder_loop_rate",g_loop_rate, 100.0);

    if (rc_encoder_eqep_init())
    {
        ROS_ERROR("Initialize motor encoders %d and %d: FAILED", g_left_encoder, g_right_encoder);
        return -1;
    }

    ROS_INFO("Initialized motor encoders %d and %d: OK", g_left_encoder, g_right_encoder);

    ros::Publisher left_wheel_pos_pub = n.advertise <std_msgs::Float64>("ddrob/left_wheel_pos",100);
    ros::Publisher right_wheel_pos_pub = n.advertise <std_msgs::Float64>("ddrob/right_wheel_pos",100);

    ros::Rate loop_rate(g_loop_rate);	//Looping at a frequency of 1 Hz

    while(ros::ok())
    {
        ROS_INFO("Left Wheel Pos: %f,     Right Wheel Pos: %f",g_wheelRPS[0]*RPS_TO_RAD,g_wheelRPS[1]*RPS_TO_RAD);

        left_wheel_pos_pub.publish(rc_encoder_eqep_read(g_left_encoder));
        right_wheel_pos_pub.publish(rc_encoder_eqep_read(g_right_encoder));

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}