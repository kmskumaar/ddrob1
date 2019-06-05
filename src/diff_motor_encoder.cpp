//
// Created by satheesh on 23.04.19.
//

#include "ros/ros.h"
#include <vector>
#include <cmath>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <rc/encoder_eqep.h>

#define RPS_TO_RAD  2*M_PI

const int RPS=1;
//global variables
int g_left_encoder;      // param default 1
int g_right_encoder;     // param default 2
int g_counts;		 // Number of encoder counts per revolution. default 12
double g_loop_rate;
/*
 * g_wheelRPS [4] - structure of the published topic
 * 0th & 1st index -respective left and right wheel velocity (rps),
 * 2nd & 3rd index -direction of rotation of left and right wheel respectively
 */
double g_wheelRPS[2];
int g_old_encoder[2];

ros::Time g_old_timestamp;

/*
 * Function to calculate the rotational speed in RPS for individual wheels and direction of rotation.
 * The rotational speed is updated in the global variable g_wheelRPS
 */
void calculateRPS(){

	for (int i=0;i<(sizeof(g_wheelRPS)/sizeof(*g_wheelRPS));i++){
		g_wheelRPS[i]=0.0;
	}
	//ROS_INFO("=======================================\nTIME DIFFERENCE: %f",ros::Time::now().toSec()-g_old_timestamp.toSec());
	//ROS_INFO("COUNT DIFFERENCE: %d",rc_encoder_eqep_read(g_left_encoder)-g_old_encoder[g_left_encoder-1]);

	g_wheelRPS[g_left_encoder-1]= -1*RPS*(rc_encoder_eqep_read(g_left_encoder)-g_old_encoder[g_left_encoder-1])/
			(g_counts*(ros::Time::now().toSec()-g_old_timestamp.toSec()));
	//ROS_INFO("LEFT WHEEL: %f \n==================================",g_wheelRPS[g_left_encoder-1]);

	g_wheelRPS[g_right_encoder-1]= RPS*(rc_encoder_eqep_read(g_right_encoder)-g_old_encoder[g_right_encoder-1])/
			(g_counts*(ros::Time::now().toSec()-g_old_timestamp.toSec()));

	g_old_encoder[g_left_encoder-1] = rc_encoder_eqep_read(g_left_encoder);
	g_old_encoder[g_right_encoder-1] = rc_encoder_eqep_read(g_right_encoder);
	g_old_timestamp = ros::Time::now();
}


int main (int argc, char **argv){

	ros::init(argc,argv,"diff_motor_encoder");
	ros::NodeHandle n;

	ros::param::param("~left_encoder", g_left_encoder, 1);
	ros::param::param("~right_encoder", g_right_encoder, 2);
	ros::param::param("~encoder_counts", g_counts, 12);
	ros::param::param("~encoder_loop_rate",g_loop_rate, 1.0);

	for (int i=0;i<(sizeof(g_old_encoder)/sizeof(*g_old_encoder));i++){
		g_old_encoder[i]=0;
		}

	if (rc_encoder_eqep_init())
	    {
	        ROS_ERROR("Initialize motor encoders %d and %d: FAILED", g_left_encoder, g_right_encoder);
	        return -1;
	    }

	ROS_INFO("Initialize motor encoders %d and %d: OK", g_left_encoder, g_right_encoder);

	ros::Publisher left_wheel_vel_pub = n.advertise <std_msgs::Float64>("ddrob/left_wheel_vel",100);
	ros::Publisher right_wheel_vel_pub = n.advertise <std_msgs::Float64>("ddrob/right_wheel_vel",100);

	ros::Rate loop_rate(g_loop_rate);	//Looping at a frequency of 1 Hz

	while(ros::ok())
	    {
		calculateRPS();
        std_msgs::Float64 left_wheel_vel, right_wheel_vel;

        ROS_INFO("Left Wheel rad/s: %f,     Right Wheel rad/s: %f",g_wheelRPS[0]*RPS_TO_RAD,g_wheelRPS[1]*RPS_TO_RAD);

        //The speed is converted from RPS to rad/s before publishing it
        left_wheel_vel.data = g_wheelRPS[g_left_encoder-1]*RPS_TO_RAD;
        right_wheel_vel.data = g_wheelRPS[g_right_encoder-1]*RPS_TO_RAD;

        left_wheel_vel_pub.publish(left_wheel_vel);
        right_wheel_vel_pub.publish(right_wheel_vel);

        ros::spinOnce();
        loop_rate.sleep();
	    }
	return 0;
}
