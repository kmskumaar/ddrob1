//
// Created by satheesh on 23.04.19.
//

#include "ros/ros.h"
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <rc/encoder_eqep.h>

#define FWD 1;
#define BKD -1;

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
 * Function to calculate the rotational speed in rps for individual wheels and direction of rotation.
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

	ros::Publisher rps_pub = n.advertise <std_msgs::Float64MultiArray>("wheel_vel",100);

	ros::Rate loop_rate(g_loop_rate);	//Looping at a frequency of 1 Hz

	while(ros::ok())
	    {
		calculateRPS();

		ROS_INFO("Left Wheel RPS: %f,     Right Wheel RPS: %f",g_wheelRPS[0],g_wheelRPS[1]);
	        std_msgs::Float64MultiArray rps;

	        rps.data.clear();

	        for(int i=0;i<(sizeof(g_wheelRPS)/sizeof(*g_wheelRPS));i++)
	        {
	            rps.data.push_back(g_wheelRPS[i]);
	        }

	        rps_pub.publish(rps);
	        ros::spinOnce();
	        loop_rate.sleep();
	    }

	return 0;

}
