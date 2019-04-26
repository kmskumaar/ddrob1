//
// Created by satheesh on 23.04.19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <rc/encoder_eqep.h>

#define FWD 1;
#define BKD -1;
//global variables
int g_left_encoder;      // param default 1
int g_right_encoder;     // param default 2

/*
 * g_wheelRPS [4] - structure of the published topic
 * 0th & 1st index -respective left and right wheel velocity (rps),
 * 2nd & 3rd index -direction of rotation of left and right wheel respectively
 */
double g_wheelRPS[4];
int g_old_encoder[2];

ros::Time g_old_timestamp;

/*
 * Function to calculate the rotational speed in rps for individual wheels and direction of rotation.
 * The rotational speed is updated in the global variable g_wheelRPS
 */
void calculateRPS(){

	for (int i=0;i<(std::end(g_wheelRPS)-std::begin(g_wheelRPS));i++){
		g_wheelRPS[i]=0.0;
	}

	g_wheelRPS[g_left_encoder-1]= (rc_encoder_eqep_read(g_left_encoder)-g_old_encoder[g_left_encoder-1])/(ros::Time::now().toSec()-g_old_timestamp.toSec());
	g_wheelRPS[g_right_encoder-1]= (rc_encoder_eqep_read(g_right_encoder)-g_old_encoder[g_right_encoder-1])/(ros::Time::now().toSec()-g_old_timestamp.toSec());

	g_old_encoder[g_left_encoder-1] = rc_encoder_eqep_read(g_left_encoder);
	g_old_encoder[g_right_encoder-1] = rc_encoder_eqep_read(g_right_encoder);

	if(g_wheelRPS[g_left_encoder-1]<0.0){
		g_wheelRPS[g_left_encoder+1]=FWD;
		g_wheelRPS[g_left_encoder-1]=fabs(g_wheelRPS[g_left_encoder-1]);
	}
	else if (g_wheelRPS[g_left_encoder-1]>0.0){
		g_wheelRPS[g_left_encoder+1]=BKD;
	}

	if(g_wheelRPS[g_right_encoder-1]<0.0){
			g_wheelRPS[g_right_encoder+1]=BKD;
			g_wheelRPS[g_right_encoder-1]=fabs(g_wheelRPS[g_right_encoder-1]);
	}
	else if(g_wheelRPS[g_right_encoder-1]>0.0){
		g_wheelRPS[g_right_encoder+1]=FWD;
	}

	g_old_timestamp = ros::Time::now();
}


int main (int argc, char **argv){

	ros::init(argc,argv,"diff_motor_encoder");
	ros::NodeHandle n;

	ros::param::param("~left_encoder", g_left_encoder, 1);
	ros::param::param("~right_encoder", g_right_encoder, 2);

	for (int i=0;i<(std::end(g_old_encoder)-std::begin(g_old_encoder));i++){
		g_old_encoder[i]=0;
		}

	if (rc_encoder_eqep_init())
	    {
	        ROS_ERROR("Initialize motor encoders %d and %d: FAILED", g_left_encoder, g_right_encoder);
	        return -1;
	    }

	ROS_INFO("Initialize motor encoders %d and %d: OK", g_left_encoder, g_right_encoder);

	ros::Publisher rps_pub = n.advertise <std_msgs::Float64MultiArray>("wheel_rps",100);

	ros::Rate loop_rate(1);	//Looping at a frequency of 1 Hz

	while(ros::ok())
	    {
			calculateRPS();

			ROS_INFO("Left Wheel RPS: %f(%f),     Right Wheel RPS: %f(%f)",g_wheelRPS[0],g_wheelRPS[2],g_wheelRPS[1],g_wheelRPS[3]);
	        std_msgs::Float64MultiArray rps;

	        rps.data.clear();

	        for(int i=0;i<sizeof(g_wheelRPS);i++)
	        {
	            rps.data.push_back(g_wheelRPS[i]);
	        }

	        rps_pub.publish(rps);
	        ros::spinOnce();
	        loop_rate.sleep();
	    }

	return 0;

}
