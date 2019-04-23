//
// Created by satheesh on 23.04.19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <rc/encoder_eqep.h>
#include <bits/stdc++.h>

int g_left_encoder;      // param default 1
int g_right_encoder;     // param default 2

ros::Time old_timestamp;

double wheelRPS[2];


void calculateRPS(){

	for (int i=0;i<sizeof(wheelRPS);i++){
		wheelRPS[i]=0.0;
	}
	if (old_timestamp){
		wheelRPS[g_left_encoder-1]= rc_encoder_eqep_read(g_left_encoder)/(ros::Time::now().toSec()-old_timestamp.toSec());
		wheelRPS[g_right_encoder-1]= rc_encoder_eqep_read(g_right_encoder)/(ros::Time::now().toSec()-old_timestamp.toSec());
	}

	rc_encoder_eqep_write(g_left_encoder,0);
	rc_encoder_eqep_write(g_right_encoder,0);
	old_timestamp = ros::Time::now();
}


int main (int argc, char **argv){

	ros::init(argc,argv,"diff_motor_encoder");
	ros::NodeHandle n;

	ros::param::param("~left_encoder", g_left_encoder, 1);
	ros::param::param("~right_encoder", g_right_encoder, 2);

	if (rc_encoder_eqep_init())
	    {
	        ROS_ERROR("Initialize motor encoders %d and %d: FAILED", g_left_encoder, g_right_encoder);
	        return -1;
	    }

	ROS_INFO("Initialize motor encoders %d and %d: OK", g_left_encoder, g_right_encoder);

	ros::Publisher rps_pub = n.advertise <std_msgs::Float64MultiArray>("wheel_rps",100);

	ros::Rate loop_rate(1);

	while(ros::ok())
	    {
			calculateRPS();
	        std_msgs::Float64MultiArray rps;

	        rps.data.clear();

	        for(int i=0;i<2;i++)
	        {
	            rps.data.push_back(wheelRPS[i]);
	        }

	        rps_pub.publish(rps);
	        ros::spinOnce();
	        loop_rate.sleep();
	    }

	return 0;

}

