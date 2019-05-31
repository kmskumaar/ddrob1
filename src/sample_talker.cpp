//
// Created by satheesh on 09.04.19.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <bits/stdc++.h>

int main (int argc, char **argv)
{
    ros::init(argc,argv,"talker");

    ros::NodeHandle n;

    double cmdspeed_left,cmdspeed_right;
    ros::param::param("~cmdspeed_left", cmdspeed_left, 0.3);
    ros::param::param("~cmdspeed_right", cmdspeed_right, 0.3);

    double wheel_vel[2]={cmdspeed_left,cmdspeed_right};

    ROS_INFO("%f    %f ",wheel_vel[0],wheel_vel[1]);

    ros::Publisher chatter_pub = n.advertise <std_msgs::Float64MultiArray>("cmd_vel",100);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        std_msgs::Float64MultiArray vel;

        vel.data.clear();

        for(int i=0;i<2;i++)
        {
            vel.data.push_back(wheel_vel[i]);
        }

        chatter_pub.publish(vel);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
