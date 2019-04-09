//
// Created by satheesh on 05.04.19.
//

#include "ros/ros.h"
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>


#include <rc/motor.h>

#include <nav_msgs/Odometry.h>
//#include <ddrob/WheelVelocity.h>

//global variables

ros::Time msg_received;

bool g_driving = 0;
int g_left_motor;      // param default 1
int g_right_motor;     // param default 2
double g_maxspeed;     // param default 0.4
double g_minspeed;     // param default 0.1
double g_duty_factor;  // param default 2.0
int g_rate;


void wheel_velCallback(const std_msgs::Float64MultiArrayConstPtr& vel_Array)
{
    msg_received = ros::Time::now();

    double wheel_vel[90];
    int i = 0;
    for(std::vector<double>::const_iterator it = vel_Array->data.begin();it<=vel_Array->data.end();it++)
    {
        wheel_vel[i] = *it;
        i++;
    }

    ROS_INFO("Received wheel_Vel: [%f %f]", wheel_vel[0], wheel_vel[1]);

    double velocity_left = wheel_vel[0];
    double velocity_right = wheel_vel[1];

    // calculate duty cycle form velocity and duty factor
    double duty_left = g_duty_factor * velocity_left;
    double duty_right = g_duty_factor * velocity_right;

    ROS_INFO("set LEFT motor: velocity:%f duty:%f RIGHT motor: velocity:%f duty:%f", velocity_left, duty_left,
             velocity_right, duty_right);

    rc_motor_set(g_left_motor, duty_left);
    rc_motor_set(g_right_motor, duty_right);
    g_driving = 1;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_motor_driver");

    ros::NodeHandle n;

    ROS_INFO("Initializing node %s in the namespace %s", ros::this_node::getName().c_str(),
            ros::this_node::getNamespace().c_str());

    msg_received = ros::Time::now();

    int sub_timeout;

    ros::param::param("~timeout", sub_timeout, 5);
    ros::param::param("~left_motor", g_left_motor, 1);
    ros::param::param("~right_motor", g_right_motor, 2);
    ros::param::param("~maxspeed", g_maxspeed, 0.4);
    ros::param::param("~minspeed", g_minspeed, 0.1);
    ros::param::param("~duty_factor", g_duty_factor, 1.0);
    ros::param::param("~rate", g_rate, 10);

    int pwm_freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;  // 25000

    if (rc_motor_init_freq(pwm_freq_hz))
    {
        ROS_ERROR("Initialize motor %d and %d with %d: FAILED", g_left_motor, g_right_motor, pwm_freq_hz);
        return -1;
    }

    ROS_INFO("Initialize motor %d and %d with %d: OK", g_left_motor, g_right_motor, pwm_freq_hz);


    ros::Subscriber sub = n.subscribe("wheel_vel",100,wheel_velCallback);

    ROS_INFO("Node is up and Subsciber started");

    ros::Rate r(g_rate);


    while (ros::ok())
    {
        ros::spinOnce();

        //Motor is stopped when no message is received within the timeout value
	ROS_INFO ("%d                  %d",ros::Time::now().toSec(),msg_received.toSec());

        if (g_driving && (ros::Time::now().toSec() - msg_received.toSec()) > sub_timeout)
        {
            ROS_INFO("TIMEOUT: No wheel_vel received: Setting the motor to 0");
            rc_motor_set(g_left_motor,0);
            rc_motor_set(g_right_motor,0);

            g_driving = 0;
        }

        r.sleep();
    }

    // closing motor hardware
    ROS_INFO("Calling rc_motor_cleanup function");
    rc_motor_cleanup();
    return 0;

}
