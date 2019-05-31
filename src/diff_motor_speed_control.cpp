//
// Created by satheesh on 26.04.19.
//

#include "ros/ros.h"
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <geometry_msgs/Twist.h>


#include <rc/motor.h>

#include <nav_msgs/Odometry.h>
//#include <ddrob/WheelVelocity.h>

const int TOTALWHEELS=2;

//global variables

ros::Time cmdVel_received;
ros::Time wheelVel_received;

ros::Time pidOld_TimeStamp;

bool g_driving = 0;
int g_left_motor;      // param default 1
int g_right_motor;     // param default 2
double g_maxspeed;     // param default 0.4
double g_minspeed;     // param default 0.1
int g_rate;
double g_cmd_vel[TOTALWHEELS];
double g_wheel_vel[TOTALWHEELS];

double g_KP_Gain;  	   // param default
double g_KD_Gain;  	   // param default
double g_KI_Gain;  	   // param default
double g_old_errorRPS[TOTALWHEELS];
double g_pidIntegral[TOTALWHEELS];



void pidController(){
    double pid_dt = ros::Time::now().toSec() - pidOld_TimeStamp.toSec();
    pidOld_TimeStamp = ros::Time::now();

    double current_errorRPS[TOTALWHEELS];
    double pid_derivative[TOTALWHEELS];
    double pid_controlInput[TOTALWHEELS];

    for (int i=0;i<TOTALWHEELS;i++){
        current_errorRPS[i] = fabs(g_cmd_vel[i]) - fabs(g_wheel_vel[i]);
        pid_derivative[i] = 0.0;
        pid_controlInput[i] = 0.0;
    }

    for (int i=0;i<TOTALWHEELS;i++){
        ROS_INFO("Loop %d of %d",i,TOTALWHEELS);
        g_pidIntegral[i] = g_pidIntegral[i] + current_errorRPS[i]*pid_dt;
        pid_derivative[i] = (current_errorRPS[i] - g_old_errorRPS[i])/pid_dt;
        //ROS_INFO("\nIntegral: %f \nError: %f \nderror: %f \nDerivative: %f \ndt: %f for wheel %d",g_pidIntegral[i],current_errorRPS[i],(current_errorRPS[i] - g_old_errorRPS[i]),pid_derivative[i],pid_dt,i);
        pid_controlInput[i] = (g_KP_Gain*current_errorRPS[i])+(g_KD_Gain*pid_derivative[i])+(g_KI_Gain*g_pidIntegral[i]);
        //ROS_INFO("\nProportional: %f     %f",g_KP_Gain,current_errorRPS[i]);

        ROS_INFO("ERROR: %f",current_errorRPS[i]);
        ROS_INFO("PROPORTIONAL PART: %f",(g_KP_Gain*current_errorRPS[i]));
        ROS_INFO("DERIVATIVE PART: %f",(g_KD_Gain*pid_derivative[i]));
        ROS_INFO("INTEGRAL PART: %f",(g_KI_Gain*g_pidIntegral[i]));

        ROS_INFO("set duty for motor: %d Control Input: %f", i, pid_controlInput[i]);

        if (pid_controlInput[i]<0.0){
            pid_controlInput[i] = 0.0;
            g_pidIntegral[i] = g_pidIntegral[i] - current_errorRPS[i]*pid_dt;
        }

        if (pid_controlInput[i] > 0.7){
            pid_controlInput[i] = 0.7;
            g_pidIntegral[i] = g_pidIntegral[i] - current_errorRPS[i]*pid_dt;
        }

        if(g_cmd_vel[i]==0){
            pid_controlInput[i] = 0.0;
        }

        if(g_cmd_vel[i]<0.0){
            pid_controlInput[i] = -pid_controlInput[i];
        }
    }

    for(int i=0;i<TOTALWHEELS;i++){
        g_old_errorRPS[i] = current_errorRPS[i];
    }



    rc_motor_set(g_left_motor, pid_controlInput[0]);
    rc_motor_set(g_right_motor, pid_controlInput[1]);
    g_driving = 1;
}


void wheel_velCallback(const std_msgs::Float64MultiArrayConstPtr& wheel_velArray){
	wheelVel_received = ros::Time::now();

	int i = 0;
	for(std::vector<double>::const_iterator it = wheel_velArray->data.begin();it<wheel_velArray->data.end();it++)
	{
        g_wheel_vel[i] = *it;
        //ROS_INFO("Loop: %d %f",it,g_wheel_vel[i]);
		i++;
	}
    ROS_INFO("==========================================================");
	ROS_INFO("Received wheel_Vel: [%f %f]", g_wheel_vel[0], g_wheel_vel[1]);
    ROS_INFO("==========================================================");
    pidController();
}


void cmd_velCallback(const std_msgs::Float64MultiArrayConstPtr& cmd_velArray)
{
	cmdVel_received = ros::Time::now();


    int i = 0;
    for(std::vector<double>::const_iterator it = cmd_velArray->data.begin();it<cmd_velArray->data.end();it++)
    {
        g_cmd_vel[i] = *it;
        i++;
    }
    //ROS_INFO("Received cmd_Vel: [%f %f]", g_cmd_vel[0], g_cmd_vel[1]);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_motor_speed_control");

    ros::NodeHandle n;

    ROS_INFO("Initializing node %s in the namespace %s", ros::this_node::getName().c_str(),
            ros::this_node::getNamespace().c_str());

    cmdVel_received = ros::Time::now();
    wheelVel_received = ros::Time::now();
    pidOld_TimeStamp = ros::Time::now();

    for (int i=0;i<TOTALWHEELS;i++){
        g_old_errorRPS[i] = 0.0;
        g_pidIntegral[i] = 0.0;
    }
    int sub_timeout;

    ros::param::param("~timeout", sub_timeout, 5);
    ros::param::param("~left_motor", g_left_motor, 1);
    ros::param::param("~right_motor", g_right_motor, 2);
    ros::param::param("~maxspeed", g_maxspeed, 0.4);
    ros::param::param("~minspeed", g_minspeed, 0.1);
    ros::param::param("~rate", g_rate, 10);
    ros::param::param("~KP_Gain", g_KP_Gain, 0.07);
    ros::param::param("~KD_Gain", g_KD_Gain, 0.0);
    ros::param::param("~KI_Gain", g_KI_Gain, 0.1);

    int pwm_freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;  // 25000

    ROS_INFO("\nPID Controller Parameters: \n  KP = %f, \nKD = %f, \nKI = %f",g_KP_Gain,g_KD_Gain,g_KI_Gain);
    if (rc_motor_init_freq(pwm_freq_hz))
    {
        ROS_ERROR("Initialize motor %d and %d with %d: FAILED", g_left_motor, g_right_motor, pwm_freq_hz);
        return -1;
    }

    ROS_INFO("Initialize motor %d and %d with %d: OK", g_left_motor, g_right_motor, pwm_freq_hz);


    ros::Subscriber sub_cmdvel = n.subscribe("cmd_vel",100,cmd_velCallback);

    ros::Subscriber sub_encoder = n.subscribe("wheel_vel",100,wheel_velCallback);


    ROS_INFO("Node is up and Subscriber started");

    ros::Rate r(g_rate);


    while (ros::ok())
    {
        ros::spinOnce();

        //Motor is stopped when no command velocity message is received within the timeout value
        if (g_driving==1 && ((ros::Time::now().toSec() - cmdVel_received.toSec()) > sub_timeout))
        {
            ROS_INFO("TIMEOUT: No cmd_vel received: Setting the motor to 0");
            rc_motor_set(g_left_motor,0);
            rc_motor_set(g_right_motor,0);

            g_driving = 0;
        }

        //Motor is stopped when no message from the encoder is received within the timeout value
		if (g_driving==1 && ((ros::Time::now().toSec() - wheelVel_received.toSec()) > sub_timeout))
		{
			ROS_INFO("TIMEOUT: No wheel_vel from encoder received: Setting the motor to 0");
			rc_motor_set(g_left_motor,0);
			rc_motor_set(g_right_motor,0);

			g_driving = 0;
		}

		if(g_driving==0){
            pidOld_TimeStamp = ros::Time::now();
            for (int i=0;i<TOTALWHEELS;i++){
                g_old_errorRPS[i] = 0.0;
                g_pidIntegral[i] = 0.0;
            }
		}
        r.sleep();
    }

    // closing motor hardware
    ROS_INFO("Calling rc_motor_cleanup function");
    rc_motor_cleanup();
    return 0;

}
