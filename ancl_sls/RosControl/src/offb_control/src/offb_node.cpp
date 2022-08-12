/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */



/* Need to do:
1. Achieve other state from mavros.
2. Translate DEA->C++ ( or cascaded PID?)
3. Normolization about Thrust  & Torque. [-1,1]
*/

/*
2020, 8.11 Note: the linear velocity should be inertia frame ; angular velocity should be body frame.
be careful the sequency of roll yaw pitch, they may matter.
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
// add by Shiyu
#include <gazebo_msgs/LinkStates.h>  // include this head file
#include "std_msgs/String.h"
#include <std_msgs/Float64.h> //
//transfomation
#include <tf/transform_datatypes.h>

//Gazebo
using namespace std;

geometry_msgs::Pose quadpose;
geometry_msgs::Pose loadpose;
geometry_msgs::Pose pendpose;
//string name ="anymal::base";
//
struct PendulumAngles {
    double alpha, beta; // roll(alpha) pitch(beta) yaw
}penangle,penangle2;

//define the all 16 state of the Slung Load System
struct sls_state {
    double x, y, z, alpha, beta, roll, pitch, yaw, vx, vy, vz, gamma_alpha, gamma_beta, omega_1, omega_2, omega_3;
    // /mavros/local_position/pose
}sls_state1;


PendulumAngles ToPenAngles(double Lx,double Ly,double Lz);

PendulumAngles ToPenAngles2(double x,double y,double z,double w);

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
//x1-x3(position) and x6-x8(r p y)
void current_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    sls_state1.x = msg -> pose.position.x;
    sls_state1.y = msg -> pose.position.y;
    sls_state1.z = msg -> pose.position.z;
    //quaternion to r p y
    double quatx = msg->pose.orientation.x;
    double quaty = msg->pose.orientation.y;
    double quatz = msg->pose.orientation.z;
    double quatw = msg->pose.orientation.w;


    // need sequency: pitch(y) roll(x) yaw(z)
    tf::Quaternion q(quatx,quaty,quatz,quatw);
    tf::Matrix3x3 m(q);
    m.getRPY(sls_state1.roll, sls_state1.pitch, sls_state1.yaw);
}

void current_velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    sls_state1.vx = msg->twist.linear.x;
    sls_state1.vy = msg->twist.linear.y;
    sls_state1.vz = msg->twist.linear.z;

    sls_state1.omega_1 = msg->twist.angular.x;
    sls_state1.omega_2 = msg->twist.angular.y;
    sls_state1.omega_3 = msg->twist.angular.z;



}

//x4-x5 from (gazebo
void gazebo_state_cb(const gazebo_msgs::LinkStates::ConstPtr& msg){
    //ROS_INFO("I heard: [%s\n]", msg->name);
    //current_state = *msg;
    // cout<< msg->name[9]<< endl;
    quadpose = msg->pose[2];
    pendpose = msg->pose[9];
    loadpose = msg->pose[10]; // 10: pose of load; 9: pose of pendulum

    double Lx = loadpose.position.x - quadpose.position.x ;
    double Ly = loadpose.position.y - quadpose.position.y ;
    double Lz = loadpose.position.z - quadpose.position.z ;
    penangle = ToPenAngles( Lx, Ly, - Lz ); // in the paper the definition of n3 are opposite to the Z axis of gazebo
    sls_state1.alpha = penangle.alpha;
    sls_state1.beta = penangle.beta;
}
//Gazebo end

//x6

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Subscriber position_state_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, current_position_cb);

    ros::Subscriber velocity_state_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/mavros/local_position/velocity_body", 10, current_velocity_cb);


    //Gazebo
    ros::Subscriber gazebo_state_sub = nh.subscribe<gazebo_msgs::LinkStates>
            ("gazebo/link_states", 10, gazebo_state_cb);
    //Gazebo end
    //ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
          //  ("mavros/setpoint_position/local", 10);
    ros::Publisher actuator_setpoint_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("/mavros/actuator_control", 1000);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //Gazebo subscribe

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1000); //need to be high enough

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 1;
    // pose.pose.position.y = 1;
    // pose.pose.position.z = 20;

    mavros_msgs::ActuatorControl actuator_setpoint;
    actuator_setpoint.group_mix = 0;
    actuator_setpoint.controls[0] = 0;
    actuator_setpoint.controls[1] = 0;
    actuator_setpoint.controls[2] = 0;
    actuator_setpoint.controls[3] = 0.8;
    actuator_setpoint.controls[4] = 0;
    actuator_setpoint.controls[5] = 0;
    actuator_setpoint.controls[6] = 0;
    actuator_setpoint.controls[7] = 0;


    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        // local_pos_pub.publish(pose);
        actuator_setpoint_pub.publish(actuator_setpoint);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // local_pos_pub.publish(pose);
        //actuator_setpoint_pub.publish(actuator_setpoint);

        //cout<< pose.orientation << endl;

        cout<<"from position alpha: "<<penangle.alpha<<"    beta: "<<penangle.beta<<endl;
        //second method
        penangle2=ToPenAngles2(pendpose.orientation.x,pendpose.orientation.y,pendpose.orientation.z,pendpose.orientation.w);
        //cout<<"from orientation alpha: "<<penangle2.alpha<<"    beta: "<<penangle2.beta<<endl;

        // cout<<"load position" <<loadpose.position << endl;
        // cout<<"quad position" <<quadpose.position << endl;



        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


PendulumAngles ToPenAngles(double Lx,double Ly,double Lz) { //x=base.x
    PendulumAngles angles;
    double L = 1;

    // alpha (x-axis rotation)

    double cosa_cosb = Lz/L;
    double sina_cosb=Ly/-L;
    angles.alpha = std::atan2(sina_cosb,cosa_cosb);

    // beta (y-axis rotation)
    double sinbeta = Lx/L;
    double cosbeta = Lz/(L*std::cos(angles.alpha));
    angles.beta = std::atan2(sinbeta,cosbeta);

    return angles;
}

PendulumAngles ToPenAngles2(double x,double y,double z,double w) { //x=base.x
    PendulumAngles angles;

    // alpha (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.alpha = std::atan2(sinr_cosp, cosr_cosp);

    // beta (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles.beta = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.beta = std::asin(sinp);


    return angles;
}
