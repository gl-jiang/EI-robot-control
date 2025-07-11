#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "Hardware/teleop.h"
#include "App/arm_control.h"
#include "App/arm_control.cpp"
#include "App/keyboard.h"
#include "App/play.h"
#include "App/solve.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <signal.h>
#include "arm_control/PosCmd.h"
#include "arm_control/JointControl.h"
#include "arm_control/JointInformation.h"
#include "arm_control/ChassisCtrl.h"

int CONTROL_MODE=0; // 0 arx5 rc ，1 5a rc ，2 arx5 joint_control ，3 5a joint_control   4 arx5 pos_control  5 5a pos_control
command cmd;

bool app_stopped = false;
void sigint_handler(int sig);
void safe_stop(can CAN_Handlej);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_node2"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);

    arx_arm ARX_ARM((int) CONTROL_MODE);


////topic ////////////////////////////////////////////////////

                            // ros::Subscriber sub_joint = node.subscribe<arm_control::JointControl>("joint_control", 10, 
                            //                             [&ARX_ARM](const arm_control::JointControl::ConstPtr& msg) {
                            //                                 ARX_ARM.ros_control_pos_t[0] = msg->joint_pos[0];
                            //                                 ARX_ARM.ros_control_pos_t[1] = msg->joint_pos[1];
                            //                                 ARX_ARM.ros_control_pos_t[2] = msg->joint_pos[2];
                            //                                 ARX_ARM.ros_control_pos_t[3] = msg->joint_pos[3];
                            //                                 ARX_ARM.ros_control_pos_t[4] = msg->joint_pos[4];
                            //                                 ARX_ARM.ros_control_pos_t[5] = msg->joint_pos[5];
                            //                                 ARX_ARM.ros_control_pos_t[6] = msg->joint_pos[6];
                            //                                 ARX_ARM.ros_control_spd_t[0] = msg->joint_vel[0];
                            //                                 ARX_ARM.ros_control_spd_t[1] = msg->joint_vel[1];
                            //                                 ARX_ARM.ros_control_spd_t[2] = msg->joint_vel[2];
                            //                                 ARX_ARM.ros_control_spd_t[3] = msg->joint_vel[3];
                            //                                 ARX_ARM.ros_control_spd_t[4] = msg->joint_vel[4];
                            //                                 ARX_ARM.ros_control_spd_t[5] = msg->joint_vel[5];
                            //                                 ARX_ARM.ros_control_spd_t[6] = msg->joint_vel[6];


                            //                             });

                            // ros::Publisher pub_current = node.advertise<arm_control::JointInformation>("joint_information", 10);
                           
    ros::Subscriber sub_joint = node.subscribe<sensor_msgs::JointState>("/master/joint_right", 10, 
                                [&ARX_ARM](const sensor_msgs::JointState::ConstPtr& msg) 
    {   
        for(int8_t i = 0; i < 7; ++i )
        {
            ARX_ARM.ros_control_pos_t[i] = msg->position[i];
            ARX_ARM.ros_control_spd_t[i] = msg->velocity[i];
        }
        
    });    

    ros::Publisher pub_joint01 = node.advertise<sensor_msgs::JointState>("/puppet/joint_right", 10);
    ros::Publisher pub_pos = node.advertise<arm_control::PosCmd>("/puppet/pose_right", 10);
                            


////topic ////////////////////////////////////////////////////



    arx5_keyboard ARX_KEYBOARD;

    ros::Rate loop_rate(200);
    can CAN_Handlej;

    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);
    sleep(1);

    while(ros::ok())
    { 
        char key = ARX_KEYBOARD.keyPress.load();
        ARX_ARM.getKey(key);

        ARX_ARM.get_joint();
        if(!ARX_ARM.is_starting){
             cmd = ARX_ARM.get_cmd();
        }
        ARX_ARM.update_real(cmd);
    
////topic ////////////////////////////////////////////////////
                    //发送关节数据
                                // arm_control::JointInformation msg_joint;   

                                // for(int i=0;i<6;i++)
                                // {
                                //     msg_joint.joint_pos[i] = ARX_ARM.current_pos[i];
                                //     msg_joint.joint_vel[i] = ARX_ARM.current_vel[i];
                                //     msg_joint.joint_cur[i] = ARX_ARM.current_torque[i];
                                // }    

                                // msg_joint.joint_pos[6]= ARX_ARM.current_pos[6]/12;
                                // msg_joint.joint_vel[6]= ARX_ARM.current_vel[6];
                                // if(ARX_ARM.current_vel[6]<2)
                                // {
                                //     // msg_joint.joint_cur[6]=-ARX_ARM.current_torque[6];  //0.3
                                //     // msg_joint.joint_cur[6]=-ARX_ARM.Data_process(ARX_ARM.current_torque[6]); 
                                // }
                                // else
                                // {
                                //     msg_joint.joint_cur[6]=0;  //0.3
                                // }
                                // pub_current.publish(msg_joint);

        //发送末端姿态
        arm_control::PosCmd msg_pos_back;            
        msg_pos_back.x      =ARX_ARM.solve.End_Effector_Pose[0];
        msg_pos_back.y      =ARX_ARM.solve.End_Effector_Pose[1];
        msg_pos_back.z      =ARX_ARM.solve.End_Effector_Pose[2];
        msg_pos_back.roll   =ARX_ARM.solve.End_Effector_Pose[3];
        msg_pos_back.pitch  =ARX_ARM.solve.End_Effector_Pose[4];
        msg_pos_back.yaw    =ARX_ARM.solve.End_Effector_Pose[5];
        msg_pos_back.gripper=ARX_ARM.current_pos[6];

        pub_pos.publish(msg_pos_back); 

        
        sensor_msgs::JointState msg_joint01;
        msg_joint01.header.stamp = ros::Time::now();
        // msg_joint01.header.frame_id = "map";
        int8_t num_joint = 7;
        msg_joint01.name.resize(num_joint);
        msg_joint01.velocity.resize(num_joint);
        msg_joint01.position.resize(num_joint);
        msg_joint01.effort.resize(num_joint);
        
        for (int8_t i=0; i < 6; ++i)
        {   
            msg_joint01.name[i] = "joint" + std::to_string(i);
            msg_joint01.position[i] = ARX_ARM.current_pos[i];
            msg_joint01.velocity[i] = ARX_ARM.current_vel[i];
            msg_joint01.effort[i] = ARX_ARM.current_torque[i];
        }

        msg_joint01.position[6]= ARX_ARM.current_pos[6]/12;
        msg_joint01.velocity[6]= ARX_ARM.current_vel[6];
        if(ARX_ARM.current_vel[6]>2)
            msg_joint01.effort[6]=0;  //0.3
        pub_joint01.publish(msg_joint01);                                 


////topic ////////////////////////////////////////////////////


        ros::spinOnce();
        loop_rate.sleep();
        
    //    CAN_Handlej.arx_1();
    }
    CAN_Handlej.arx_2();
    return 0;
}

