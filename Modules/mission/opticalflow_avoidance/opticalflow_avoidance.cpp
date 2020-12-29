/***************************************************************************************************************************
 * opticalflow_avoidance.cpp
 *
 * Author: Zwb
 *
 * Update Time: 2020.12.28
 *
 * 说明: 光流法避障示例程序
 *      1. 
 *      2. 
 *      3. 发布上层控制指令 (prometheus_msgs::ControlCommand)
***************************************************************************************************************************/
//ros头文件
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <mission_utils.h>

//topic 头文件
#include <prometheus_msgs/ControlCommand.h>
#include <prometheus_msgs/DroneState.h>
#include <prometheus_msgs/DetectionInfo.h>
#include "message_utils.h"

using namespace std;
using namespace Eigen;
# define NODE_NAME "opticalflow_avoidance"
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
prometheus_msgs::DroneState _DroneState;   
Eigen::Vector3f drone_pos;
//---------------------------------------Vision---------------------------------------------
// 光流信息

//---------------------------------------Track---------------------------------------------

//---------------------------------------Output---------------------------------------------
prometheus_msgs::ControlCommand Command_Now;                               //发送给控制模块 [px4_pos_controller.cpp]的命令
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void drone_state_cb(const prometheus_msgs::DroneState::ConstPtr& msg)
{
    _DroneState = *msg;

    drone_pos[0] = _DroneState.position[0];
    drone_pos[1] = _DroneState.position[1];
    drone_pos[2] = _DroneState.position[2];
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_tracking");
    ros::NodeHandle nh("~");
    
    //节点运行频率： 20hz 【视觉端解算频率大概为20HZ】
    ros::Rate rate(20.0);

    // 【订阅】视觉消息 来自视觉节点
    //  方向定义： 识别算法发布的目标位置位于相机坐标系（从相机往前看，物体在相机右方x为正，下方y为正，前方z为正）
    //  标志位：   detected 用作标志位 ture代表识别到目标 false代表丢失目标
    // 注意这里为了复用程序使用了/prometheus/object_detection/kcf_tracker作为话题名字，适用于椭圆、二维码、yolo等视觉算法
    // 故同时只能运行一种视觉识别程序，如果想同时追踪多个目标，这里请修改接口话题的名字
    ros::Subscriber vision_sub = nh.subscribe<prometheus_msgs::DetectionInfo>("/prometheus/object_detection/kcf_tracker", 10, vision_cb);

    ros::Subscriber drone_state_sub = nh.subscribe<prometheus_msgs::DroneState>("/prometheus/drone_state", 10, drone_state_cb);

    // 【发布】发送给控制模块 [px4_pos_controller.cpp]的命令
    ros::Publisher command_pub = nh.advertise<prometheus_msgs::ControlCommand>("/prometheus/control_command", 10);

    // 【发布】用于地面站显示的提示消息
    ros::Publisher message_pub = nh.advertise<prometheus_msgs::Message>("/prometheus/message/main", 10);
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参数读取<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    //视觉丢失次数阈值
    nh.param<int>("Thres_vision", Thres_vision, 10);

    //追踪的前后间隔
    nh.param<float>("tracking_delta_x", tracking_delta[0], 0.0);
    nh.param<float>("tracking_delta_y", tracking_delta[1], 0.0);
    nh.param<float>("tracking_delta_z", tracking_delta[2], 0.0);

    nh.param<float>("camera_offset_x", camera_offset[0], 0.0);
    nh.param<float>("camera_offset_y", camera_offset[1], 0.0);
    nh.param<float>("camera_offset_z", camera_offset[2], 0.0);

    //追踪控制参数
    nh.param<float>("kpx_track", kpx_track, 0.1);
    nh.param<float>("kpy_track", kpy_track, 0.1);
    nh.param<float>("kpz_track", kpz_track, 0.1);

    nh.param<float>("start_point_x", start_point_x, 0.0);
    nh.param<float>("start_point_y", start_point_y, 0.0);
    nh.param<float>("start_point_z", start_point_z, 2.0);
    nh.param<float>("start_yaw", start_yaw, 0.0);

    //打印现实检查参数
    printf_param();
    //固定的浮点显示
    cout.setf(ios::fixed);
    //setprecision(n) 设显示小数精度为n位
    cout<<setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    // Waiting for input
    int start_flag = 0;
    while(start_flag == 0)
    {
        cout << ">>>>>>>>>>>>>>>>>>>>>>>>>Object Tracking Mission<<<<<<<<<<<<<<<<<<<<<< "<< endl;
        cout << "Please check the parameter and setting，enter 1 to continue， else for quit: "<<endl;
        cin >> start_flag;
    }

    // 起飞
    cout<<"[object tracking]: "<<"Takeoff to predefined position."<<endl;
    Command_Now.Command_ID = 1;
    Command_Now.source = NODE_NAME;
    while( _DroneState.position[2] < 0.3)
    {
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Mode  = prometheus_msgs::ControlCommand::Idle;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.yaw_ref = 999;
        command_pub.publish(Command_Now);   
        cout << "Switch to OFFBOARD and arm ..."<<endl;
        ros::Duration(3.0).sleep();
        
        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Mode                                = prometheus_msgs::ControlCommand::Move;
        Command_Now.Command_ID = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
        Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
        Command_Now.Reference_State.position_ref[0]     = start_point_x;
        Command_Now.Reference_State.position_ref[1]     = start_point_y;
        Command_Now.Reference_State.position_ref[2]     = start_point_z;
        Command_Now.Reference_State.yaw_ref             = start_yaw/180*3.1415926;
        command_pub.publish(Command_Now);
        cout << "Takeoff ..."<<endl;
        ros::Duration(3.0).sleep();

        ros::spinOnce();
    }

    // 先读取一些飞控的数据
    for(int i=0;i<10;i++)
    {
        ros::spinOnce();
        rate.sleep();
    }

    pos_des_prev[0] = drone_pos[0];
    pos_des_prev[1] = drone_pos[1];
    pos_des_prev[2] = drone_pos[2];

    ros::Duration(3.0).sleep();

    while (ros::ok())
    {
        //回调
        ros::spinOnce();

        Command_Now.header.stamp                    = ros::Time::now();
        Command_Now.Command_ID                      = Command_Now.Command_ID + 1;

        printf_result();

        distance_to_setpoint = pos_body_frame.norm();        
        if(!is_detected)
        {
            Command_Now.Mode = prometheus_msgs::ControlCommand::Hold;
            pos_des_prev[0] = drone_pos[0];
            pos_des_prev[1] = drone_pos[1];
            pos_des_prev[2] = drone_pos[2];
            cout <<"[object_tracking]: Lost the Target "<< endl;
            pub_message(message_pub, prometheus_msgs::Message::WARN, NODE_NAME, "Lost the Target.");
        }else 
        {
            cout <<"[object_tracking]: Tracking the Target, distance_to_setpoint : "<< distance_to_setpoint << " [m] " << endl;
            Command_Now.Mode = prometheus_msgs::ControlCommand::Move;
            Command_Now.Reference_State.Move_frame = prometheus_msgs::PositionReference::ENU_FRAME;
            Command_Now.Reference_State.Move_mode = prometheus_msgs::PositionReference::XYZ_POS;   //xy velocity z position

            Eigen::Vector3f vel_command;
            vel_command[0] = kpx_track * (pos_body_enu_frame[0] - tracking_delta[0]);
            vel_command[1] = kpy_track * (pos_body_enu_frame[1] - tracking_delta[1]);
            vel_command[2] = kpz_track * (pos_body_enu_frame[2] - tracking_delta[2]);

            for (int i=0; i<3; i++)
            {
                Command_Now.Reference_State.position_ref[i] = pos_des_prev[i] + vel_command[i]* 0.05;
            }
            Command_Now.Reference_State.yaw_ref             = start_yaw/180*3.1415926;
            
            for (int i=0; i<3; i++)
            {
                pos_des_prev[i] = Command_Now.Reference_State.position_ref[i];
            }
        }
        
        //Publish
        Command_Now.header.stamp = ros::Time::now();
        Command_Now.Command_ID   = Command_Now.Command_ID + 1;
        Command_Now.source = NODE_NAME;
        command_pub.publish(Command_Now);

        rate.sleep();
    }

    return 0;

}




