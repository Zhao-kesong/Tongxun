#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include "geometry_msgs/PoseStamped.h"

using namespace std;
double angle_z;
double angle_y;
double angle_x;
double marker_00[7];
double marker_01[7];
double marker_02[7];
double marker_03[7];
double marker_04[7];
bool STARTMARKER582 = false;
bool SUCCESS = false;
char receive_singal[10];
int sockfd = socket(AF_INET, SOCK_STREAM, 0); // 面向连接或者流格式套接字
void infoCallback_aruco_582(const geometry_msgs::Pose::ConstPtr &msg_marker0);
void infoCallback_aruco_001(const geometry_msgs::Pose::ConstPtr &msg_marker1);
void infoCallback_aruco_002(const geometry_msgs::Pose::ConstPtr &msg_marker2);
void infoCallback_aruco_003(const geometry_msgs::Pose::ConstPtr &msg_marker3);
void infoCallback_aruco_006(const geometry_msgs::Pose::ConstPtr &msg_marker4);
void Rotation(double a[4]);
double angle_z_value();

bool RecieveData()
{
    ROS_INFO("Recieve start message1!!");
    memset(receive_singal, 0, sizeof(receive_singal));
    ROS_INFO("Recieve start message2!!");
    ssize_t recv_size = recv(sockfd, receive_singal, sizeof(receive_singal), 0); // 返回接收的字节数
    ROS_INFO("here!!");
    //ROS_INFO("changdu:%d", sizeof(receive_singal));
    
    if (recv_size <= 0)
    {
        std::cerr << "Failed to receive data from MES." << std::endl;
        return false;
    }
    ROS_INFO("Start message is %s", receive_singal);
    //ROS_INFO("Recieve start message3!!");
    return true;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_dingyue");

    ROS_INFO("test");

    // 创建TCP套接字
    if (sockfd == -1)
    {
        std::cerr << "Failed to create socket." << std::endl;
        return -1;
    }
    // 设置服务器信息
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;                         // IPv4
    server_addr.sin_port = htons(6666);                      // 服务器端口
    server_addr.sin_addr.s_addr = inet_addr("172.31.1.147"); // 服务器IP地址

    /*
        // 连接到服务器
        if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
        {
            std::cerr << "Failed to connect to the server." << std::endl;
            return -1;
        }
        ROS_INFO("Connected Sever!!!");
    */
    while (true)
    {
        if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
        {
            std::cerr << "Failed to connect to the server." << std::endl;
            sleep(1);
            continue;
        }
        ROS_INFO("Connected Sever!!!");
        break;
    }

    // 创建ROS节点句柄
    ros::NodeHandle nh_aruco;
    ros::Subscriber aruco_sub0 = nh_aruco.subscribe("/aruco_simple/pose0", 1, infoCallback_aruco_582);
    ros::Subscriber aruco_sub1 = nh_aruco.subscribe("/aruco_simple/pose1", 1, infoCallback_aruco_001);
    ros::Subscriber aruco_sub2 = nh_aruco.subscribe("/aruco_simple/pose2", 1, infoCallback_aruco_002);
    ros::Subscriber aruco_sub3 = nh_aruco.subscribe("/aruco_simple/pose3", 1, infoCallback_aruco_003);
    ros::Subscriber aruco_sub4 = nh_aruco.subscribe("/aruco_simple/pose4", 1, infoCallback_aruco_006);

    ros::Rate loop_rate(5); // 循环频率10ms
    while (ros::ok())
    {

        if (RecieveData())
        {
            std::stringstream ss_aruco;
            string receive_id = receive_singal;

            ROS_INFO("Start to photo");
            ros::Duration(2).sleep();

            ros::spinOnce();
            //ROS_INFO("Start");

            if (receive_id == "id0")
            {
                double Rota0[4] = {marker_00[3], marker_00[4], marker_00[5], marker_00[6]};
                Rotation(Rota0);
                angle_z_value();
                ss_aruco << marker_00[0] << "," << marker_00[1] << "," << marker_00[2] << "," << angle_z << "," << angle_y << "," << angle_x;
                std::string aruco_messages = ss_aruco.str();
                if (send(sockfd, aruco_messages.c_str(), aruco_messages.size(), 0) < 0)
                {
                    std::cerr << "Failed to send buffer." << std::endl;
                    return -1;
                }
                //ROS_INFO("msgs_id0: %s", aruco_messages.c_str());
                //ROS_INFO("msgs_id0_size: %ld", aruco_messages.size());
                ROS_INFO("Send jointAngles_id0:%f,%f,%f,%f,%f,%f", marker_00[0], marker_00[1], marker_00[2], angle_z, angle_y, angle_x);
            }
            else if (receive_id == "id1")
            {
                double Rota1[4] = {marker_01[3], marker_01[4], marker_01[5], marker_01[6]};
                Rotation(Rota1);
                angle_z_value();
                ss_aruco << marker_01[0] << "," << marker_01[1] << "," << marker_01[2] << "," << angle_z << "," << angle_y << "," << angle_x;
                std::string aruco_messages = ss_aruco.str();
                if (send(sockfd, aruco_messages.c_str(), aruco_messages.size(), 0) < 0)
                {
                    std::cerr << "Failed to send buffer." << std::endl;
                    return -1;
                }
                //ROS_INFO("msgs_id1: %s", aruco_messages.c_str());
                //ROS_INFO("msgs_size_id1: %ld", aruco_messages.size());
                ROS_INFO("Send jointAngles_id1:%f,%f,%f,%f,%f,%f", marker_01[0], marker_01[1], marker_01[2], angle_z, angle_y, angle_x);
            }
            else if (receive_id == "id2")
            {
                double Rota2[4] = {marker_02[3], marker_02[4], marker_02[5], marker_02[6]};
                Rotation(Rota2);
                angle_z_value();
                ss_aruco << marker_02[0] << "," << marker_02[1] << "," << marker_02[2] << "," << angle_z << "," << angle_y << "," << angle_x;
                std::string aruco_messages = ss_aruco.str();
                if (send(sockfd, aruco_messages.c_str(), aruco_messages.size(), 0) < 0)
                {
                    std::cerr << "Failed to send buffer." << std::endl;
                    return -1;
                }
                //ROS_INFO("msgs_id2: %s", aruco_messages.c_str());
                //ROS_INFO("msgs_id2_size: %ld", aruco_messages.size());
                ROS_INFO("Send jointAngles_id2:%f,%f,%f,%f,%f,%f", marker_02[0], marker_02[1], marker_02[2], angle_z, angle_y, angle_x);
            }
            else if (receive_id == "id3")
            {
                double Rota3[4] = {marker_03[3], marker_03[4], marker_03[5], marker_03[6]};
                Rotation(Rota3);
                angle_z_value();
                ss_aruco << marker_03[0] << "," << marker_03[1] << "," << marker_03[2] << "," << angle_z << "," << angle_y << "," << angle_x;
                std::string aruco_messages = ss_aruco.str();
                if (send(sockfd, aruco_messages.c_str(), aruco_messages.size(), 0) < 0)
                {
                    std::cerr << "Failed to send buffer." << std::endl;
                    return -1;
                }
                //ROS_INFO("msgs_id3: %s", aruco_messages.c_str());
                //ROS_INFO("msgs_id3_size: %ld", aruco_messages.size());
                ROS_INFO("Send jointAngles_id3:%f,%f,%f,%f,%f,%f", marker_03[0], marker_03[1], marker_03[2], angle_z, angle_y, angle_x);
            }
            else if (receive_id == "id4")
            {
                double Rota4[4] = {marker_04[3], marker_04[4], marker_04[5], marker_04[6]};
                Rotation(Rota4);
                angle_z_value();
                ss_aruco << marker_04[0] << "," << marker_04[1] << "," << marker_04[2] << "," << angle_z << "," << angle_y << "," << angle_x;
                std::string aruco_messages = ss_aruco.str();
                if (send(sockfd, aruco_messages.c_str(), aruco_messages.size(), 0) < 0)
                {
                    std::cerr << "Failed to send buffer." << std::endl;
                    return -1;
                }
                //ROS_INFO("msgs_id4: %s", aruco_messages.c_str());
                //ROS_INFO("msgs_id4_size: %ld", aruco_messages.size());
                ROS_INFO("Send jointAngles_id4:%f,%f,%f,%f,%f,%f", marker_04[0], marker_04[1], marker_04[2], angle_z, angle_y, angle_x);
            }
            else
            {
                ROS_INFO("No id pass!!!");
            }

            ROS_INFO("Complish to send joint_Angles !!");
            continue;
            ROS_INFO("have quitted!!");
        }
        else
        {
            ROS_INFO("Not to photo");
        }

        loop_rate.sleep();
    }

    // 关闭套接字
    close(sockfd);

    return 0;
};

void infoCallback_aruco_582(const geometry_msgs::Pose::ConstPtr &msg_marker0)
{
    marker_00[0] = msg_marker0->position.x;
    marker_00[1] = msg_marker0->position.y;
    marker_00[2] = msg_marker0->position.z;
    marker_00[3] = msg_marker0->orientation.x;
    marker_00[4] = msg_marker0->orientation.y;
    marker_00[5] = msg_marker0->orientation.z;
    marker_00[6] = msg_marker0->orientation.w;
    ROS_INFO("msg_marker_582:%f,%f,%f,%f,%f,%f,%f", marker_00[0], marker_00[1], marker_00[2], marker_00[3], marker_00[4], marker_00[5], marker_00[6]);
};

void infoCallback_aruco_001(const geometry_msgs::Pose::ConstPtr &msg_marker1)
{
    marker_01[0] = msg_marker1->position.x;
    marker_01[1] = msg_marker1->position.y;
    marker_01[2] = msg_marker1->position.z;
    marker_01[3] = msg_marker1->orientation.x;
    marker_01[4] = msg_marker1->orientation.y;
    marker_01[5] = msg_marker1->orientation.z;
    marker_01[6] = msg_marker1->orientation.w;
    ROS_INFO("msg_marker_001:%f,%f,%f,%f,%f,%f,%f", marker_01[0], marker_01[1], marker_01[2], marker_01[3], marker_01[4], marker_01[5], marker_01[6]);
}

void infoCallback_aruco_002(const geometry_msgs::Pose::ConstPtr &msg_marker2)
{
    marker_02[0] = msg_marker2->position.x;
    marker_02[1] = msg_marker2->position.y;
    marker_02[2] = msg_marker2->position.z;
    marker_02[3] = msg_marker2->orientation.x;
    marker_02[4] = msg_marker2->orientation.y;
    marker_02[5] = msg_marker2->orientation.z;
    marker_02[6] = msg_marker2->orientation.w;
    ROS_INFO("msg_marker_002:%f,%f,%f,%f,%f,%f,%f", marker_02[0], marker_02[1], marker_02[2], marker_02[3], marker_02[4], marker_02[5], marker_02[6]);
}

void infoCallback_aruco_003(const geometry_msgs::Pose::ConstPtr &msg_marker3)
{
    marker_03[0] = msg_marker3->position.x;
    marker_03[1] = msg_marker3->position.y;
    marker_03[2] = msg_marker3->position.z;
    marker_03[3] = msg_marker3->orientation.x;
    marker_03[4] = msg_marker3->orientation.y;
    marker_03[5] = msg_marker3->orientation.z;
    marker_03[6] = msg_marker3->orientation.w;
    ROS_INFO("msg_marker_003:%f,%f,%f,%f,%f,%f,%f", marker_03[0], marker_03[1], marker_03[2], marker_03[3], marker_03[4], marker_03[5], marker_03[6]);
}

void infoCallback_aruco_006(const geometry_msgs::Pose::ConstPtr &msg_marker4)
{
    marker_04[0] = msg_marker4->position.x;
    marker_04[1] = msg_marker4->position.y;
    marker_04[2] = msg_marker4->position.z;
    marker_04[3] = msg_marker4->orientation.x;
    marker_04[4] = msg_marker4->orientation.y;
    marker_04[5] = msg_marker4->orientation.z;
    marker_04[6] = msg_marker4->orientation.w;
    ROS_INFO("msg_marker_006:%f,%f,%f,%f,%f,%f,%f", marker_04[0], marker_04[1], marker_04[2], marker_04[3], marker_04[4], marker_04[5], marker_04[6]);
}

void Rotation(double a[4])
{

    // 假设有两个四元数表示两种不同的旋转顺序
    Eigen::Quaterniond quaternion1(a[0], a[1], a[2], a[3]); // 示例四元数1

    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d rotationMatrix1 = quaternion1.toRotationMatrix();

    // 提取欧拉角，注意顺序
    Eigen::Vector3d euler_angles1 = rotationMatrix1.eulerAngles(2, 1, 0); // 顺序为 ZYX

    // 存储欧拉角的三个值
    angle_z = euler_angles1(2); // 绕 Z 轴的角度
    angle_y = euler_angles1(1); // 绕 Y 轴的角度
    angle_x = euler_angles1(0); // 绕 X 轴的角度

    ROS_INFO("Angles:%f,%f,%f", angle_z, angle_y, angle_x);

    // // 输出欧拉角
    // std::cout << "Euler Angles for Quaternion 1 (ZYX): " << euler_angles1.transpose() << "\n";
};

double angle_z_value()
{
    if (angle_z > M_PI_2 && angle_z < M_PI)
    {
        ROS_INFO("Angles:1");

        angle_z = -(M_PI - angle_z);
    }
    else if (angle_z > -M_PI && angle_z < -M_PI_2)
    {
        ROS_INFO("Angles:2");
        angle_z = M_PI + angle_z;
    };

    return angle_z;
}