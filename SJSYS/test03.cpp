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
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/Int8.h>
#include "geometry_msgs/PoseStamped.h"

using namespace std;
double angle_z;
double angle_y;
double angle_x;
double marker_00[7];
bool STARTMARKER582 = false;
bool SUCCESS = false;
int sockfd = socket(AF_INET, SOCK_STREAM, 0); // 面向连接或者流格式套接字
void infoCallback_aruco(const geometry_msgs::Pose::ConstPtr &msg_marker0);
void Rotation(double a[4]);

bool RecieveData()
{
    ROS_INFO("Recieve start message!!");
    int receive_singal;
    ssize_t recv_size = recv(sockfd, &receive_singal, sizeof(receive_singal), 0); // 返回接收的字节数
    ROS_INFO("Recieve start message!!");
    if (recv_size <= 0)
    {
        std::cerr << "Failed to receive data from MES." << std::endl;
        return false;
    }
    ROS_INFO("Recieve start message!!");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_dingyue");

    // ROS_INFO("test");

    // 创建TCP套接字
    if (sockfd == -1)
    {
        std::cerr << "Failed to create socket." << std::endl;
        return -1;
    }

    // 设置服务器信息
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;                        // IPv4
    server_addr.sin_port = htons(30006);                     // 服务器端口
    server_addr.sin_addr.s_addr = inet_addr("172.31.1.147"); // 服务器IP地址

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        std::cerr << "Failed to connect to the server." << std::endl;
        close(sockfd);
        return -1;
    }
    ROS_INFO("Connected Sever!!!");

    // 创建ROS节点句柄
    ros::NodeHandle nh_aruco;
    ros::Subscriber aruco_sub = nh_aruco.subscribe("/aruco_simple/pose0", 1, infoCallback_aruco);

    ros::Rate loop_rate(1); // 循环频率10ms
    while (ros::ok())
    {

        if (RecieveData())
        {

            ROS_INFO("Start to photo");
            ros::Duration(2).sleep();

            ROS_INFO("Start");
            ros::spinOnce();

            double Rota[4] = {marker_00[3], marker_00[4], marker_00[5], marker_00[6]};
            Rotation(Rota);

            if (angle_z > 1.57 && angle_z < 3.14)
            {
                ROS_INFO("Angles:1");

                angle_z = -(3.14159 - angle_z);
            }
            else if (angle_z > -3.14 && angle_z < -1.57)
            {
                ROS_INFO("Angles:2");
                angle_z = 3.14159 + angle_z;
            };
            ROS_INFO("testAngles:%f,%f,%f,%f,%f,%f", marker_00[0], marker_00[1], marker_00[2], angle_z, angle_y, angle_x);

            std::stringstream ss_aruco;
            ss_aruco << marker_00[0] << "," << marker_00[1] << "," << marker_00[2] << "," << angle_z << "," << angle_y << "," << angle_x;
            std::string aruco_messages = ss_aruco.str();

            // ROS_INFO("main_msg_marker:%f,%f,%f,%f,%f,%f,%f", marker_00[0], marker_00[1], marker_00[2], marker_00[3], marker_00[4], marker_00[5], marker_00[6]);
            ROS_INFO("Here!!!");
            if (send(sockfd, aruco_messages.c_str(), aruco_messages.size(), 0) < 0)
            {
                std::cerr << "Failed to send buffer." << std::endl;
                return -1;
            }
            ROS_INFO("msgs: %s", aruco_messages.c_str());
            ROS_INFO("msgs_size: %ld", aruco_messages.size());

            ROS_INFO("Finish!!");
            continue;
            // RecieveData() == false;
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

void infoCallback_aruco(const geometry_msgs::Pose::ConstPtr &msg_marker0)
{
    marker_00[0] = msg_marker0->position.x;
    marker_00[1] = msg_marker0->position.y;
    marker_00[2] = msg_marker0->position.z;
    marker_00[3] = msg_marker0->orientation.x;
    marker_00[4] = msg_marker0->orientation.y;
    marker_00[5] = msg_marker0->orientation.z;
    marker_00[6] = msg_marker0->orientation.w;
    ROS_INFO("msg_marker:%f,%f,%f,%f,%f,%f,%f", marker_00[0], marker_00[1], marker_00[2], marker_00[3], marker_00[4], marker_00[5], marker_00[6]);
    // STARTMARKER582 = true;
    // ROS_INFO("STARTMARKER:%d", STARTMARKER582);
};

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
