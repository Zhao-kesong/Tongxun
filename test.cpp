#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <geometry_msgs/Pose.h>

std::vector<double> marker_00(7, 0.0);
void infoCallback_aruco(const geometry_msgs::Pose::ConstPtr &msg_marker0);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_dingyue");

    // 创建TCP套接字
    int sockfd = socket(AF_INET, SOCK_STREAM, 0); // 面向连接或者流格式套接字
    if (sockfd == -1)
    {
        std::cerr << "Failed to create socket." << std::endl;
        return -1;
    }

    // 设置服务器信息
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;                         // IPv4
    server_addr.sin_port = htons(3333);                       // 服务器端口
    server_addr.sin_addr.s_addr = inet_addr("192.168.31.200"); // 服务器IP地址

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        std::cerr << "Failed to connect to the server." << std::endl;
        close(sockfd);
        return -1;
    }

    // 创建ROS节点句柄
    ros::NodeHandle nh_aruco;
    ros::Subscriber aruco_sub = nh_aruco.subscribe("1234", 10, infoCallback_aruco);

    while (ros::ok())
    {
        std::vector<double> aruco_megs = {marker_00[0], marker_00[1], marker_00[2], marker_00[3], marker_00[4], marker_00[5], marker_00[6]};
        std::cout << "Vector elements: ";
        for (double num : aruco_megs)
        {
            std::cout << num << " ";
        }
        
        int aruco_megs_size = aruco_megs.size();
        send(sockfd, &aruco_megs_size, sizeof(aruco_megs_size), 0);
        if (send(sockfd, aruco_megs.data(), sizeof(double) * aruco_megs_size, 0) < 0)
        {
            std::cerr << "Failed to send buffer." << std::endl;
            return -1;
        };
    }
    return 0;
};


void infoCallback_aruco(const geometry_msgs::Pose::ConstPtr &msg_marker0)
{
    marker_00[0] = msg_marker0->position.x;
    marker_00[1] = msg_marker0->position.y;
    marker_00[2] = msg_marker0->position.z;
    marker_00[4] = msg_marker0->orientation.y;
    marker_00[5] = msg_marker0->orientation.z;
    marker_00[6] = msg_marker0->orientation.w;
    ROS_INFO("msg_marker0:%f,%f,%f,%f,%f,%f,%f", marker_00[0], marker_00[1], marker_00[2], marker_00[3], marker_00[4], marker_00[5], marker_00[6]);
};