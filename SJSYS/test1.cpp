#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <unistd.h>
#include <geometry_msgs/Pose.h>

double marker_00[7];
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
    server_addr.sin_addr.s_addr = inet_addr("192.168.55.1"); // 服务器IP地址

    // 连接到服务器
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
    {
        std::cerr << "Failed to connect to the server." << std::endl;
        close(sockfd);
        return -1;
    }

    // 创建ROS节点句柄
    ros::NodeHandle nh_aruco;
    ros::Subscriber aruco_sub = nh_aruco.subscribe("/aruco_simple/pose0", 10, infoCallback_aruco);

    ros::Rate loop_rate(1); // 循环频率10ms
    while (ros::ok())
    {
        if (send(sockfd, marker_00, sizeof(marker_00), 0) < 0)
        {
            std::cerr << "Failed to send buffer." << std::endl;
            return -1;
        }
        ros::spinOnce(); // 循环等待回调函数
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
};

