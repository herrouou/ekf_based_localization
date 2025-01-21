#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

struct VIOdata {
    double timestamp;
    int frame;
    float px, py, pz;
    float x, y, z, w;
};

// 函数：从 CSV 文件读取 VIO 数据
std::vector<VIOdata> readVIO(const std::string& filename) {
    std::vector<VIOdata> vio_data;
    std::fstream file(filename);
    std::string line;

    // 检查文件是否成功打开
    if (!file.is_open()) {
        ROS_ERROR("无法打开文件: %s", filename.c_str());
        return vio_data;
    }

    std::getline(file, line); // 跳过标题行

    while (std::getline(file, line)) {
        // 移除行中的所有空格
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

        std::stringstream ss(line);
        std::string field;
        VIOdata data;

        // 按逗号分隔解析每个字段
        std::getline(ss, field, ',');
        data.timestamp = std::stod(field);
        std::getline(ss, field, ',');
        data.frame = std::stoi(field);
        std::getline(ss, field, ',');
        data.px = std::stof(field);
        std::getline(ss, field, ',');
        data.py = std::stof(field);
        std::getline(ss, field, ',');
        data.pz = std::stof(field);
        std::getline(ss, field, ',');
        data.x = std::stof(field);
        std::getline(ss, field, ',');
        data.y = std::stof(field);
        std::getline(ss, field, ',');
        data.z = std::stof(field);
        std::getline(ss, field, ',');
        data.w = std::stof(field);

        vio_data.push_back(data);
    }
    file.close();
    return vio_data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vio_visual_path");
    ros::NodeHandle nh("~");

    // 获取 VIO CSV 文件路径参数
    std::string vio_csv_file;
    if (!nh.getParam("vio_csv_file", vio_csv_file)) {
        ROS_ERROR("参数 'vio_csv_file' 未设置");
        return 1;
    }

    // 读取 CSV 文件中的 VIO 数据
    std::vector<VIOdata> vio_data = readVIO(vio_csv_file);

    if (vio_data.empty()) {
        ROS_ERROR("没有 VIO 数据可发布");
        return 1;
    }

    // 创建 Path 消息的发布器
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("vio_path", 1, true); // latched 为 true

    // 初始化 Path 消息
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map"; // 根据需要设置坐标框架

    // 遍历所有 VIO 数据点并添加到 Path 消息中
    for (const auto& data : vio_data) {
        geometry_msgs::PoseStamped pose_stamped;
        // 使用数据中的时间戳或当前时间
        pose_stamped.header.stamp = ros::Time::now(); // 或者使用 data.timestamp 进行转换
        pose_stamped.header.frame_id = "map";

        // 设置位置
        pose_stamped.pose.position.x = data.px;
        pose_stamped.pose.position.y = data.py;
        pose_stamped.pose.position.z = data.pz;

        // 设置方向（如果需要，可以使用实际的四元数；此处仅位置）
        pose_stamped.pose.orientation.x = data.x;
        pose_stamped.pose.orientation.y = data.y;
        pose_stamped.pose.orientation.z = data.z;
        pose_stamped.pose.orientation.w = data.w;

        // 将 PoseStamped 添加到 Path 中
        path_msg.poses.push_back(pose_stamped);
    }

    // 设置 Path 消息的时间戳
    path_msg.header.stamp = ros::Time::now();

    // 发布 Path 消息
    path_pub.publish(path_msg);
    ROS_INFO("已发布 VIO 路径，共 %lu 个点", path_msg.poses.size());

    // 保持节点运行以确保 RViz 能持续接收 Path 消息
    ros::Rate loop_rate(1); // 1 Hz 发布频率
    while (ros::ok()) {
        path_pub.publish(path_msg); // 持续发布 Path 消息（由于 latched 为 true，RViz 只需要第一次接收即可）
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
