#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
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

std::vector<VIOdata> readVIO(const std::string& filename) {
    std::vector<VIOdata> vio_data;
    std::fstream file(filename);
    std::string line;
    std::getline(file, line); // Skip header

    while (std::getline(file, line)) {
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

        std::stringstream ss(line);
        std::string field;
        VIOdata data;

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
    return vio_data;
}

void publishDataAtTimestamp(
    ros::Publisher& vio_pub,
    const VIOdata& data,
    ros::Time start_time,
    double initial_timestamp) {
    nav_msgs::Odometry vio_msg;

    // Convert relative timestamp to ROS time
    ros::Time msg_time = start_time + ros::Duration(data.timestamp - initial_timestamp);

    vio_msg.header.stamp = msg_time;
    vio_msg.header.frame_id = "map";
    vio_msg.child_frame_id = "base_link";

    vio_msg.pose.pose.position.x = data.px;
    vio_msg.pose.pose.position.y = data.py;
    vio_msg.pose.pose.position.z = data.pz;

    vio_msg.pose.pose.orientation.x = data.x;
    vio_msg.pose.pose.orientation.y = data.y;
    vio_msg.pose.pose.orientation.z = data.z;
    vio_msg.pose.pose.orientation.w = data.w;

    vio_msg.pose.covariance[0] = 0.01;  // x
    vio_msg.pose.covariance[7] = 0.01;  // y
    vio_msg.pose.covariance[14] = 0.01; // z

    vio_msg.pose.covariance[21] = 0.001; // roll
    vio_msg.pose.covariance[28] = 0.001; // pitch
    vio_msg.pose.covariance[35] = 0.001; // yaw

    vio_pub.publish(vio_msg);
    ROS_INFO("Published VIO data at timestamp: %.3f", data.timestamp);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vio_visual");
    ros::NodeHandle nh("~");

    std::string vio_csv_file;
    nh.getParam("vio_csv_file", vio_csv_file);

    std::vector<VIOdata> vio_data = readVIO(vio_csv_file);

    if (vio_data.empty()) {
        ROS_ERROR("No data to publish");
        return 1;
    }

    // 创建 Publisher
    ros::Publisher vio_pub = nh.advertise<nav_msgs::Odometry>("vio_odom", 10);

    // 获取初始时间戳，用于同步 ROS 时间
    double initial_timestamp = vio_data.front().timestamp;
    ros::Time start_time = ros::Time::now();

    size_t vio_index = 0;
    ros::Rate loop_rate(100); // 控制主循环的频率

    while (ros::ok() && vio_index < vio_data.size()) {
        // 计算目标时间点
        ros::Time target_time = start_time + ros::Duration(vio_data[vio_index].timestamp - initial_timestamp);

        // 如果当前时间已经达到目标时间点，发布数据
        if (ros::Time::now() >= target_time) {
            publishDataAtTimestamp(vio_pub, vio_data[vio_index], start_time, initial_timestamp);
            vio_index++;
        }

        ros::spinOnce(); // 处理 ROS 回调
        loop_rate.sleep();
    }

    ROS_INFO("Finished publishing all VIO data.");
    return 0;
}
