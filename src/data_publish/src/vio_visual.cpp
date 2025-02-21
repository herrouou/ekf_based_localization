#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>

// Structure to store VIO data
struct VIOdata {
    double timestamp;
    int frame;
    float px, py, pz;
    float x, y, z, w;
};

// Read VIO data from a CSV file
std::vector<VIOdata> readVIO(const std::string& filename) {
    std::vector<VIOdata> vio_data;
    std::fstream file(filename);
    std::string line;
    if (!file.is_open()) {
        ROS_ERROR("Cannot open file: %s", filename.c_str());
        return vio_data;
    }
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
    file.close();
    return vio_data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vio_visual_path");
    ros::NodeHandle nh("~");

    // Get VIO CSV file parameter
    std::string vio_csv_file;
    if (!nh.getParam("vio_csv_file", vio_csv_file)) {
        ROS_ERROR("Parameter 'vio_csv_file' not set");
        return 1;
    }

    // Read VIO data from CSV
    std::vector<VIOdata> vio_data = readVIO(vio_csv_file);
    if (vio_data.empty()) {
        ROS_ERROR("No VIO data to publish");
        return 1;
    }

    // Create a latched publisher for the Path message
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("vio_path", 1, true);

    // Build the Path message
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    for (const auto& data : vio_data) {
        geometry_msgs::PoseStamped pose_stamped;
        // Set header timestamp (using current time)
        pose_stamped.header.stamp = ros::Time::now(); 
        pose_stamped.header.frame_id = "map";
        // Set position from VIO data
        pose_stamped.pose.position.x = data.px;
        pose_stamped.pose.position.y = data.py;
        pose_stamped.pose.position.z = data.pz;
        // Set orientation from VIO data
        pose_stamped.pose.orientation.x = data.x;
        pose_stamped.pose.orientation.y = data.y;
        pose_stamped.pose.orientation.z = data.z;
        pose_stamped.pose.orientation.w = data.w;
        path_msg.poses.push_back(pose_stamped);
    }
    path_msg.header.stamp = ros::Time::now();

    // Publish the Path message
    path_pub.publish(path_msg);
    ROS_INFO("Published VIO path with %lu points", path_msg.poses.size());

    // Keep the node alive to ensure RViz can receive the latched message
    ros::Rate loop_rate(1); // 1 Hz
    while (ros::ok()) {
        path_pub.publish(path_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
