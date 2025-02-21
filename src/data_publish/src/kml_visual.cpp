#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tinyxml.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath> 

// Structure to store KML coordinates
struct Coordinate {
    double longitude;
    double latitude;
    double altitude;
};

// Parse KML file and return vector of coordinates
std::vector<Coordinate> parseKML(const std::string &file_path) {
    std::vector<Coordinate> coordinates;
    TiXmlDocument doc(file_path.c_str());
    if(!doc.LoadFile()) {
        ROS_ERROR("Fail to load the file: %s", file_path.c_str());
        return coordinates;
    }

    TiXmlElement* root = doc.RootElement();
    if (!root) {
        ROS_ERROR("No root elements found in %s", file_path.c_str());
        return coordinates;
    }

    // Find "Document" -> "Placemark"
    TiXmlElement* placemark = root->FirstChildElement("Document")->FirstChildElement("Placemark");
    while (placemark) {
        TiXmlElement* lineString = placemark->FirstChildElement("LineString");
        if (lineString) {
            TiXmlElement* coordinateElement = lineString->FirstChildElement("coordinates");
            if (coordinateElement && coordinateElement->GetText()) {
                std::string coord_text = coordinateElement->GetText();
                std::istringstream ss(coord_text);
                std::string coordinate;
                while (std::getline(ss, coordinate, ' ')) {
                    double lo, la, al;
                    if (sscanf(coordinate.c_str(), "%lf,%lf,%lf", &lo, &la, &al) == 3) {
                        coordinates.push_back({lo, la, al});
                    }
                }
            }
        }
        placemark = placemark->NextSiblingElement("Placemark");
    }
    ROS_INFO("Parsed %lu coordinates from %s", coordinates.size(), file_path.c_str());
    return coordinates;
}

// Parse time.txt file and return vector of time stamps
std::vector<double> parseTimeFile(const std::string &time_file) {
    std::vector<double> times;
    std::ifstream ifs(time_file.c_str());
    if (!ifs.is_open()) {
        ROS_ERROR("Fail to open time file: %s", time_file.c_str());
        return times;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        double t = 0.0;
        if (std::sscanf(line.c_str(), "%lf", &t) == 1) {
            times.push_back(t);
        }
    }
    ROS_INFO("Parsed %lu time stamps from %s", times.size(), time_file.c_str());
    return times;
}

class KMLVisualizer
{
public:
    KMLVisualizer(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                  const std::string &kml_file, const std::string &time_file)
        : nh_(nh),
          nh_private_(nh_private),
          first_vio_odom_received_(false),
          current_idx_(0)
    {
        // Parse KML and time file
        coordinates_ = parseKML(kml_file);
        times_ = parseTimeFile(time_file);

        // Resize if sizes differ
        if (times_.size() != coordinates_.size()) {
            ROS_WARN("time.txt size (%lu) != KML coords size (%lu). Will use min of them.",
                     times_.size(), coordinates_.size());
            size_t min_size = std::min(times_.size(), coordinates_.size());
            coordinates_.resize(min_size);
            times_.resize(min_size);
        }

        // Setup publishers and subscriber
        path_pub_ = nh_.advertise<nav_msgs::Path>("kml_path_raw", 1);
        path_interpolation_pub_ = nh_.advertise<nav_msgs::Path>("kml_path_interpolation", 1);
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/ground_truth", 1);
        vio_sub_ = nh_.subscribe("/vio_odom", 1, &KMLVisualizer::vioCallback, this);

        // Setup timer at 50Hz
        timer_ = nh_.createTimer(ros::Duration(0.02), &KMLVisualizer::timerCallback, this);

        // Prepare Path message from KML data
        path_.header.frame_id = "map";
        path_.header.stamp = ros::Time::now();
        for (size_t i = 0; i < coordinates_.size(); ++i) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now() + ros::Duration(times_[i]);
            pose.pose.position.x = coordinates_[i].latitude;
            pose.pose.position.y = coordinates_[i].longitude;
            pose.pose.position.z = coordinates_[i].altitude;
            pose.pose.orientation.w = 1.0;
            path_.poses.push_back(pose);
        }

        time_interval = 1;
        KMLVisualizer::PathInterpolation(path_, times_, time_interval);
    }

private:
    // Callback to record first vio_odom arrival time
    void vioCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        if (!first_vio_odom_received_) {
            first_vio_odom_received_ = true;
            start_time_ = ros::Time::now();
            ROS_INFO("Received the first vio_odom, start replaying KML path from time=0");
        }
    }

    // Interpolate between path poses at given time interval
    void PathInterpolation(nav_msgs::Path &path, std::vector<double> &time, double time_interval){
        if (path.poses.size() != time.size()){
            ROS_ERROR("can't interpolate!");
            return;
        }
        path_interpolation_.poses.push_back(path.poses.front());
        for (size_t i = 1; i < path.poses.size(); ++i){
            const geometry_msgs::PoseStamped &prev_pose = path_interpolation_.poses.back();
            const geometry_msgs::PoseStamped &current_pose = path.poses[i];
            int interval_times = static_cast<int>(std::floor((time[i] - time[i-1]) / time_interval));
            double time_difference = time[i] - time[i-1];
            float pose_distance_x = current_pose.pose.position.x - prev_pose.pose.position.x;
            float pose_distance_y = current_pose.pose.position.y - prev_pose.pose.position.y;
            for (size_t j = 1; j <= interval_times; ++j){
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now() + ros::Duration(time[i-1] + j * time_interval);
                pose.pose.position.x = prev_pose.pose.position.x + pose_distance_x * j * time_interval / time_difference;
                pose.pose.position.y = prev_pose.pose.position.y + pose_distance_y * j * time_interval / time_difference;
                pose.pose.position.z = 0;
                path_interpolation_.poses.push_back(pose);
            }
        }
    }

    // Timer callback to publish KML points and interpolated paths based on elapsed time
    void timerCallback(const ros::TimerEvent &)
    {
        if (!first_vio_odom_received_) {
            return;
        }
        path_.header.stamp = ros::Time::now();
        path_pub_.publish(path_);
        path_interpolation_pub_.publish(path_interpolation_);
        if (current_idx_ >= times_.size()) {
            return;
        }
        ros::Time now = ros::Time::now();
        double elapsed = (now - start_time_).toSec();  // seconds elapsed
        if (elapsed >= times_[current_idx_]) {
            publishGpsPoint(current_idx_);
            current_idx_++;
        }
    }

    // Publish a single GPS point as NavSatFix message
    void publishGpsPoint(size_t idx)
    {
        if (idx >= coordinates_.size()) return;
        sensor_msgs::NavSatFix fix_msg;
        fix_msg.header.frame_id = "map";
        fix_msg.header.stamp = start_time_ + ros::Duration(times_[idx]);
        fix_msg.latitude  = coordinates_[idx].latitude;
        fix_msg.longitude = coordinates_[idx].longitude;
        fix_msg.altitude  = coordinates_[idx].altitude;
        fix_msg.status.status  = sensor_msgs::NavSatStatus::STATUS_FIX;
        fix_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        gps_pub_.publish(fix_msg);
        ROS_INFO("Published kml point %lu at time=%.3f (longitude=%.6f, latitude=%.6f, altitude=%.3f)",
                 idx, times_[idx],
                 coordinates_[idx].longitude,
                 coordinates_[idx].latitude,
                 coordinates_[idx].altitude);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber vio_sub_;
    ros::Publisher path_pub_;
    ros::Publisher path_interpolation_pub_;
    ros::Publisher gps_pub_;
    ros::Timer timer_;

    std::vector<Coordinate> coordinates_;
    std::vector<double> times_;
    std::vector<double> times_interpolation_;

    nav_msgs::Path path_;
    nav_msgs::Path path_interpolation_;    

    bool first_vio_odom_received_;
    ros::Time start_time_;
    size_t current_idx_;
    double time_interval;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "kml_visual");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Get KML and time file paths from parameter server
    std::string kml_file, time_file;
    nh_priv.param<std::string>("kml_file", kml_file, "");
    nh_priv.param<std::string>("time_file", time_file, "");

    if (kml_file.empty() || time_file.empty()) {
        ROS_ERROR("Please set both ~kml_file and ~time_file parameters!");
        return -1;
    }

    KMLVisualizer node(nh, nh_priv, kml_file, time_file);
    ros::spin();
    return 0;
}
