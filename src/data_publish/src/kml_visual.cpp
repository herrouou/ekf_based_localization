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

// 结构体：存储KML中的经纬高（或者直接存储x, y, z，取决于你的需求）
struct Coordinate {
    double longitude;
    double latitude;
    double altitude;
};

// 函数：解析KML文件，返回坐标数组
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

    // 找到"Document"->"Placemark"
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
                    // KML格式通常是 "longitude,latitude,altitude"
                    // 注意：如果顺序不同，请根据实际情况做修改
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

// 函数：解析time.txt文件
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
        // 1. 解析KML
        coordinates_ = parseKML(kml_file);
        // 2. 解析time.txt
        times_ = parseTimeFile(time_file);

        // 简单保护：若解析出的数量不一致，可能导致访问越界
        if (times_.size() != coordinates_.size()) {
            ROS_WARN("time.txt size (%lu) != KML coords size (%lu). Will use min of them.",
                     times_.size(), coordinates_.size());
            size_t min_size = std::min(times_.size(), coordinates_.size());
            coordinates_.resize(min_size);
            times_.resize(min_size);
        }

        // 发布者：用于发布KML整条Path（一次性或周期性都可以）
        path_pub_ = nh_.advertise<nav_msgs::Path>("kml_path_raw", 1);

        // 新增：发布单点 GPS（NavSatFix）
        gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/ground_truth", 1);

        // 订阅 vio_odom，等待第一次消息来“启动”播放
        vio_sub_ = nh_.subscribe("/vio_odom", 1, &KMLVisualizer::vioCallback, this);

        // 定时器，循环检查是否需要发布下一个点
        // 这里设置 50Hz （可根据需求调整）
        timer_ = nh_.createTimer(ros::Duration(0.02), &KMLVisualizer::timerCallback, this);

        // 准备一个 Path（一次性发布或持续发布都可以）
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
    }

private:
    void vioCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        // 只在第一次 vio_odom 到来时，记录起始时间
        if (!first_vio_odom_received_) {
            first_vio_odom_received_ = true;
            start_time_ = ros::Time::now();
            ROS_INFO("Received the first vio_odom, start replaying KML path from time=0");
        }
    }

    void timerCallback(const ros::TimerEvent &)
    {
        // 如果还没接收到 vio_odom，则不做任何发布
        if (!first_vio_odom_received_) {
            return;
        }

        // 不断发布整条Path（若不需要，可注释掉）
        path_.header.stamp = ros::Time::now();
        path_pub_.publish(path_);

        // 如果已经把所有点都发布完了，就不再继续
        if (current_idx_ >= times_.size()) {
            return;
        }

        // 计算当前时间与 start_time 的差值
        ros::Time now = ros::Time::now();
        double elapsed = (now - start_time_).toSec();  // 单位：秒

        // 当 elapsed 时间 >= times_[current_idx_] 时，可以发布这个点
        // 也可以考虑 while (elapsed >= times_[current_idx_]) 以防止跳过多帧
        if (elapsed >= times_[current_idx_]) {
            // 发布第 current_idx_ 个 GPS 点
            publishGpsPoint(current_idx_);
            current_idx_++;
        }
    }

    // 发布一个 NavSatFix 类型的 GPS 单点
    void publishGpsPoint(size_t idx)
    {
        if (idx >= coordinates_.size()) return;

        sensor_msgs::NavSatFix fix_msg;
        fix_msg.header.frame_id = "map";
        // 为了让时间戳与 time.txt 对应，可加上 start_time_，也可直接用 now
        fix_msg.header.stamp = start_time_ + ros::Duration(times_[idx]);

        fix_msg.latitude  = coordinates_[idx].latitude;
        fix_msg.longitude = coordinates_[idx].longitude;
        fix_msg.altitude  = coordinates_[idx].altitude;

        // 其他可选字段（如 status, position_covariance 等）可根据需要设置
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

    // 订阅 vio_odom：用来“启动”我们的发布
    ros::Subscriber vio_sub_;

    // 发布：Path（可选） + GPS 单点
    ros::Publisher path_pub_;
    ros::Publisher gps_pub_;

    // 定时器：控制发布节奏
    ros::Timer timer_;

    // 数据
    std::vector<Coordinate> coordinates_;
    std::vector<double> times_;

    nav_msgs::Path path_;

    // 状态变量
    bool first_vio_odom_received_;
    ros::Time start_time_;   // 接收第一个vio_odom时的时间
    size_t current_idx_;     // 当前要发布第几个点
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "kml_visual");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // 从参数服务器获取KML和time文件路径
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