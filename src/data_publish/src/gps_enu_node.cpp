#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <geo_transform/geo_transform.h>
#include <tf2/LinearMath/Quaternion.h>

class GPSToENUNode {
public:
  GPSToENUNode(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : nh_(nh), nh_priv_(nh_priv), initialized_(false)
  {
    // 从参数服务器获取参考点，如果没有，则等待首帧GPS数据后再设置
    if (nh_priv_.getParam("ref_lat", ref_lat_) &&
        nh_priv_.getParam("ref_lon", ref_lon_) &&
        nh_priv_.getParam("ref_alt", ref_alt_)) {
      geo_trans_ = std::make_shared<geo_transform::GeoTransform>(ref_lat_, ref_lon_, ref_alt_);
      initialized_ = true;
      ROS_INFO("Using provided reference point: lat=%f, lon=%f, alt=%f",
               ref_lat_, ref_lon_, ref_alt_);
    } else {
      ROS_WARN("No reference point provided, will use the first GPS fix as reference.");
    }

    // 订阅GPS数据
    gps_sub_ = nh_.subscribe("/gps_data", 10, &GPSToENUNode::gpsCallback, this);
    // 发布ENU里程计
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps/enu_odom", 10);

    // 订阅过滤后的里程计数据
    filtered_odom_sub_ = nh_.subscribe("/odometry/filtered", 10, &GPSToENUNode::filteredOdomCallback, this);
    // 发布转换回的GPS数据
    filtered_gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/filtered_gps", 10);
  }

private:
  // 回调函数：处理GPS数据并转换为ENU里程计
  void gpsCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
      ROS_WARN_THROTTLE(5, "No GPS fix available.");
      return;
    }

    if (!initialized_) {
      // 使用第一条有效GPS数据作为参考点
      ref_lat_ = msg->latitude;
      ref_lon_ = msg->longitude;
      ref_alt_ = msg->altitude;
      geo_trans_ = std::make_shared<geo_transform::GeoTransform>(ref_lat_, ref_lon_, ref_alt_);
      initialized_ = true;
      ROS_INFO("Set reference point to: lat=%f, lon=%f, alt=%f", ref_lat_, ref_lon_, ref_alt_);
    }

    double e, n, u;
    geo_trans_->lla2enu(msg->latitude, msg->longitude, msg->altitude, e, n, u);

    nav_msgs::Odometry odom_msg;
    odom_msg.header = msg->header;
    odom_msg.header.frame_id = "map";        // 全局坐标系
    odom_msg.child_frame_id = "base_link";   // 机器人坐标系，可根据需求修改

    // 设置位置
    odom_msg.pose.pose.position.x = e;
    odom_msg.pose.pose.position.y = n;
    odom_msg.pose.pose.position.z = u;

    // 设置协方差（根据实际需求调整）
    odom_msg.pose.covariance[0] = 15;   // x
    odom_msg.pose.covariance[7] = 15;   // y
    odom_msg.pose.covariance[14] = 15;  // z

    // 设置姿态为单位四元数（无旋转）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 发布里程计消息
    odom_pub_.publish(odom_msg);
  }

  // 新增回调函数：处理过滤后的里程计数据并转换回GPS数据
  void filteredOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (!initialized_) {
      ROS_WARN_THROTTLE(5, "Reference point not initialized, cannot convert ENU to LLA.");
      return;
    }

    // 获取ENU坐标
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    // 转换回经纬度
    double lat, lon, alt;
    geo_trans_->enu2lla(x, y, z, lat, lon, alt);

    // 创建并填充NavSatFix消息
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header = msg->header;
    gps_msg.header.frame_id = "map";  // 可根据需求调整
    gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    gps_msg.latitude = lat;
    gps_msg.longitude = lon;
    gps_msg.altitude = alt;

    gps_msg.position_covariance[0] = 5;
    gps_msg.position_covariance[4] = 5;
    gps_msg.position_covariance[8] = 10;


    // 设置协方差（如果有相关信息，可以在此设置）
    // 这里假设协方差未知或不需要设置
    // 可以根据需要从里程计的协方差中推断或设置默认值

    // 发布转换后的GPS消息
    filtered_gps_pub_.publish(gps_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  
  ros::Subscriber gps_sub_;
  ros::Publisher odom_pub_;

  // 新增订阅者和发布者
  ros::Subscriber filtered_odom_sub_;
  ros::Publisher filtered_gps_pub_;

  std::shared_ptr<geo_transform::GeoTransform> geo_trans_;
  bool initialized_;
  double ref_lat_, ref_lon_, ref_alt_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_enu_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  GPSToENUNode node(nh, nh_priv);

  ros::spin();
  return 0;
}
