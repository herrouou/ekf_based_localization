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
    // Load reference point parameters if provided
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

    // Initialize internal variables
    accumulate_time = 0;
    accumulate_times = 1;
    last_gps_time = 0;
    last_gps_x = 0;
    last_gps_y = 0;

    // Subscribers and publishers
    gps_sub_ = nh_.subscribe("/gps_data", 10, &GPSToENUNode::gpsCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/gps/enu_odom", 10);
    filtered_odom_sub_ = nh_.subscribe("/odometry/filtered", 10, &GPSToENUNode::filteredOdomCallback, this);
    filtered_gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/filtered_gps", 10);
  }

private:
  // Handle incoming raw GPS data, convert to ENU, and publish as nav_msgs::Odometry
  void gpsCallback(const sensor_msgs::NavSatFixConstPtr& msg) {
    if (msg->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
      ROS_WARN_THROTTLE(5, "No GPS fix available.");
      return;
    }

    // If not initialized, set the first valid GPS data as reference
    if (!initialized_) {
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
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "base_link";

    current_gps_time = odom_msg.header.stamp.toSec();
    current_gps_x = e;
    current_gps_y = n;

    // Detect outlier based on velocity
    bool outlier_flag = false;
    if (last_gps_time != 0) {
      double velocity = sqrt(pow((current_gps_x - last_gps_x), 2) + 
                             pow((current_gps_y - last_gps_y), 2))
                        / (current_gps_time - last_gps_time);
      ROS_INFO_STREAM("velocity: " << velocity);
      if (velocity >= 2) {
        outlier_flag = true;
      }
    }

    outlier_flag_list.push_back(outlier_flag);
    bool is_outlier = true;
    int continue_sum = 5;

    // Check a small recent window if all values are non-outliers
    if (outlier_flag_list.size() >= (size_t)continue_sum) {
      if (std::all_of(outlier_flag_list.end() - continue_sum, 
                      outlier_flag_list.end(), 
                      [](bool val) {return !val;})) {
        is_outlier = false;
      }
    }

    // Accumulate time logic (example)
    if (((current_gps_time - last_gps_time) < 1.5) && (accumulate_times < 140)) {
      accumulate_time = accumulate_time + current_gps_time;
      accumulate_times = accumulate_times + 5;
    } else {
      if (accumulate_times > 6) {
        accumulate_times = accumulate_times - 5;
      }
    }
    if (accumulate_times >= 140) {
      accumulate_times = 140;
    }

    // Set position in ENU
    odom_msg.pose.pose.position.x = e;
    odom_msg.pose.pose.position.y = n;
    odom_msg.pose.pose.position.z = u;

    // Set different covariance for outliers
    if (!is_outlier) {
      odom_msg.pose.covariance[0] = 145 - accumulate_times;
      odom_msg.pose.covariance[7] = 145 - accumulate_times;
      odom_msg.pose.covariance[14] = 150;
    } else {
      odom_msg.pose.covariance[0] = 1000;
      odom_msg.pose.covariance[7] = 1000;
      odom_msg.pose.covariance[14] = 1000;
    }

    ROS_INFO_STREAM("covariance: " << odom_msg.pose.covariance[0]);

    // Orientation is set to identity
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Some arbitrary orientation covariance
    odom_msg.pose.covariance[21] = 0.2;
    odom_msg.pose.covariance[28] = 0.2;
    odom_msg.pose.covariance[35] = 0.2;

    // Publish the odometry
    odom_pub_.publish(odom_msg);

    last_gps_time = current_gps_time;
    last_gps_x = current_gps_x;
    last_gps_y = current_gps_y;
  }

  // Handle filtered odometry, convert ENU back to GPS, and publish as sensor_msgs::NavSatFix
  void filteredOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (!initialized_) {
      ROS_WARN_THROTTLE(5, "Reference point not initialized, cannot convert ENU to LLA.");
      return;
    }
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;

    double lat, lon, alt;
    geo_trans_->enu2lla(x, y, z, lat, lon, alt);

    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header = msg->header;
    gps_msg.header.frame_id = "map";
    gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    gps_msg.latitude = lat;
    gps_msg.longitude = lon;
    gps_msg.altitude = alt;

    gps_msg.position_covariance[0] = 250;
    gps_msg.position_covariance[4] = 250;
    gps_msg.position_covariance[8] = 10;

    filtered_gps_pub_.publish(gps_msg);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::Subscriber gps_sub_;
  ros::Publisher odom_pub_;

  // Subscriber and publisher for filtered data
  ros::Subscriber filtered_odom_sub_;
  ros::Publisher filtered_gps_pub_;

  std::shared_ptr<geo_transform::GeoTransform> geo_trans_;
  bool initialized_;
  double ref_lat_, ref_lon_, ref_alt_;
  double accumulate_time;
  double tau;

  double last_gps_time;
  double current_gps_time;
  int accumulate_times;

  double last_gps_x;
  double last_gps_y;

  double current_gps_x;
  double current_gps_y;
  std::vector<bool> outlier_flag_list;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gps_enu_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  GPSToENUNode node(nh, nh_priv);

  ros::spin();
  return 0;
}
