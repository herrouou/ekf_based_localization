#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>

class PathPublisher
{
public:
    PathPublisher()
    {
        
        filtered_odom_path_pub_ = nh_.advertise<nav_msgs::Path>("filterd_path", 10);
        raw_gps_path_pub_ = nh_.advertise<nav_msgs::Path>("raw_gps_path", 10);
        filtered_gps_path_pub_ = nh_.advertise<nav_msgs::Path>("filtered_gps_path", 10);
        kml_path_pub_ = nh_.advertise<nav_msgs::Path>("kml_path", 10);
        vio_path_pub_ = nh_.advertise<nav_msgs::Path>("vio_path", 10);
        


        filtered_odom_sub_ = nh_.subscribe("odometry/filtered", 10, &PathPublisher::filtered_odom_Callback, this);
        raw_gps_sub_ = nh_.subscribe("gps_data", 10, &PathPublisher::raw_gps_Callback, this);
        filtered_gps_sub_ = nh_.subscribe("gps/filtered", 10, &PathPublisher::filtered_gps_Callback, this);
        kml_path_raw_sub_ = nh_.subscribe("/kml_visual/kml_path_raw", 10, &PathPublisher::kml_path_Callback, this);
        vio_sub_ = nh_.subscribe("vio_odom", 10, &PathPublisher::vio_path_Callback, this);

        filtered_odom_path_.header.frame_id = "map";  
        raw_gps_path_.header.frame_id = "map";
        filtered_gps_path_.header.frame_id = "map";
        kml_path_.header.frame_id = "map";
        vio_path_.header.frame_id = "map";

        reference_set_ = false;
    }

    void filtered_odom_Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        filtered_odom_path_.header.stamp = msg->header.stamp;
        filtered_odom_path_.poses.push_back(pose);
        filtered_odom_path_pub_.publish(filtered_odom_path_);
    }

    void raw_gps_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg){

        if (!reference_set_)
        {
            return;
        }

        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = longitudeToMeters(msg->longitude, reference_longitude_, reference_latitude_);
        pose.pose.position.y = latitudeToMeters(msg->latitude, reference_latitude_);
        pose.pose.position.z = 0; // msg->altitude - reference_altitude_;

        raw_gps_path_.header.stamp = msg->header.stamp;
        raw_gps_path_.poses.push_back(pose);
        raw_gps_path_pub_.publish(raw_gps_path_);        
    }

    void filtered_gps_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg){

        static bool kml_recieve = false;
        if(!reference_set_){
            return;
        }    

            
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = longitudeToMeters(msg->longitude, reference_longitude_, reference_latitude_);
        pose.pose.position.y = latitudeToMeters(msg->latitude, reference_latitude_);
        pose.pose.position.z = 0; // msg->altitude - reference_altitude_;


        filtered_gps_path_.header.stamp = msg->header.stamp;
        filtered_gps_path_.poses.push_back(pose);
        filtered_gps_path_pub_.publish(filtered_gps_path_);  
    }

    void kml_path_Callback(const nav_msgs::Path::ConstPtr& msg){
        
        if (kml_path_processed_)
        {
            return;
        }

        if(!reference_set_){
            reference_latitude_ = msg->poses[0].pose.position.y;
            reference_longitude_ = msg->poses[0].pose.position.x;
            reference_altitude_ = msg->poses[0].pose.position.z;
            reference_set_ = true;
        }

        
        kml_path_.poses.clear();

        for (const auto& pose : msg->poses)
        {
            geometry_msgs::PoseStamped transformed_pose;
            transformed_pose.header = pose.header;

            double kml_lon = pose.pose.position.x;
            double kml_lat = pose.pose.position.y;
            double kml_alt = pose.pose.position.z;

            // 转换经纬度为相对参考点的平面坐标
            transformed_pose.pose.position.x = longitudeToMeters(kml_lon, reference_longitude_, reference_latitude_);
            transformed_pose.pose.position.y = latitudeToMeters(kml_lat, reference_latitude_);
            transformed_pose.pose.position.z = 0; //kml_alt - reference_altitude_;

            // 保留原来的姿态
            transformed_pose.pose.orientation = pose.pose.orientation;

            kml_path_.poses.push_back(transformed_pose);
        }

        
        kml_path_.header.stamp = ros::Time::now();

        kml_path_processed_ = true;            
    }

    void vio_path_Callback(const nav_msgs::Odometry::ConstPtr& msg){
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        vio_path_.header.stamp = msg->header.stamp;
        vio_path_.poses.push_back(pose);
        vio_path_pub_.publish(vio_path_);       
    }

    void publishPaths()
    {

        if (kml_path_processed_)
        {
            kml_path_pub_.publish(kml_path_);
        }
    }

private:
    ros::NodeHandle nh_;

    ros::Publisher filtered_odom_path_pub_;
    ros::Publisher raw_gps_path_pub_;
    ros::Publisher filtered_gps_path_pub_;
    ros::Publisher kml_path_pub_;
    ros::Publisher vio_path_pub_;

    ros::Subscriber filtered_odom_sub_;
    ros::Subscriber raw_gps_sub_;
    ros::Subscriber filtered_gps_sub_;
    ros::Subscriber kml_path_raw_sub_;
    ros::Subscriber vio_sub_;

    nav_msgs::Path filtered_odom_path_;
    nav_msgs::Path raw_gps_path_;
    nav_msgs::Path filtered_gps_path_;
    nav_msgs::Path kml_path_;
    nav_msgs::Path vio_path_;

    double reference_latitude_;
    double reference_longitude_;
    double reference_altitude_;
    bool reference_set_;
    bool kml_path_processed_ = false;

    // Conversion functions
    double latitudeToMeters(double lat, double ref_lat)
    {
        const double earth_radius = 6378137.0; // in meters
        return (lat - ref_lat) * M_PI / 180.0 * earth_radius;
    }

    double longitudeToMeters(double lon, double ref_lon, double ref_lat)
    {
        const double earth_radius = 6378137.0; // in meters
        return (lon - ref_lon) * M_PI / 180.0 * earth_radius * cos(ref_lat * M_PI / 180.0);
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publish");
    PathPublisher path_publisher;

    ros::Rate loop_rate(5); 

    while (ros::ok())
    {
        path_publisher.publishPaths(); 
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
