#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float64.h>
#include <thread>
#include <chrono>
#include <mutex>

#include <cmath>

#include <geo_transform/geo_transform.h>

class PathPublisher
{
public:
    PathPublisher()
    {
        // Initialize publishers
        filtered_odom_global_path_pub_ = nh_.advertise<nav_msgs::Path>("filterd_path_global", 100);
        filtered_odom_local_path_pub_ = nh_.advertise<nav_msgs::Path>("filterd_path_local", 100);
        filtered_odom_local_abs_path_pub_ = nh_.advertise<nav_msgs::Path>("filterd_path_local_abs", 100);
        raw_gps_path_pub_ = nh_.advertise<nav_msgs::Path>("raw_gps_path", 100);
        ground_truth_path_pub_ = nh_.advertise<nav_msgs::Path>("kml_path", 100);
        vio_path_pub_ = nh_.advertise<nav_msgs::Path>("vio_path", 10);
        ground_truth_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("ground_truth_markers", 1000);
        filtered_position_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("filtered_position_markers", 1000);
        raw_gps_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("gps_raw_markers", 1000);


        fusion_error_pub = nh_.advertise<std_msgs::Float64>("global_fusion_error", 10);
        fusion_error_relative_pub = nh_.advertise<std_msgs::Float64>("local_fusion_error_relative", 10);
        vio_error_relative_pub = nh_.advertise<std_msgs::Float64>("ARKit_error", 10);
        gps_error_relative_pub = nh_.advertise<std_msgs::Float64>("gps_error", 10);
        gps_x_relative_pub = nh_.advertise<std_msgs::Float64>("gps_x", 10);
        gps_y_relative_pub = nh_.advertise<std_msgs::Float64>("gps_y", 10);
        vio_x_relative_pub = nh_.advertise<std_msgs::Float64>("vio_x", 10);
        vio_y_relative_pub = nh_.advertise<std_msgs::Float64>("vio_y", 10);
        fusion_x_relative_pub = nh_.advertise<std_msgs::Float64>("fusion_x", 10);
        fusion_y_relative_pub = nh_.advertise<std_msgs::Float64>("fusion_y", 10);
        //

        // Initialize subscribers
        filtered_odom_global_sub_ = nh_.subscribe("odometry/filtered", 10, &PathPublisher::filtered_odom_global_Callback, this);
        filtered_odom_local_sub_ = nh_.subscribe("odometry/filtered_", 10, &PathPublisher::filtered_odom_local_Callback, this);
        raw_gps_sub_ = nh_.subscribe("gps_data", 10, &PathPublisher::raw_gps_Callback, this);
        kml_path_raw_sub_ = nh_.subscribe("kml_path_interpolation", 10, &PathPublisher::kml_path_raw_Callback, this);
        vio_sub_ = nh_.subscribe("vio_odom", 10, &PathPublisher::vio_path_Callback, this);

        // Initialize Path headers
        filtered_odom_global_path_.header.frame_id = "map";  
        filtered_odom_local_path_.header.frame_id = "map";  
        filtered_odom_local_abs_path_.header.frame_id = "map";  
        raw_gps_path_.header.frame_id = "map";
        ground_truth_path_.header.frame_id = "map";
        vio_path_.header.frame_id = "map";
        eva_path_.header.frame_id = "map";
        kml_path_raw_.header.frame_id = "map";

        // Initialize flags and variables
        reference_set_ = false;
        ground_truth_initial_ = false;
        kml_path_raw_initial_ = false;
        marker_id_ = 0;
        marker_filtered_id_ = 0;
        marker_gps_id_ = 0;
        kml_publishing_started_ = false;
        initial_transfer_flag_ = false;
        kml_current_pose_index_ = 0;


        // erroe
        fusion_error = 0;
        fusion_error_average = 0;
        fusion_error_sum = 0;
        fusion_error_relative = 0;
        fusion_error_relative_average = 0;
        fusion_error_relative_sum = 0;
        vio_error_relative = 0;
        vio_error_relative_average = 0;
        vio_error_relative_sum = 0;
        gps_error_relative = 0;
        gps_error_relative_average = 0;
        gps_error_relative_sum = 0;



        // Initialize ROS Timer for publishing markers
        // Timer runs at 100 Hz
        timer_ = nh_.createTimer(ros::Duration(0.01), &PathPublisher::timerCallback, this);



    }


    void filtered_odom_global_Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        filtered_odom_global_path_.header.stamp = msg->header.stamp;
        filtered_odom_global_path_.poses.push_back(pose);
        filtered_odom_global_path_pub_.publish(filtered_odom_global_path_);
    }

    void filtered_odom_local_Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        filtered_odom_local_path_.header.stamp = msg->header.stamp;
        filtered_odom_local_path_.poses.push_back(pose);
        filtered_odom_local_path_pub_.publish(filtered_odom_local_path_);

        if (initial_transfer_flag_){

            
            geometry_msgs::PoseStamped pose;
            pose.header = msg->header;
            pose.pose.position.x = msg->pose.pose.position.x + initial_pose_.pose.position.x;
            pose.pose.position.y = msg->pose.pose.position.y + initial_pose_.pose.position.y;
            pose.pose.position.z = msg->pose.pose.position.z + initial_pose_.pose.position.z;

            filtered_odom_local_abs_path_.header.stamp = msg->header.stamp;
            filtered_odom_local_abs_path_.poses.push_back(pose);       
            filtered_odom_local_abs_path_pub_.publish(filtered_odom_local_abs_path_);
        }

    }

    void raw_gps_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
    {
        if (!reference_set_)
        {
            ref_lat_ = msg->latitude;
            ref_lon_ = msg->longitude;
            ref_alt_ = msg->altitude;
            geo_trans_ = std::make_shared<geo_transform::GeoTransform>(ref_lat_, ref_lon_, ref_alt_);
            reference_set_ = true;


            
            // Check if kml_path_raw_ is already received to start publishing
            if (kml_path_raw_initial_ && !kml_publishing_started_)
            {
                initializeKmlPublishing();
            }
        }

        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.header.stamp = ros::Time::now();

        double e, n, u;
        geo_trans_->lla2enu(msg->latitude, msg->longitude, msg->altitude, e, n, u);
        pose.pose.position.x = e;
        pose.pose.position.y = n;
        pose.pose.position.z = 0; 

        publish_raw_gps_Marker(e, n, 0, marker_gps_id_);
        marker_gps_id_++;

        raw_gps_path_.header.stamp = msg->header.stamp;
        raw_gps_path_.poses.push_back(pose);
        raw_gps_path_pub_.publish(raw_gps_path_);   



         
    }

    void kml_path_raw_Callback(const nav_msgs::Path::ConstPtr& msg)
    {
        if (!kml_path_raw_initial_)
        {
            kml_path_raw_ = *msg;
            ROS_INFO_STREAM("Received kml_path_raw with " << kml_path_raw_.poses.size() << " poses.");
            kml_path_raw_initial_ = true;

            

            // Check if reference is already set to start publishing
            if (reference_set_ && !kml_publishing_started_)
            {
                initializeKmlPublishing();
            }
        }


    }

    void vio_path_Callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;

        vio_path_.header.stamp = msg->header.stamp;
        vio_path_.poses.push_back(pose);
        vio_path_pub_.publish(vio_path_);       
    }



private:
    ros::NodeHandle nh_;

    // Publishers
    ros::Publisher filtered_odom_global_path_pub_;
    ros::Publisher filtered_odom_local_path_pub_;
    ros::Publisher filtered_odom_local_abs_path_pub_;
    ros::Publisher raw_gps_path_pub_;
    ros::Publisher ground_truth_path_pub_;
    ros::Publisher vio_path_pub_;
    ros::Publisher ground_truth_marker_pub_;
    ros::Publisher filtered_position_marker_pub_;
    ros::Publisher raw_gps_marker_pub_;

    ros::Publisher fusion_error_pub;
    ros::Publisher fusion_error_relative_pub;
    ros::Publisher vio_error_relative_pub;
    ros::Publisher gps_error_relative_pub;
    ros::Publisher gps_x_relative_pub;
    ros::Publisher gps_y_relative_pub;
    ros::Publisher vio_x_relative_pub;
    ros::Publisher vio_y_relative_pub;
    ros::Publisher fusion_x_relative_pub;
    ros::Publisher fusion_y_relative_pub;

    std::thread publish_thread;
    std::mutex data_mutex;
    

    // Subscribers
    ros::Subscriber filtered_odom_global_sub_;
    ros::Subscriber filtered_odom_local_sub_;
    ros::Subscriber raw_gps_sub_;
    ros::Subscriber ground_truth_raw_sub_;
    ros::Subscriber vio_sub_;
    ros::Subscriber kml_path_raw_sub_;

    // Paths
    nav_msgs::Path filtered_odom_global_path_;
    nav_msgs::Path filtered_odom_local_path_;
    nav_msgs::Path filtered_odom_local_abs_path_;
    nav_msgs::Path raw_gps_path_;
    nav_msgs::Path ground_truth_path_;
    nav_msgs::Path eva_path_;
    nav_msgs::Path vio_path_;
    nav_msgs::Path kml_path_raw_;

    geometry_msgs::PoseStamped initial_pose_;


    // Reference coordinates
    double reference_latitude_;
    double reference_longitude_;
    double reference_altitude_;
    bool reference_set_;
    bool ground_truth_initial_;
    bool kml_path_raw_initial_;
    size_t marker_id_;
    size_t marker_filtered_id_;
    size_t marker_gps_id_;


    bool initial_transfer_flag_;


    double fusion_error;
    std::vector<double> fusion_error_all;
    double fusion_error_average;
    int fusion_error_sum;

    double fusion_error_relative;
    std::vector<double> fusion_error_relative_all;
    double fusion_error_relative_average;
    int fusion_error_relative_sum;

    double vio_error_relative;
    std::vector<double> vio_error_relative_all;
    double vio_error_relative_average;
    int vio_error_relative_sum; 


    std::vector<double> fusion_and_vio_error_time;  

    double gps_error_relative;
    std::vector<double> gps_error_relative_all;
    double gps_error_relative_average;
    int gps_error_relative_sum; 

    std::vector<double> gps_error_time;  



    // Geo transformation
    std::shared_ptr<geo_transform::GeoTransform> geo_trans_;
    bool initialized_;
    double ref_lat_, ref_lon_, ref_alt_;

    // Timer for publishing kml_path_raw_ markers
    ros::Timer timer_;

    // Variables for kml_path_raw_ publishing
    bool kml_publishing_started_;
    size_t kml_current_pose_index_;
    ros::Time kml_start_ros_time_;
    ros::Time kml_first_pose_time_;

    // Mutex for thread safety (optional, if needed)
    // std::mutex mutex_;

    void publish_gt_Marker(double e, double n, double u, bool is_initial, size_t id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "ground_truth_markers";
        marker.id = id; // Unique ID

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = e;
        marker.pose.position.y = n;
        marker.pose.position.z = u; // 0 indicates ground level

        marker.pose.orientation.w = 1.0;

        if (is_initial)
        {
            // Initial point color and size
            marker.scale.x = 3.5;
            marker.scale.y = 3.5;
            marker.scale.z = 3.5;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else
        {
            // Subsequent points color and size
            marker.scale.x = 1.5;
            marker.scale.y = 1.5;
            marker.scale.z = 1.5;
            // marker.scale.x = 3.5;
            // marker.scale.y = 3.5;
            // marker.scale.z = 3.5;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        marker.lifetime = ros::Duration(0); // Forever

        // Publish Marker
        ground_truth_marker_pub_.publish(marker);
    }

    void publish_filtered_position_Marker(double e, double n, double u, size_t id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "filtered_position_truth_markers";
        marker.id = id; // Unique ID

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = e;
        marker.pose.position.y = n;
        marker.pose.position.z = u; // 0 indicates ground level

        marker.pose.orientation.w = 1.0;


        
        marker.scale.x = 2.2;
        marker.scale.y = 2.2;
        marker.scale.z = 2.2;
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        marker.lifetime = ros::Duration(0); // Forever

        // Publish Marker
        filtered_position_marker_pub_.publish(marker);
    }

    void publish_raw_gps_Marker(double e, double n, double u, size_t id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "raw_gps_markers";
        marker.id = id; // Unique ID

        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = e;
        marker.pose.position.y = n;
        marker.pose.position.z = u; // 0 indicates ground level

        marker.pose.orientation.w = 1.0;


        
        marker.scale.x = 1.5;
        marker.scale.y = 1.5;
        marker.scale.z = 1.5;
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;  // 确保颜色不透明

        marker.lifetime = ros::Duration(0); // Forever

        // Publish Marker
        raw_gps_marker_pub_.publish(marker);
    }

    void initializeKmlPublishing()
    {
        if (kml_path_raw_.poses.empty())
        {
            ROS_WARN("kml_path_raw_ is empty. No markers to publish.");
            return;
        }

        kml_current_pose_index_ = 0;
        kml_first_pose_time_ = kml_path_raw_.poses[0].header.stamp;
        kml_start_ros_time_ = ros::Time::now();
        kml_publishing_started_ = true;

        ROS_INFO("Initialized KML Path publishing.");
    }

    void timerCallback(const ros::TimerEvent& event)
    {
        // Check if publishing has started and there are poses to publish
        if (kml_publishing_started_ && kml_current_pose_index_ < kml_path_raw_.poses.size())
        {
            const auto& pose_stamped = kml_path_raw_.poses[kml_current_pose_index_];
            ros::Duration time_since_start = ros::Time::now() - kml_start_ros_time_;
            ros::Duration pose_time = pose_stamped.header.stamp - kml_first_pose_time_;

            if (time_since_start >= pose_time)
            {
                // Convert LLA to ENU
                double e=0, n=0, u=0;
                geo_trans_->lla2enu(pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z, e, n, u);

                // Determine if this is the initial pose
                if (kml_current_pose_index_ == 0){
                    initial_pose_.pose.position.x = e;
                    initial_pose_.pose.position.y = n;
                    initial_pose_.pose.position.z = u;
                    initial_transfer_flag_ = true;

                }
                bool is_initial = (kml_current_pose_index_ == 0);
                

                // Publish the marker
                publish_gt_Marker(e, n, u, is_initial, marker_id_++);

                
               
                

                if (kml_current_pose_index_!=0){

                    const geometry_msgs::PoseStamped& last_pose = filtered_odom_global_path_.poses.back();
                    double x = last_pose.pose.position.x;
                    double y = last_pose.pose.position.y;
                    double z = last_pose.pose.position.z;
                    fusion_error_sum ++;
                    fusion_error = fusion_error + sqrt(pow((e-x),2)+ pow((n-y),2));
                    fusion_error_average = fusion_error / fusion_error_sum;
                    fusion_error_all.push_back(sqrt(pow((e-x),2)+ pow((n-y),2)));
                    std_msgs::Float64 fusion_error_msg;
                    std_msgs::Float64 fusion_x_relative_msg;
                    std_msgs::Float64 fusion_y_relative_msg;
                    fusion_error_msg.data = sqrt(pow((e-x),2)+ pow((n-y),2));
                    fusion_error_pub.publish(fusion_error_msg);
                    fusion_x_relative_msg.data = e-x;
                    fusion_y_relative_msg.data = n-y;
                    fusion_x_relative_pub.publish(fusion_x_relative_msg);
                    fusion_y_relative_pub.publish(fusion_y_relative_msg);
                    ROS_INFO("fusion error: %f", fusion_error_average);
                    publish_filtered_position_Marker(x,y,z, marker_filtered_id_++);


                    const geometry_msgs::PoseStamped& last_pose0 = filtered_odom_local_abs_path_.poses.back();
                    x = last_pose0.pose.position.x;
                    y = last_pose0.pose.position.y;
                    z = last_pose0.pose.position.z;
                    fusion_error_relative_sum ++;
                    fusion_error_relative = fusion_error_relative + sqrt(pow((e-x),2)+ pow((n-y),2));
                    fusion_error_relative_average = fusion_error_relative / fusion_error_relative_sum;
                    fusion_error_relative_all.push_back(sqrt(pow((e-x),2)+ pow((n-y),2)));
                    std_msgs::Float64 fusion_error_relative_msg;
                    fusion_error_relative_msg.data = sqrt(pow((e-x),2)+ pow((n-y),2));
                    fusion_error_relative_pub.publish(fusion_error_relative_msg);
                    ROS_INFO("fusion error relative: %f", fusion_error_relative_average);



                    const geometry_msgs::PoseStamped& last_pose1 = vio_path_.poses.back();
                    x = last_pose1.pose.position.x;
                    y = last_pose1.pose.position.y;
                    z = last_pose1.pose.position.z;
                    vio_error_relative_sum ++;
                    vio_error_relative = vio_error_relative + sqrt(pow((e-x),2)+ pow((n-y),2));
                    vio_error_relative_average = vio_error_relative / vio_error_relative_sum;
                    vio_error_relative_all.push_back(sqrt(pow((e-x),2)+ pow((n-y),2)));
                    std_msgs::Float64 vio_error_relative_msg;
                    std_msgs::Float64 vio_x_relative_msg;
                    std_msgs::Float64 vio_y_relative_msg;
                    vio_error_relative_msg.data = sqrt(pow((e-x),2)+ pow((n-y),2));
                    vio_error_relative_pub.publish(vio_error_relative_msg);
                        
                    vio_x_relative_msg.data = e-x;
                    vio_y_relative_msg.data = n-y;
                    vio_x_relative_pub.publish(vio_x_relative_msg);
                    vio_y_relative_pub.publish(vio_y_relative_msg);
                    
                    ROS_INFO("vio error: %f", vio_error_relative_average);


                    fusion_and_vio_error_time.push_back(ros::Time().now().toSec());




                    
                    const geometry_msgs::PoseStamped& last_pose2 = raw_gps_path_.poses.back();
                    ros::Duration time_diff = ros::Time::now() - last_pose2.header.stamp;
                    double time_diff_sec = time_diff.toSec();
                    
                    if (std::abs(time_diff_sec) < 1.0){
                        double x,y,z;
                        x = last_pose2.pose.position.x;
                        y = last_pose2.pose.position.y;
                        z = last_pose2.pose.position.z;



                        gps_error_relative_sum ++;
                        gps_error_relative = gps_error_relative + sqrt(pow((e-x),2)+ pow((n-y),2));
                        gps_error_relative_average = gps_error_relative / gps_error_relative_sum;
                        gps_error_relative_all.push_back(sqrt(pow((e-x),2)+ pow((n-y),2)));
                        ROS_INFO("gps error: %f", gps_error_relative_average);

                        gps_error_time.push_back(ros::Time().now().toSec());

                        std_msgs::Float64 gps_error_relative_msg;
                        std_msgs::Float64 gps_x_relative_msg;
                        std_msgs::Float64 gps_y_relative_msg;
                   
                        gps_error_relative_msg.data = sqrt(pow((e-x),2)+ pow((n-y),2));
                        gps_x_relative_msg.data = e-x;
                        gps_y_relative_msg.data = n-y;
                        gps_error_relative_pub.publish(gps_error_relative_msg);
                        gps_x_relative_pub.publish(gps_x_relative_msg);
                        gps_y_relative_pub.publish(gps_y_relative_msg);
                    

                    }

                    
                    
                }

                kml_current_pose_index_++;
            }
        }
    }

    void PlotFigure(){

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_publish");
    PathPublisher path_publisher;
    ros::spin();
    return 0;
}
