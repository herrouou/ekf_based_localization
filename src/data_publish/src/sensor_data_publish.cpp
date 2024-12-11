#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <boost/array.hpp>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>




struct ImuData{
    double timestamp;
    float x, y, z, w;
    float w_x, w_y, w_z;
    float a_x, a_y, a_z;
    // float mag_x, mag_y, mag_z;
};

struct GpsData{
    double timestamp;
    double latitude, longitude, altitude;
    float horacc, veracc;
};

struct VIOdata{
    double timestamp;
    int frame;
    float px, py, pz;
    float x, y, z, w;
};

template<typename T>
size_t findClosestTimeIndex(const std::vector<T>& data, double target_time) {
    auto it = std::min_element(data.begin(), data.end(), [target_time](const T& a, const T& b) {
        return std::abs(a.timestamp - target_time) < std::abs(b.timestamp - target_time);
    });

    size_t closest_index = std::distance(data.begin(), it);

    // 计算差值并打印
    double time_difference = std::abs(it->timestamp - target_time);
    ROS_INFO("Target time: %.6f, Closest time: %.6f, Difference: %.6f", target_time, it->timestamp, time_difference);

    return std::distance(data.begin(), it);
}



// multiple
std::array<float, 4> quaternionMultiply(const std::array<float, 4>& q1, const std::array<float, 4>& q2) {
    return {
        q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1],
        q1[3] * q2[1] + q1[1] * q2[3] + q1[2] * q2[0] - q1[0] * q2[2],
        q1[3] * q2[2] + q1[2] * q2[3] + q1[0] * q2[1] - q1[1] * q2[0],
        q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
    };
}

void quaternionMultiply0(float w1, float x1, float y1, float z1,
                        float w2, float x2, float y2, float z2,
                        float &w, float &x, float &y, float &z) {
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}

// 
std::array<float, 3> rotateVectorByQuaternion(const std::array<float, 3>& vec, const std::array<float, 4>& quat) {
    std::array<float, 4> vecQuat = {vec[0], vec[1], vec[2], 0};  
    std::array<float, 4> quatConjugate = {-quat[0], -quat[1], -quat[2], quat[3]};  

    // v' = q * v * q^(-1)
    auto resultQuat = quaternionMultiply(quaternionMultiply(quat, vecQuat), quatConjugate);
    return {resultQuat[0], resultQuat[1], resultQuat[2]};
}




std::vector<ImuData> readImu(const std::string& filename){
    std::vector<ImuData> imu_data;
    std::fstream file(filename);
    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)){
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

        std::stringstream ss(line);
        std::string field;
        ImuData data;
        

        std::getline(ss, field, ',');
        data.timestamp = std::stod(field);

        
        std::getline(ss, field, ',');
        data.a_x = std::stof(field);
        std::getline(ss, field, ',');
        data.a_y = std::stof(field);
        std::getline(ss, field, ',');
        data.a_z = std::stof(field);



        std::getline(ss, field, ',');
        data.w_x = std::stof(field);                
        std::getline(ss, field, ',');
        data.w_y = std::stof(field);
        std::getline(ss, field, ',');
        data.w_z = std::stof(field);

        float x, y, z, w;
        std::getline(ss, field, ',');
        x = std::stof(field);                
        std::getline(ss, field, ',');
        y = std::stof(field);
        std::getline(ss, field, ',');
        z = std::stof(field);        
        std::getline(ss, field, ',');
        w = std::stof(field);  

        // 定义旋转四元数 (绕 Z 轴逆时针旋转 90°)
        float angle_degrees = 90.0f;
        float angle_radians = angle_degrees * M_PI / 180.0f;
        float sin_half = std::sin(angle_radians / 2.0f); // sin(45°) = √2/2
        float cos_half = std::cos(angle_radians / 2.0f); // cos(45°) = √2/2

        float rx = 0, ry = 0, rz = sin_half, rw = cos_half;

        // 旋转四元数的共轭
        float rx_conj = -rx, ry_conj = -ry, rz_conj = -rz, rw_conj = rw;

        // 计算 q_mid = q_rotation * q_A
        float mid_x = rw * x + rx * w + ry * z - rz * y;
        float mid_y = rw * y - rx * z + ry * w + rz * x;
        float mid_z = rw * z + rx * y - ry * x + rz * w;
        float mid_w = rw * w - rx * x - ry * y - rz * z;

        // 计算 q_B = q_mid * q_rotation⁻¹
        data.x = mid_w * rx_conj + mid_x * rw_conj + mid_y * rz_conj - mid_z * ry_conj;
        data.y = mid_w * ry_conj - mid_x * rz_conj + mid_y * rw_conj + mid_z * rx_conj;
        data.z = mid_w * rz_conj + mid_x * ry_conj - mid_y * rx_conj + mid_z * rw_conj;
        data.w = mid_w * rw_conj - mid_x * rx_conj - mid_y * ry_conj - mid_z * rz_conj;

        // float yy = data.y;
        // data.y = data.x;
        // data.x = yy;
        // data.z = -data.z;
        std::array<float, 4> quat = {data.x, data.y, data.z, data.w};

        
        std::array<float, 3> accel = {data.a_x, data.a_y, data.a_z};
        std::array<float, 3> gyro = {data.w_x, data.w_y, data.w_z};
        std::array<float, 4> quatInverse = {-quat[0], -quat[1], -quat[2], quat[3]};

        auto rotatedAccel = rotateVectorByQuaternion(accel, quatInverse);
        auto rotatedGyro = rotateVectorByQuaternion(gyro, quatInverse);

        
        data.a_x = rotatedAccel[0];
        data.a_y = rotatedAccel[1];
        data.a_z = -rotatedAccel[2];

        data.w_x = rotatedGyro[0];
        data.w_y = rotatedGyro[1];
        data.w_z = rotatedGyro[2];
        imu_data.push_back(data);


    }
    return imu_data;

}

std::vector<GpsData> readGps(const std::string& filename){
    
    std::vector<GpsData> gps_data;
    std::fstream file(filename);
    std::string line;
    std::getline(file, line);

    while(std::getline(file, line)){
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

        std::stringstream ss(line);
        std::string field;
        GpsData data;

        std::getline(ss, field, ',');
        data.timestamp = std::stod(field);
        std::getline(ss, field, ',');
        data.latitude = std::stof(field);
        std::getline(ss, field, ',');
        data.longitude = std::stof(field);
        std::getline(ss, field, ',');
        data.altitude = std::stof(field);
        std::getline(ss, field, ',');
        data.horacc = std::stof(field);
        std::getline(ss, field, ',');
        data.veracc = std::stof(field);
        gps_data.push_back(data);
    }
    return gps_data;

}

std::vector<VIOdata> readVIO(const std::string& filename){
    std::vector<VIOdata> vio_data;
    std::fstream file(filename);
    std::string line;
    std::getline(file, line); 

    while(std::getline(file, line)){
        line.erase(std::remove(line.begin(), line.end(), ' '), line.end());

        std::stringstream ss(line);
        std::string field;
        VIOdata data;

        float x,y,z,w,px,py,pz;
        std::getline(ss, field, ',');
        data.timestamp = std::stod(field);
        std::getline(ss, field, ',');
        data.frame = std::stoi(field);
        std::getline(ss, field, ',');
        px = std::stof(field);
        std::getline(ss, field, ',');
        py = std::stof(field);
        std::getline(ss, field, ',');
        pz = std::stof(field);
        std::getline(ss, field, ',');
        x = std::stof(field);
        std::getline(ss, field, ',');
        y = std::stof(field);
        std::getline(ss, field, ',');
        z = std::stof(field);
        std::getline(ss, field, ',');
        w = std::stof(field);


        // A到C的四元数
        tf2::Quaternion quat_A_to_C(x, y, z, w);

        // C到D的旋转
        tf2::Quaternion quat_C_to_D;
        tf2::Quaternion rotation_y, rotation_z;
        rotation_y.setRPY(0, tf2Radians(180), 0);     // Y轴旋转180°
        rotation_z.setRPY(0, 0, tf2Radians(-90));    // Z轴旋转-90°
        quat_C_to_D = rotation_y * rotation_z;       // 复合旋转：先Y后Z

        // B到A的旋转
        tf2::Quaternion quat_B_to_A;
        quat_B_to_A.setRPY(tf2Radians(90), 0, 0);  // 使用Roll=0, Pitch=90°, Yaw=0

        // 计算B到D的旋转：C到D * A到C * B到A
        tf2::Quaternion quat_B_to_D = quat_B_to_A * quat_A_to_C * quat_C_to_D  ; 

        // 将结果赋值给data
        data.x = quat_B_to_D.x();
        data.y = quat_B_to_D.y();
        data.z = quat_B_to_D.z();
        data.w = quat_B_to_D.w();

        // data.x = x;
        // data.y = y;
        // data.z = z;
        // data.w = w;


        data.px = px;
        data.py = -pz;
        data.pz = py;

        vio_data.push_back(data);
    }
    return vio_data;       
}



void publishData (const std::vector<ImuData>& imu_data, const std::vector<GpsData>& gps_data,
                  const std::vector<VIOdata>& vio_data){
    ros::NodeHandle nh;
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 10);
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("gps_data", 10);
    ros::Publisher vio_pub = nh.advertise<nav_msgs::Odometry>("vio_odom", 10);
    ros::Publisher imu_orientation_pub = nh.advertise<nav_msgs::Odometry>("imu_orientation", 10);

    size_t imu_index = 0, gps_index = 0, vio_index = 0;
    size_t imu_data_size = imu_data.size();
    size_t gps_data_size = gps_data.size();
    size_t vio_data_size = vio_data.size();

    double imu_start_time = imu_data[0].timestamp;
    double gps_start_time = gps_data[0].timestamp;
    double vio_start_time = vio_data[0].timestamp;
    double global_start_time = std::min({imu_start_time, gps_start_time, vio_start_time});

    

    double ros_start_time = ros::Time::now().toSec();

    while(ros::ok() && (imu_index < imu_data_size || gps_index < gps_data_size || vio_index < vio_data_size)){
        double current_time = ros::Time().now().toSec();



        double imu_wait = imu_index < imu_data_size ? (imu_data[imu_index].timestamp - global_start_time) - (current_time - ros_start_time) : std::numeric_limits<double>::max();
        double gps_wait = gps_index < gps_data_size ? (gps_data[gps_index].timestamp - global_start_time) - (current_time - ros_start_time) : std::numeric_limits<double>::max();
        double vio_wait = vio_index < vio_data_size ? (vio_data[vio_index].timestamp - global_start_time) - (current_time - ros_start_time) : std::numeric_limits<double>::max();
        if (imu_index < imu_data_size && imu_wait <= 0){
            sensor_msgs::Imu imu_msg;
            nav_msgs::Odometry orientation;

            imu_msg.header.stamp = ros::Time(imu_data[imu_index].timestamp);
            // imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu_link";
            
            
            // transfer the imu_data from  to ENU
            imu_msg.orientation.x = imu_data[imu_index].x;
            imu_msg.orientation.y = imu_data[imu_index].y;
            imu_msg.orientation.z = imu_data[imu_index].z;
            imu_msg.orientation.w = imu_data[imu_index].w;

            

            orientation.header.frame_id = "odom";
            orientation.child_frame_id = "base_link";
            orientation.pose.pose.orientation.x = imu_data[imu_index].x;
            orientation.pose.pose.orientation.y = imu_data[imu_index].y;
            orientation.pose.pose.orientation.z = imu_data[imu_index].z;
            orientation.pose.pose.orientation.w = imu_data[imu_index].w;



            imu_msg.orientation_covariance[0] = 0.05;
            imu_msg.orientation_covariance[0] = 0.05;
            imu_msg.orientation_covariance[4] = 0.05;
            imu_msg.orientation_covariance[8] = 0.05;

            imu_msg.angular_velocity.x = imu_data[imu_index].w_x;
            imu_msg.angular_velocity.y = imu_data[imu_index].w_y;
            imu_msg.angular_velocity.z = imu_data[imu_index].w_z;
            
            imu_msg.angular_velocity_covariance[0] = 0.05;
            imu_msg.angular_velocity_covariance[4] = 0.05;
            imu_msg.angular_velocity_covariance[8] = 0.05;
            
            imu_msg.linear_acceleration.x = imu_data[imu_index].a_x;
            imu_msg.linear_acceleration.y = imu_data[imu_index].a_y;
            imu_msg.linear_acceleration.z = imu_data[imu_index].a_z;
            
            imu_msg.linear_acceleration_covariance[0] = 0.025;
            imu_msg.linear_acceleration_covariance[4] = 0.025;
            imu_msg.linear_acceleration_covariance[8] = 0.025;



            imu_pub.publish(imu_msg);
            // imu_orientation_pub.publish(orientation);

            imu_index++;
            
        } 

        if (gps_index < gps_data_size && gps_wait <=0){
            sensor_msgs::NavSatFix gps_msg; 

            gps_msg.header.stamp = ros::Time(gps_data[gps_index].timestamp);

            gps_msg.header.frame_id = "base_link";
            gps_msg.altitude = gps_data[gps_index].altitude;
            gps_msg.longitude = gps_data[gps_index].longitude;
            gps_msg.latitude = gps_data[gps_index].latitude;
            gps_msg.position_covariance[0] = gps_data[gps_index].horacc;
            gps_msg.position_covariance[4] = gps_data[gps_index].horacc;
            gps_msg.position_covariance[8] = gps_data[gps_index].veracc;

            // gps_msg.position_covariance[0] = 0.5;
            // gps_msg.position_covariance[4] = 0.5;
            // gps_msg.position_covariance[8] = 0.5;
            gps_pub.publish(gps_msg);
            ROS_INFO("Published GPS data at timestamp: %f", gps_data[gps_index].timestamp);
            gps_index++;
        } 

        if (vio_index < vio_data_size && vio_wait <= 0){
            nav_msgs::Odometry vio_msg;

            vio_msg.header.stamp = ros::Time(vio_data[vio_index].timestamp);
            vio_msg.header.frame_id = "odom";
            vio_msg.child_frame_id = "base_link";

            vio_msg.pose.pose.position.x = vio_data[vio_index].px;
            vio_msg.pose.pose.position.y = vio_data[vio_index].py;
            vio_msg.pose.pose.position.z = vio_data[vio_index].pz;

            vio_msg.pose.pose.orientation.x = vio_data[vio_index].x;
            vio_msg.pose.pose.orientation.y = vio_data[vio_index].y;
            vio_msg.pose.pose.orientation.z = vio_data[vio_index].z;
            vio_msg.pose.pose.orientation.w = vio_data[vio_index].w;

            vio_msg.pose.covariance[0] = 5;// 
            vio_msg.pose.covariance[7] = 5; // y
            vio_msg.pose.covariance[14] = 2; // z

            

            
            vio_msg.pose.covariance[21] = 0.02; // roll
            vio_msg.pose.covariance[28] = 0.02; // pitch
            vio_msg.pose.covariance[35] = 0.02; // yaw



            vio_msg.twist.covariance[0] = -1;
            vio_msg.twist.covariance[7] = -1;
            vio_msg.twist.covariance[14] = -1;

            vio_msg.twist.covariance[21] = -1;
            vio_msg.twist.covariance[28] = -1;
            vio_msg.twist.covariance[35] = -1;            
            


            vio_pub.publish(vio_msg);
            // ROS_INFO("Published VIO data at timestamp: %f", vio_data[vio_index].timestamp);
            vio_index++;
        }

        double publish_duration = ros::Time::now().toSec() - current_time;
        double min_wait = std::min({imu_wait, gps_wait, vio_wait}) - publish_duration;
        if (min_wait > 0) {
            ros::Duration(min_wait).sleep();
        }

             
    }
    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_data_publish");
    ros::NodeHandle nh("~");
    std::string imu_csv_file;
    std::string gps_csv_file;
    std::string vio_csv_file;




    nh.getParam("imu_csv_file", imu_csv_file);
    nh.getParam("gps_csv_file", gps_csv_file);
    nh.getParam("vio_csv_file", vio_csv_file);



    std::vector<ImuData> imu_data = readImu(imu_csv_file);
    std::vector<GpsData> gps_data = readGps(gps_csv_file);
    std::vector<VIOdata> vio_data = readVIO(vio_csv_file);



    if (imu_data.empty() || gps_data.empty() || vio_data.empty()){
        ROS_ERROR("No data to publish");
        return 1;
    }



    publishData(imu_data, gps_data, vio_data);
    return 0;


}