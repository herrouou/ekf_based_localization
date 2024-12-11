#include <tinyxml.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <string>

struct Coordinate{
    double longitude;
    double latitude;
    double altitude;
};

std::vector<Coordinate> parseKML(const std::string& file_path){
    std::vector<Coordinate> coordinates;
    TiXmlDocument doc(file_path.c_str());

    if(!doc.LoadFile()){
        
        ROS_ERROR("Fail to load the file");
        return coordinates;
    }
    TiXmlElement* root = doc.RootElement();
    if (!root){
        ROS_ERROR("No root elements be found");
        return coordinates;
    }
    // root 是一个指向 TiXmlElement 类型的指针。
    // root->FirstChildElement("Document") 表示调用 root 指向的对象的 FirstChildElement("Document") 方法，返回的也是一个 TiXmlElement 类型的指针。
    // 再次调用 ->FirstChildElement("Placemark") 表示继续对返回的 TiXmlElement 指针调用 FirstChildElement("Placemark") 方法。
    TiXmlElement* placemark = root->FirstChildElement("Document")->FirstChildElement("Placemark");
    while (placemark){
        TiXmlElement* lineString = placemark->FirstChildElement("LineString");
        if (lineString){
            TiXmlElement* coordinateElement = lineString->FirstChildElement("coordinates");
            if (coordinateElement && coordinateElement->GetText()){
                std::string coord_text = coordinateElement->GetText();
                std::istringstream ss(coord_text);
                std::string coordinate;
                while (std::getline(ss, coordinate, ' ')){
                    double lo, la, al;
                    sscanf(coordinate.c_str(), "%lf,%lf,%lf", &lo, &la, &al);
                    coordinates.push_back({lo, la, al});
                }
            }
        }
        placemark = placemark->NextSiblingElement("Placemark");
    }
    return coordinates;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kml_visual");
    ros::NodeHandle nh("~");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("kml_path_raw", 1);

    std::string kml_file;
    nh.getParam("kml_file", kml_file);
    ROS_INFO("KML file path: %s", kml_file.c_str());
    std::vector<Coordinate> coordinates = parseKML(kml_file);
    
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();

    for (const auto& coord : coordinates){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        pose.pose.position.x = coord.longitude;
        pose.pose.position.y = coord.latitude;
        pose.pose.position.z = coord.altitude;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);

    }


    ros::Rate r(1);
    while(ros::ok()){
        path.header.stamp = ros::Time::now();
        path_pub.publish(path);
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}


