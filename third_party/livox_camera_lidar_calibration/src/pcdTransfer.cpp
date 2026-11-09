#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>
#include <time.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include "common.h"

using namespace std;

struct pointData{
    float x;
    float y;
    float z;
    int i;
};
vector<pointData> vector_data;
livox_ros_driver2::msg::CustomMsg livox_cloud;
string input_bag_path, output_path;
int threshold_lidar, data_num;

void loadAndSavePointcloud(int index);
void writeTitle(const string filename, unsigned long point_num);
void writePointCloud(const string filename, const vector<pointData> singlePCD);
void dataSave(int index);

void loadAndSavePointcloud(int index) {
    string path = input_bag_path + int2str(index) + ".bag";
    RCLCPP_INFO(rclcpp::get_logger("pcdTransfer"), "Start to load the rosbag %s", path.c_str());
    rosbag2_cpp::Reader reader;
    try {
        reader.open(path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("pcdTransfer"), "LOADING BAG FAILED: " << e.what());
        return;
    }

    rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serialization;
    while (reader.has_next()) {
        auto bag_message = reader.read_next();
        
        if (bag_message->topic_name != "/livox/lidar") {
            continue;
        }
        
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &livox_cloud);
        
        for(uint i = 0; i < livox_cloud.point_num; ++i) {
            pointData myPoint;
            myPoint.x = livox_cloud.points[i].x;
            myPoint.y = livox_cloud.points[i].y;
            myPoint.z = livox_cloud.points[i].z;
            myPoint.i = livox_cloud.points[i].reflectivity;

            vector_data.push_back(myPoint);
        }
        if (vector_data.size() >= static_cast<size_t>(threshold_lidar)) {
            break;
        }
    }
    dataSave(index);
    vector_data.clear();
}

void writeTitle(const string filename, unsigned long point_num) {
    ofstream outfile(filename.c_str(), ios_base::out);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        outfile << "# .PCD v.7 - Point Cloud Data file format" << endl;
        outfile << "VERSION .7" << endl;
        outfile << "FIELDS x y z intensity" << endl;
        outfile << "SIZE 4 4 4 4" << endl;
        outfile << "TYPE F F F F" << endl;
        outfile << "COUNT 1 1 1 1" << endl;
        outfile << "WIDTH " << long2str(point_num) << endl;
        outfile << "HEIGHT 1" << endl;
        outfile << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
        outfile << "POINTS " << long2str(point_num) << endl;
        outfile << "DATA ascii" << endl;
    }
    RCLCPP_INFO(rclcpp::get_logger("pcdTransfer"), "Save file %s", filename.c_str());
}

void writePointCloud(const string filename, const vector<pointData> singlePCD) {
    ofstream outfile(filename.c_str(), ios_base::app);
    if (!outfile) {
        cout << "Can not open the file: " << filename << endl;
        exit(0);
    }
    else {
        for (unsigned long i = 0; i < singlePCD.size(); ++i) {
            outfile << float2str(singlePCD[i].x) << " " << float2str(singlePCD[i].y) << " " << float2str(singlePCD[i].z) << " " << int2str(singlePCD[i].i) << endl;
        }
    }
}

void dataSave(int index) {
    if (!std::filesystem::exists(output_path)) {
        std::filesystem::create_directories(output_path);
    }
    string outputName = output_path + int2str(index) + ".pcd";
    writeTitle(outputName, vector_data.size());
    writePointCloud(outputName, vector_data);
}

void getParameters(std::shared_ptr<rclcpp::Node> node) {
    cout << "Get the parameters from the launch file" << endl;

    node->declare_parameter<std::string>("input_bag_path", "");
    node->declare_parameter<std::string>("output_pcd_path", "");
    node->declare_parameter<int>("threshold_lidar", 0);
    node->declare_parameter<int>("data_num", 0);

    node->get_parameter("input_bag_path", input_bag_path);
    node->get_parameter("output_pcd_path", output_path);
    node->get_parameter("threshold_lidar", threshold_lidar);
    node->get_parameter("data_num", data_num);
    
    cout << input_bag_path << endl;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pcdTransfer");
    getParameters(node);

    for (int i = 0; i < data_num; ++i) {
        loadAndSavePointcloud(i);
    }
    RCLCPP_INFO(node->get_logger(), "Finish all!");
    rclcpp::shutdown();
    return 0;
}
