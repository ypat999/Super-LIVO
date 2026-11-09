#ifndef BAG_TO_PCD_H
#define BAG_TO_PCD_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <vector>
#include <algorithm>

using namespace std;
namespace fs = std::filesystem;

class bag_to_pcd : public rclcpp::Node {
public:
    bag_to_pcd();
    ~bag_to_pcd();
    void read_parameters();
    void process_bag_file(const std::string &bag_file, const int &bag_count);
    std::vector<std::string> find_sorted_ros2_bags();
    void saveimage_from_bag(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &message, const int &bag_count);

    string bags_dir, pcds_dir, images_dir;
    string lidar_topic, image_topic;
    bool is_custom_msg;

    int frame_count, image_frame_count;
    int bag_count = 0;

    std::vector<std::string> bag_folders;

    pcl::PointCloud<pcl::PointXYZI> output_cloud;
    fstream file_;

};

#endif