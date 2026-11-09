#include <bag_to_pcd.hpp>

bag_to_pcd::bag_to_pcd()
    : rclcpp::Node("bag_to_pcd") 
{
    read_parameters();
    // 检查输入的bag路径是一个bag还是多个bag组成的文件夹
    std::string metadata_path = bags_dir + "/metadata.yaml";
    if (fs::exists(metadata_path)) {
        auto bag_file = bags_dir;
        process_bag_file(bags_dir, bag_count);        
    } else {
        std::vector<std::string> bag_files = find_sorted_ros2_bags();
        if (bag_files.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No ROS 2 bag files found in the specified directory.");
            return;
        }
        for (const auto &bag_file : bag_files) {
            process_bag_file(bag_file, bag_count);
            bag_count++;
        }
    }
};

bag_to_pcd::~bag_to_pcd() {};

void bag_to_pcd::read_parameters() {
    bags_dir = this->declare_parameter<string>("bags_dir", "/data/DeltaElect/Bag/20250302_bag");
    pcds_dir = this->declare_parameter<string>("pcds_dir", "/data/DeltaElect/lib/calib_ros2_ws/src/calib_ros2/result/pcd");
    images_dir = this->declare_parameter<string>("images_dir", "/data/DeltaElect/lib/calib_ros2_ws/src/calib_ros2/result/img_left");
    lidar_topic = this->declare_parameter<string>("lidar_topic", "/livox/lidar");
    image_topic = this->declare_parameter<string>("image_topic", "/hikcamera/left/compressed");
    is_custom_msg = this->declare_parameter<bool>("is_custom_msg", true);

    if (bags_dir.empty() || pcds_dir.empty() || images_dir.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Bag file or PCD file parameter is empty.");
        return;
    }
};

std::vector<std::string> bag_to_pcd::find_sorted_ros2_bags() {
    try {
        // 遍历目录
        for (const auto &entry : fs::directory_iterator(bags_dir)) {
            if (fs::is_directory(entry.path())) {
                // 检查是否包含 metadata.yaml
                std::string metadata_path = entry.path().string() + "/metadata.yaml";
                if (fs::exists(metadata_path)) {
                    bag_folders.push_back(entry.path().string());
                }
            }
        }
        // 按照文件夹名称升序排序
        std::sort(bag_folders.begin(), bag_folders.end());

        return bag_folders;
    } catch (const std::exception &e) {
        std::cerr << "Error accessing directory: " << e.what() << std::endl;
        return {};
    }
};

void bag_to_pcd::process_bag_file(const std::string &bag_file, const int &bag_count) {    
    file_.open(bag_file, ios::in);
    if (!file_) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open ROS bag file: %s", bag_file.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loading ROS bag: %s", bag_file.c_str());

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file;
    storage_options.storage_id = "sqlite3";

    try {
        reader.open(storage_options);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Loading bag failed: %s", e.what());
        return;
    }
    bool saved_frame = false;
    image_frame_count = 0;
    while (reader.has_next()) {
        auto message = reader.read_next();
        if (message->topic_name == lidar_topic) {
            if (is_custom_msg) {
                auto livox_msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>();
                rclcpp::Serialization<livox_ros_driver2::msg::CustomMsg> serializer;
                rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
                serializer.deserialize_message(&serialized_msg, livox_msg.get());

                for (uint i = 0; i < livox_msg->point_num; ++i) {
                    pcl::PointXYZI p;
                    p.x = livox_msg->points[i].x;
                    p.y = livox_msg->points[i].y;
                    p.z = livox_msg->points[i].z;
                    p.intensity = livox_msg->points[i].reflectivity;
                    output_cloud.points.push_back(p);
                }
            } else {
                auto livox_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
                rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
                serializer.deserialize_message(&serialized_msg, livox_cloud.get());

                pcl::PointCloud<pcl::PointXYZI> cloud;
                pcl::PCLPointCloud2 pcl_pc;
                pcl_conversions::toPCL(*livox_cloud, pcl_pc);
                pcl::fromPCLPointCloud2(pcl_pc, cloud);

                for (uint i = 0; i < cloud.size(); ++i) {
                    output_cloud.points.push_back(cloud.points[i]);
                }
            }
        }
        // **处理图像数据**
        if (message->topic_name == image_topic) {
            image_frame_count++;

            if (image_frame_count == 50) {
                saveimage_from_bag(message, bag_count);
                saved_frame = true;
            }
        }
    }
    // **保存 PCD 文件**
    output_cloud.is_dense = false;
    output_cloud.width = output_cloud.points.size();
    output_cloud.height = 1;
    string pcd_file = pcds_dir + "/" + std::to_string(bag_count) + ".pcd";
    pcl::io::savePCDFileASCII(pcd_file, output_cloud);

    RCLCPP_INFO(this->get_logger(), "Successfully saved point cloud to PCD file: %s", pcd_file.c_str());
    file_.close();
    output_cloud.clear();
};

void bag_to_pcd::saveimage_from_bag(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> &message, const int &bag_count) {
    // 判断是否为 CompressedImage（一般 compressed 话题以 "/compressed" 结尾）
    if (message->topic_name.find("/compressed") != std::string::npos) {
        auto compressed_image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
        rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
        serializer.deserialize_message(&serialized_msg, compressed_image_msg.get());

        try {
            cv::Mat image = cv::imdecode(cv::Mat(compressed_image_msg->data), cv::IMREAD_COLOR);
            if (image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
                return;
            }
            string image_file = images_dir + "/" + std::to_string(bag_count) + ".bmp";
            cv::imwrite(image_file, image);
            RCLCPP_INFO(this->get_logger(), "Saved image from sensor_msgs::msg::CompressedImage to: %s", image_file.c_str());
        } catch (const cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV imdecode failed: %s", e.what());
        }
    } else {  // 处理标准的 `sensor_msgs::msg::Image`
        auto image_msg = std::make_shared<sensor_msgs::msg::Image>();
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        rclcpp::SerializedMessage serialized_msg(*message->serialized_data);
        serializer.deserialize_message(&serialized_msg, image_msg.get());

        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
            string image_file = images_dir + "/" + std::to_string(bag_count) + ".bmp";
            cv::imwrite(image_file, cv_ptr->image);
            RCLCPP_INFO(this->get_logger(), "Saved image from sensor_msgs::msg::Image to: %s", image_file.c_str());
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge conversion failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bag_to_pcd>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}