#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class CloudTransformer : public rclcpp::Node
{
public:
  CloudTransformer()
    : Node("ur_manipulator_point_cloud_tf"), demo_(false)
  {
    // Define Publishers and Subscribers here
    pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/top_rgbd_depth_sensor/points", 1,
      std::bind(&CloudTransformer::pclCallback, this, std::placeholders::_1));

    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ur_manipulator/base_link/points", 1);

    buffer_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    buffer_->header.frame_id = "base_link";

    this->declare_parameter<bool>("demo", true);
    this->get_parameter("demo", demo_);
    RCLCPP_INFO(this->get_logger(), "Demo flag: %s", demo_ ? "true" : "false");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  bool demo_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  sensor_msgs::msg::PointCloud2::SharedPtr buffer_;

  void pclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_msg)
  {
    float standard_deviation = 0.025;
    geometry_msgs::msg::TransformStamped transformStamped;
    
    try {
      transformStamped = tf_buffer_->lookupTransform(
        "base_link", "top_rgbd_camera_link", tf2::TimePointZero);
      
      pcl_ros::transformPointCloud("base_link", *pcl_msg, *buffer_, *tf_buffer_);
      
      if (demo_) {
        pcl_pub_->publish(*buffer_);
      } else {
        // Add noise to buffer Point Cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
        sensor_msgs::msg::PointCloud2 noisy_buffer;

        pcl::fromROSMsg(*buffer_, *cloud);

        // Create the filtering object
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(0.02f, 0.02f, 0.02f);
        sor.filter(*cloud_out);

        cloud_filtered->points.resize(cloud_out->points.size());
        cloud_filtered->header = cloud_out->header;
        cloud_filtered->width = cloud_out->width;
        cloud_filtered->height = cloud_out->height;

        std::mt19937 rng(std::random_device{}());
        std::normal_distribution<float> nd(0.0f, standard_deviation);

        for (size_t i = 0; i < cloud_out->points.size(); ++i) {
          cloud_filtered->points[i].x = cloud_out->points[i].x + nd(rng);
          cloud_filtered->points[i].y = cloud_out->points[i].y + nd(rng);
          cloud_filtered->points[i].z = cloud_out->points[i].z + nd(rng);
          cloud_filtered->points[i].r = cloud_out->points[i].r;
          cloud_filtered->points[i].g = cloud_out->points[i].g;
          cloud_filtered->points[i].b = cloud_out->points[i].b;
        }

        *cloud_out += *cloud_filtered;

        pcl::toROSMsg(*cloud_out, noisy_buffer);

        pcl_pub_->publish(noisy_buffer);
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<CloudTransformer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}