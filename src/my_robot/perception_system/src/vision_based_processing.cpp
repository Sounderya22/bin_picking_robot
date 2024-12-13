#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr
applySOR(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(10); // Lower value, faster but less accurate
  // Increase StddevMulThresh to speed up the process by retaining more points
  sor.setStddevMulThresh(2.0); // Looser threshold, faster
  sor.filter(*filtered_cloud);
  return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
            double distance_threshold = 0.01) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_without_plane(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  // Check if a plane was found
  if (inliers->indices.size() == 0) {
    std::cerr << "Could not estimate a planar model for the given dataset."
              << std::endl;
    return input_cloud;
  }

  // Remove the planar inliers, extract the rest
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_without_plane);

  return cloud_without_plane;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
getCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
           float cluster_tolerance = 0.005, int min_cluster_size = 100,
           int max_cluster_size = 25000) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *cloud_filtered, indices);

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  if (cluster_indices.empty()) {
    std::cerr << "No clusters found." << std::endl;
    return cluster_cloud; // Return empty cloud
  }

  // Find the largest cluster
  auto largest_cluster = std::max_element(
      cluster_indices.begin(), cluster_indices.end(),
      [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
        return a.indices.size() < b.indices.size();
      });

  // Extract the largest cluster
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_filtered);
  pcl::PointIndices::Ptr largest_cluster_indices(
      new pcl::PointIndices(*largest_cluster));
  extract.setIndices(largest_cluster_indices);
  extract.filter(*cluster_cloud);

  // Set cloud properties
  cluster_cloud->width = cluster_cloud->points.size();
  cluster_cloud->height = 1;
  cluster_cloud->is_dense = true;

  return cluster_cloud;
}

Eigen::Matrix4f
getPoseFromCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster) {
  // 1. Compute the centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);

  // 2. Compute the principal axes
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cluster);
  Eigen::Matrix3f eigenVectors = pca.getEigenVectors();

  // Ensure the eigenvectors form a right-handed coordinate system
  if (eigenVectors.determinant() < 0) {
    eigenVectors.col(2) = -eigenVectors.col(2);
  }

  // 3. Construct the transformation matrix
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = eigenVectors;
  transformation.block<3, 1>(0, 3) = centroid.head<3>();
  return transformation;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
applySOR(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);            // Adjust based on your data
  sor.setStddevMulThresh(1.0); // Adjust based on your data
  sor.filter(*filtered_cloud);
  return filtered_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
removePlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
            double distance_threshold = 0.01) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_without_plane(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setInputCloud(input_cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cout << "Could not estimate a planar model for the given dataset.\n";
    return input_cloud;
  }

  // Remove the planar inliers and extract the rest
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true); // Keep points NOT part of the plane
  extract.filter(*cloud_without_plane);

  return cloud_without_plane;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
getCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud,
           float cluster_tolerance = 0.02, int min_cluster_size = 100,
           int max_cluster_size = 25000) {
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGB>());

  tree->setInputCloud(input_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input_cloud, *cloud_filtered, indices);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  if (cluster_indices.empty()) {
    std::cerr << "No clusters found." << std::endl;
    return cluster_cloud; // Return empty cloud
  }

  // Find the largest cluster
  auto largest_cluster = std::max_element(
      cluster_indices.begin(), cluster_indices.end(),
      [](const pcl::PointIndices &a, const pcl::PointIndices &b) {
        return a.indices.size() < b.indices.size();
      });

  // Extract the largest cluster
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud_filtered);
  pcl::PointIndices::Ptr largest_cluster_indices(
      new pcl::PointIndices(*largest_cluster));
  extract.setIndices(largest_cluster_indices);
  extract.filter(*cluster_cloud);

  // Set cloud properties
  cluster_cloud->width = cluster_cloud->points.size();
  cluster_cloud->height = 1;
  cluster_cloud->is_dense = true;

  return cluster_cloud;
}

Eigen::Matrix4f
getPoseFromCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cluster) {
  // 1. Compute the centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cluster, centroid);

  // 2. Compute the principal axes
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(cluster);
  Eigen::Matrix3f eigenVectors = pca.getEigenVectors();

  // Ensure the eigenvectors form a right-handed coordinate system
  if (eigenVectors.determinant() < 0) {
    eigenVectors.col(2) = -eigenVectors.col(2);
  }

  // 3. Construct the transformation matrix
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = eigenVectors;
  transformation.block<3, 1>(0, 3) = centroid.head<3>();
  return transformation;
}

// Function to transform a pose from one frame to another
Eigen::Matrix4f transformPoseToBaseLink(
    const Eigen::Matrix4f &input_pose, const std::string &base_link_frame,
    const std::string &input_frame, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Logger logger) {
  try {
    // Convert Eigen::Matrix4f to geometry_msgs::msg::PoseStamped
    geometry_msgs::msg::PoseStamped input_pose_msg;
    input_pose_msg.header.frame_id = input_frame;
    input_pose_msg.header.stamp = rclcpp::Clock().now();

    // Extract translation and rotation from Eigen matrix
    Eigen::Vector3f translation = input_pose.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation_matrix = input_pose.block<3, 3>(0, 0);

    Eigen::Quaternionf quaternion(
        rotation_matrix); // Convert rotation matrix to quaternion

    // Set position and orientation in the PoseStamped message
    input_pose_msg.pose.position.x = translation.x();
    input_pose_msg.pose.position.y = translation.y();
    input_pose_msg.pose.position.z = translation.z();
    input_pose_msg.pose.orientation.x = quaternion.x();
    input_pose_msg.pose.orientation.y = quaternion.y();
    input_pose_msg.pose.orientation.z = quaternion.z();
    input_pose_msg.pose.orientation.w = quaternion.w();

    // Transform the pose to the base_link frame
    geometry_msgs::msg::PoseStamped transformed_pose_msg;
    transformed_pose_msg =
        tf_buffer->transform(input_pose_msg, base_link_frame);

    // Convert the transformed PoseStamped back to Eigen::Matrix4f
    Eigen::Matrix4f transformed_pose = Eigen::Matrix4f::Identity();

    // Set the translation part
    transformed_pose.block<3, 1>(0, 3) << transformed_pose_msg.pose.position.x,
        transformed_pose_msg.pose.position.y,
        transformed_pose_msg.pose.position.z;

    // Set the rotation part
    Eigen::Quaternionf transformed_quaternion(
        transformed_pose_msg.pose.orientation.w,
        transformed_pose_msg.pose.orientation.x,
        transformed_pose_msg.pose.orientation.y,
        transformed_pose_msg.pose.orientation.z);
    transformed_pose.block<3, 3>(0, 0) =
        transformed_quaternion.toRotationMatrix();

    // Log the result
    RCLCPP_INFO(logger, "Transformed Position: [%.3f, %.3f, %.3f]",
                transformed_pose(0, 3), transformed_pose(1, 3),
                transformed_pose(2, 3));
    RCLCPP_INFO(
        logger,
        "Transformed Orientation (Quaternion): [%.3f, %.3f, %.3f, %.3f]",
        transformed_quaternion.x(), transformed_quaternion.y(),
        transformed_quaternion.z(), transformed_quaternion.w());

    return transformed_pose;
  } catch (const tf2::TransformException &ex) {
    RCLCPP_ERROR(logger, "Failed to transform pose: %s", ex.what());
    throw; // Rethrow exception for caller to handle
  }
}

class PointCloudProcessor : public rclcpp::Node {
public:
  PointCloudProcessor() : Node("point_cloud_processor") {
    // Subscriber to the input PointCloud2 topic
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ur_manipulator/base_link/points", // Replace with your input topic
                                            // name
        10,
        std::bind(&PointCloudProcessor::pointCloudCallback, this,
                  std::placeholders::_1));

    // Publisher for the filtered PointCloud2 topic
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "filtered_pointcloud", 10);
    // Create a TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Point Cloud Processor Node has started.");
  }

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    Eigen::Matrix4f pose;
    Eigen::Vector3f position;
    Eigen::Matrix3f rotation;
    Eigen::Vector3f euler_angles;
    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    auto filteredCloud = applySOR(cloud);
    if (filteredCloud->empty()) {
      std::cout << "Filtered cloud after SOR is empty\n";
      return;
    }

    // Step 2: Remove the table plane using RANSAC
    auto segmentCloud = removePlane(filteredCloud);
    if (segmentCloud->empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "segmentCloud cloud after removing plane is empty");
      return;
    }

    auto clusteredCloud = getCluster(segmentCloud);
    if (clusteredCloud->empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "clusteredCloud cloud after clustering is empty");
      return;
    }
    pose = getPoseFromCluster(clusteredCloud);
    position = pose.block<3, 1>(0, 3);
    rotation = pose.block<3, 3>(0, 0);
    euler_angles = rotation.eulerAngles(0, 1, 2);
    std::cout << "Position: " << position.transpose() << std::endl;
    std::cout << "Orientation (Euler angles): " << euler_angles.transpose()
              << std::endl;

    std::stringstream position_stream;
    position_stream << "Position: " << position.transpose();
    RCLCPP_INFO(this->get_logger(), position_stream.str().c_str());

    std::stringstream orientation_stream;
    orientation_stream << "Orientation (Euler angles): "
                       << euler_angles.transpose();
    RCLCPP_INFO(this->get_logger(), orientation_stream.str().c_str());
    std::stringstream ss;
    ss << pose.format(Eigen::IOFormat(Eigen::StreamPrecision,
                                      Eigen::DontAlignCols, ", ", ";\n", "", "",
                                      "[", "]"));

    // Log the result using RCLCPP_INFO
    RCLCPP_INFO(this->get_logger(), "Final Transformed Pose:\n%s",
                ss.str().c_str());
    /*  std::string base_link_frame = "base_link";
     std::string input_frame = "world";
     try
     {
         Eigen::Matrix4f result =
         transformPoseToBaseLink(pose, base_link_frame, input_frame, tf_buffer_,
     this->get_logger());

         // Convert the Eigen matrix to a string using stringstream
         std::stringstream ss;
         ss << result.format(Eigen::IOFormat(Eigen::StreamPrecision,
     Eigen::DontAlignCols, ", ", ";\n", "", "", "[", "]"));

         // Log the result using RCLCPP_INFO
         RCLCPP_INFO(this->get_logger(), "Final Transformed Pose:\n%s",
     ss.str().c_str());
         // RCLCPP_INFO(this->get_logger(), "Final Transformed Pose:\n%s",
     result.format(Eigen::IOFormat())); position = result.block<3, 1>(0, 3);
             rotation = result.block<3, 3>(0, 0);
             euler_angles = rotation.eulerAngles(0, 1, 2);
             std::cout << "Position: " << position.transpose() << std::endl;
             std::cout << "Orientation (Euler angles): " <<
     euler_angles.transpose()
             << std::endl;

             std::stringstream position_stream;
             position_stream << "Final Position: " << position.transpose();
             RCLCPP_INFO(this->get_logger(), position_stream.str().c_str());

             std::stringstream orientation_stream;
             orientation_stream << "Final Orientation (Euler angles): " <<
     euler_angles.transpose(); RCLCPP_INFO(this->get_logger(),
     orientation_stream.str().c_str());
     }
     catch (const std::exception &e)
     {
         RCLCPP_ERROR(this->get_logger(), "Error transforming pose: %s",
     e.what());
     } */

    // Convert filtered PCL PointCloud back to ROS PointCloud2 and publish it
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*clusteredCloud, output_msg);
    output_msg.header =
        msg->header; // Keep the same header as the input message
    publisher_->publish(output_msg);
    RCLCPP_INFO(this->get_logger(), "Filtered point cloud published.");
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
