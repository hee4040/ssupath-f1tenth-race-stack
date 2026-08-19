// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "euclidean_cluster_node.hpp"
#include <sys/time.h>
#include "autoware/euclidean_cluster/utils.hpp"
#include <pcl/common/common.h>  
#include <sys/resource.h>
#include <memory>
#include <vector>

namespace autoware::euclidean_cluster
{
EuclideanClusterNode::EuclideanClusterNode(const rclcpp::NodeOptions & options)
: Node("euclidean_cluster_node", options)
{
  const bool use_height = this->declare_parameter("use_height", false);
  const int min_cluster_size = this->declare_parameter("min_cluster_size", 3);
  const int max_cluster_size = this->declare_parameter("max_cluster_size", 200);
  const float tolerance = this->declare_parameter("tolerance", 1.0);
  
  //size_filter parameters
  max_x_ = this->declare_parameter("max_x", 0.50);
  max_y_ = this->declare_parameter("max_y", 0.50);
  max_z_ = this->declare_parameter("max_z", 0.40);

  cluster_ =
    std::make_shared<EuclideanCluster>(use_height, min_cluster_size, max_cluster_size, tolerance);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&EuclideanClusterNode::onPointCloud, this, _1));

  cluster_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "output", rclcpp::QoS{1});
  debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);

}

void EuclideanClusterNode::onPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);
  // clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  cluster_->cluster(raw_pointcloud_ptr, clusters);
  
  // Without Cluster Size filtering
  // tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  // convertPointCloudClusters2Msg(input_msg->header, clusters, output);

  // for (size_t i = 0; i < clusters.size(); ++i) {
  //   pcl::PointXYZ min_pt, max_pt;
  //   pcl::getMinMax3D(clusters[i], min_pt, max_pt);

  //   double dx = max_pt.x - min_pt.x;
  //   double dy = max_pt.y - min_pt.y;
  //   double dz = max_pt.z - min_pt.z;

  //   output.feature_objects[i].object.shape.dimensions.x = dx;
  //   output.feature_objects[i].object.shape.dimensions.y = dy;
  //   output.feature_objects[i].object.shape.dimensions.z = dz;

  //   output.feature_objects[i].object.shape.type = 0; // BOX
  // }
  // cluster_pub_->publish(output);  


  // Cluster Size Filtering //

  std::vector<pcl::PointCloud<pcl::PointXYZ>> filtered_clusters;
  for (const auto & cluster : clusters) {
      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(cluster, min_pt, max_pt);

      double dx = max_pt.x - min_pt.x;
      double dy = max_pt.y - min_pt.y;
      double dz = max_pt.z - min_pt.z;
      
      if (dx > max_x_ || dy > max_y_ || dz > max_z_) {
        // if (dx > 0.50) {
        //   RCLCPP_INFO(this->get_logger(), "over dxxxxxxxxxxxxxxxxxxxxxx dx=%.3f", dx);
        // } else if (dy > 0.50) {
        //   RCLCPP_INFO(this->get_logger(), "over dyyyyyyyyyyyyyyyyyyyyyy dy=%.3f", dy);
        // } else if (dz > 0.40) {
        //   RCLCPP_INFO(this->get_logger(), "over dzzzzzzzzzzzzzzzzzzzzzz dz=%.3f", dz);
        // }
          continue;  
      }
      filtered_clusters.push_back(cluster);
  }

  tier4_perception_msgs::msg::DetectedObjectsWithFeature output;
  convertPointCloudClusters2Msg(input_msg->header, filtered_clusters, output);

  for (size_t i = 0; i < filtered_clusters.size(); ++i) {
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(filtered_clusters[i], min_pt, max_pt);

    double dx = max_pt.x - min_pt.x;
    double dy = max_pt.y - min_pt.y;
    double dz = max_pt.z - min_pt.z;

    output.feature_objects[i].object.shape.dimensions.x = dx;
    output.feature_objects[i].object.shape.dimensions.y = dy;
    output.feature_objects[i].object.shape.dimensions.z = dz;

    output.feature_objects[i].object.shape.type = 0; // BOX
  }
  cluster_pub_->publish(output);
  if (debug_pub_->get_subscription_count() >= 1) {
    sensor_msgs::msg::PointCloud2 debug;
    convertObjectMsg2SensorMsg(output, debug);
    debug_pub_->publish(debug);
  }


}

} 

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::euclidean_cluster::EuclideanClusterNode)
