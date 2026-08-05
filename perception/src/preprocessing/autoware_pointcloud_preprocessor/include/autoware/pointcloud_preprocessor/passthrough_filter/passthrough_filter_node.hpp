#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_NODE_HPP_

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include <pcl/search/pcl_search.h>
#include <vector>
#include <mutex>  // <- 추가: mutex 사용

namespace autoware::pointcloud_preprocessor
{
class PassThroughFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
protected:
  virtual void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output);

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  /** 추가: Z축 필터링 범위와 동기화를 위한 mutex */
  std::mutex mutex_;
  double z_min_;
  double z_max_;
  double y_min_;
  double y_max_;
  double x_max_;
  double x_min_;
public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit PassThroughFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__PASSTHROUGH_FILTER__PASSTHROUGH_FILTER_NODE_HPP_
