#include "autoware/pointcloud_preprocessor/passthrough_filter/passthrough_filter_node.hpp"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/segment_differences.h>

#include <vector>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

namespace autoware::pointcloud_preprocessor
{

PassThroughFilterComponent::PassThroughFilterComponent(const rclcpp::NodeOptions & options)
: Filter("PassThroughFilter", options)
{
    using std::placeholders::_1;
    set_param_res_ = this->add_on_set_parameters_callback(
        std::bind(&PassThroughFilterComponent::param_callback, this, _1));

    // pcd filtering parameters
    this->declare_parameter<double>("z_max", 0.5);
    this->declare_parameter<double>("z_min", 0.0);
    this->declare_parameter<double>("x_min", -3.0); 
    this->declare_parameter<double>("x_max", 3.0); 
    this->declare_parameter<double>("y_min", -3.0); 
    this->declare_parameter<double>("y_max", 3.0);  

    z_max_ = this->get_parameter("z_max").as_double();
    z_min_ = this->get_parameter("z_min").as_double();
    x_min_ = this->get_parameter("x_min").as_double();
    x_max_ = this->get_parameter("x_max").as_double();
    y_min_ = this->get_parameter("y_min").as_double();
    y_max_ = this->get_parameter("y_max").as_double();

}
//0909 새로 추가한 코드 x,y,z에 대해 모두 필터링 수행
void PassThroughFilterComponent::filter(
    const PointCloud2ConstPtr & input,
    const IndicesPtr & indices,
    PointCloud2 & output)
{
    std::scoped_lock lock(mutex_);

    if (indices) {
        RCLCPP_WARN(get_logger(), "Indices are not supported and will be ignored");
    }

    // ROS PointCloud2 -> PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *pcl_cloud);

    // PassThrough 필터 객체 생성
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 1. Z축 필터링 (z_max만 제한)
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-std::numeric_limits<double>::max(), z_max_);
    // pass.setFilterLimits(z_min_, z_max_);
    pass.setKeepOrganized(false);
    pass.filter(*tmp_cloud);

    // // 2. X축 필터링
    pass.setInputCloud(tmp_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min_, x_max_);
    pass.filter(*tmp_cloud);

    // 3. Y축 필터링
    pass.setInputCloud(tmp_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min_, y_max_);
    pass.filter(*tmp_cloud);

    // 결과 복사
    pcl::toROSMsg(*tmp_cloud, output);
    output.header = input->header;
}

rcl_interfaces::msg::SetParametersResult PassThroughFilterComponent::param_callback(
    const std::vector<rclcpp::Parameter> & p)
{
    std::scoped_lock lock(mutex_);

    for (const auto & param : p)
    {
        if (param.get_name() == "z_max")
        {
            z_max_ = param.as_double();
        }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Z-axis max filter parameter updated";
    return result;
}
}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::PassThroughFilterComponent)
