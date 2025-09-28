#include "pointcloud2_cutter/pointcloud2_cutter_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <functional>
#include <utility>
#include <filesystem>

namespace pointcloud2_cutter
{

namespace
{
std::string resolveDefaultConfigPath()
{
  try
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory("pointcloud2_cutter");
    return share_dir + "/config/tsudanuma_regions.yaml";
  }
  catch (const std::exception & ex)
  {
    (void)ex;
    return std::string();
  }
}
}  // namespace

PointCloud2CutterNode::PointCloud2CutterNode()
: Node("pointcloud2_cutter"),
  region_selector_(declare_parameter<double>("default_laser_height_min", 2.0),
    declare_parameter<double>("default_laser_height_max", 50.0))
{
  const auto input_topic = declare_parameter<std::string>("input_pointcloud_topic", "/livox/lidar");
  const auto filtered_topic = declare_parameter<std::string>("filtered_pointcloud_topic", "pointcloud/cut");
  const auto output_topic = declare_parameter<std::string>("output_filtered_pointcloud_topic", filtered_topic);
  const auto pose_topic = declare_parameter<std::string>("pose_topic", "/mcl_pose");
  const auto scan_frame = declare_parameter<std::string>("scan_frame_id", "lidar_link");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", scan_frame);

  std::string config_path_param = declare_parameter<std::string>("regions_config_path", "");
  if (config_path_param.empty())
  {
    config_path_param = resolveDefaultConfigPath();
  }

  std::string config_path = config_path_param;
  try
  {
    std::filesystem::path path(config_path_param);
    if (!path.is_absolute())
    {
      const auto share_dir = ament_index_cpp::get_package_share_directory("pointcloud2_cutter");
      path = std::filesystem::path(share_dir) / path;
    }
    config_path = path.lexically_normal().string();
  }
  catch (const std::exception & ex)
  {
    RCLCPP_WARN(get_logger(), "領域設定パスの解決に失敗しました (%s)。入力値をそのまま使用します", ex.what());
  }

  if (config_path.empty())
  {
    RCLCPP_WARN(get_logger(), "有効な regions_config_path が得られませんでした。領域フィルタはデフォルト高さのみを使用します");
  }
  else
  {
    try
    {
      region_selector_.loadFromFile(config_path);
      RCLCPP_INFO(get_logger(), "領域設定を %zu 件読み込みました (%s)",
        region_selector_.regions().size(), config_path.c_str());
    }
    catch (const std::exception & ex)
    {
      RCLCPP_ERROR(get_logger(), "領域設定の読み込みに失敗しました: %s", ex.what());
    }
  }

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_topic, rclcpp::QoS(10),
    std::bind(&PointCloud2CutterNode::handlePose, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic, rclcpp::SensorDataQoS(),
    std::bind(&PointCloud2CutterNode::handlePointCloud, this, std::placeholders::_1));

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, rclcpp::SensorDataQoS());

  RCLCPP_INFO(get_logger(),
    "pointcloud2_cutter を起動しました: input=%s, output=%s, pose=%s, frame=%s",
    input_topic.c_str(), output_topic.c_str(), pose_topic.c_str(), output_frame_id_.c_str());
}

void PointCloud2CutterNode::handlePose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;
  has_pose_ = true;
}

std::pair<double, double> PointCloud2CutterNode::currentLaserZ() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  if (!has_pose_)
  {
    return region_selector_.defaultLaserZ();
  }

  return region_selector_.determineLaserZ(current_x_, current_y_);
}

void PointCloud2CutterNode::handlePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (!cloud_pub_)
  {
    return;
  }

  const auto [z_min, z_max] = currentLaserZ();
  auto filtered = PointCloudFilter::filterByHeight(*msg, z_min, z_max, output_frame_id_);

  if (filtered && filtered->width > 0)
  {
    cloud_pub_->publish(*filtered);
  }
}

}  // namespace pointcloud2_cutter
