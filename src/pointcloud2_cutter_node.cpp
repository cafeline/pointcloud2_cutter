#include "pointcloud2_cutter/pointcloud2_cutter_node.hpp"
#include "pointcloud2_cutter/region_marker_helper.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <functional>
#include <filesystem>
#include <limits>
#include <utility>

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
  int64_t max_filtered_points_param = declare_parameter<int64_t>("max_filtered_points", 0);
  if (max_filtered_points_param < 0)
  {
    RCLCPP_WARN(get_logger(), "max_filtered_points は0以上である必要があります (入力値: %ld)", max_filtered_points_param);
    max_filtered_points_param = 0;
  }
  max_filtered_points_ = static_cast<std::size_t>(max_filtered_points_param);
  scan_frame_id_ = declare_parameter<std::string>("scan_frame_id", "lidar_link");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", scan_frame_id_);
  publish_scan_ = declare_parameter<bool>("publish_scan", true);
  const auto scan_topic = declare_parameter<std::string>("output_scan_topic", "scan");
  scan_height_min_ = declare_parameter<double>("scan_height_min", 0.0);
  scan_height_max_ = declare_parameter<double>("scan_height_max", 2.0);
  scan_range_min_ = declare_parameter<double>("scan_range_min", 0.05);
  scan_range_max_ = declare_parameter<double>("scan_range_max", 100.0);
  const double angle_min_param = declare_parameter<double>("scan_angle_min", -3.14159265358979323846);
  const double angle_max_param = declare_parameter<double>("scan_angle_max", 3.14159265358979323846);
  scan_angle_increment_ = declare_parameter<double>("scan_angle_increment", 0.005);
  region_marker_frame_id_ = declare_parameter<std::string>("region_marker_frame_id", "map");

  auto region_marker_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  region_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "region_markers", region_marker_qos);

  if (publish_scan_)
  {
    bool valid_scan_config = true;
    if (scan_angle_increment_ <= 0.0)
    {
      RCLCPP_ERROR(get_logger(), "scan_angle_increment は正の値である必要があります (%.6f)", scan_angle_increment_);
      valid_scan_config = false;
    }
    if (angle_max_param <= angle_min_param)
    {
      RCLCPP_ERROR(get_logger(), "scan_angle_max (%.6f) は scan_angle_min (%.6f) より大きい必要があります",
        angle_max_param, angle_min_param);
      valid_scan_config = false;
    }
    if (scan_range_max_ <= scan_range_min_)
    {
      RCLCPP_ERROR(get_logger(), "scan_range_max (%.2f) は scan_range_min (%.2f) より大きい必要があります",
        scan_range_max_, scan_range_min_);
      valid_scan_config = false;
    }

    if (!valid_scan_config)
    {
      publish_scan_ = false;
    }
    else
    {
      const double span = angle_max_param - angle_min_param;
      scan_bin_count_ = static_cast<std::size_t>(std::floor(span / scan_angle_increment_)) + 1;
      if (scan_bin_count_ == 0)
      {
        RCLCPP_ERROR(get_logger(), "LaserScan のビン数が 0 になりました。パラメータを確認してください。");
        publish_scan_ = false;
      }
      else
      {
        scan_angle_min_ = angle_min_param;
        scan_angle_max_ = scan_angle_min_ + static_cast<double>(scan_bin_count_ - 1) * scan_angle_increment_;

        if (std::abs(scan_angle_max_ - angle_max_param) > 1e-6)
        {
          RCLCPP_WARN(get_logger(),
            "scan_angle_max を角度刻みと整合させました (要求 %.6f -> 実際 %.6f)",
            angle_max_param, scan_angle_max_);
        }

        if (scan_height_max_ <= scan_height_min_)
        {
          RCLCPP_WARN(get_logger(),
            "scan_height_max (%.2f) が scan_height_min (%.2f) 以下です。LaserScan が空になる可能性があります。",
            scan_height_max_, scan_height_min_);
        }

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
      }
    }
  }
  RCLCPP_INFO(get_logger(), "領域マーカーの publish を試みます");

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

  publishRegionMarkers();
  RCLCPP_INFO(get_logger(), "領域マーカーの publish を試みました");

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

  if (publish_scan_ && scan_pub_)
  {
    RCLCPP_INFO(get_logger(),
      "LaserScan 出力を有効化: topic=%s, height=[%.2f, %.2f], range=[%.2f, %.2f], bins=%zu, angle_increment=%.4f",
      scan_topic.c_str(), scan_height_min_, scan_height_max_, scan_range_min_, scan_range_max_,
      scan_bin_count_, scan_angle_increment_);
  }
  else if (!publish_scan_)
  {
    RCLCPP_INFO(get_logger(), "LaserScan 出力は無効化されています");
  }
  else
  {
    RCLCPP_WARN(get_logger(), "LaserScan 出力を初期化できませんでした。設定を確認してください。");
  }
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
  if (!cloud_pub_ && !publish_scan_)
  {
    return;
  }

  const auto [z_min, z_max] = currentLaserZ();
  auto filtered = PointCloudFilter::filterByHeight(*msg, z_min, z_max, output_frame_id_);
  auto filtered_for_scan = std::shared_ptr<sensor_msgs::msg::PointCloud2>();
  const bool can_reuse_for_scan =
    publish_scan_ &&
    std::abs(scan_height_min_ - z_min) < 1e-9 &&
    std::abs(scan_height_max_ - z_max) < 1e-9 &&
    scan_frame_id_ == output_frame_id_;
  if (can_reuse_for_scan) {
    filtered_for_scan = filtered;
  }

  if (filtered && max_filtered_points_ > 0)
  {
    filtered = PointCloudFilter::limitPointCount(filtered, max_filtered_points_);
  }

  if (cloud_pub_ && filtered && filtered->width > 0)
  {
    cloud_pub_->publish(*filtered);
  }

  if (publish_scan_)
  {
    publishLaserScan(*msg, filtered_for_scan);
  }
}

void PointCloud2CutterNode::publishRegionMarkers()
{
  if (region_markers_published_ || !region_marker_pub_)
  {
    return;
  }

  const auto & regions = region_selector_.regions();
  if (regions.empty())
  {
    return;
  }

  RegionMarkerConfig marker_config;
  marker_config.frame_id = region_marker_frame_id_;

  auto markers = create_region_markers(regions, marker_config);
  const auto stamp = this->now();
  for (auto & marker : markers.markers)
  {
    marker.header.stamp = stamp;
  }

  region_marker_pub_->publish(markers);
  region_markers_published_ = true;
  RCLCPP_INFO(get_logger(), "領域マーカーを %zu 件 publish しました", markers.markers.size());
}

void PointCloud2CutterNode::publishLaserScan(
  const sensor_msgs::msg::PointCloud2 & msg,
  const std::shared_ptr<sensor_msgs::msg::PointCloud2> & prefiltered)
{
  if (!scan_pub_ || scan_bin_count_ == 0)
  {
    return;
  }

  auto filtered = prefiltered ? prefiltered :
    PointCloudFilter::filterByHeight(msg, scan_height_min_, scan_height_max_, scan_frame_id_);

  sensor_msgs::msg::LaserScan scan;
  scan.header = msg.header;
  scan.header.frame_id = scan_frame_id_;
  scan.angle_min = static_cast<float>(scan_angle_min_);
  scan.angle_max = static_cast<float>(scan_angle_max_);
  scan.angle_increment = static_cast<float>(scan_angle_increment_);
  scan.time_increment = 0.0;
  scan.scan_time = 0.0;
  scan.range_min = static_cast<float>(scan_range_min_);
  scan.range_max = static_cast<float>(scan_range_max_);
  scan.ranges.assign(scan_bin_count_, std::numeric_limits<float>::quiet_NaN());

  if (filtered && filtered->width > 0)
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*filtered, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*filtered, "y");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y)
    {
      const double x = static_cast<double>(*iter_x);
      const double y = static_cast<double>(*iter_y);

      const double range = std::hypot(x, y);
      if (range < scan_range_min_ || range > scan_range_max_)
      {
        continue;
      }

      const double angle = std::atan2(y, x);
      if (angle < scan_angle_min_ || angle > scan_angle_max_)
      {
        continue;
      }

      const double index_real = (angle - scan_angle_min_) / scan_angle_increment_;
      const std::size_t index = static_cast<std::size_t>(std::floor(index_real));
      if (index >= scan_bin_count_)
      {
        continue;
      }

      float & stored = scan.ranges[index];
      if (std::isnan(stored) || range < stored)
      {
        stored = static_cast<float>(range);
      }
    }
  }

  scan_pub_->publish(scan);
}

}  // namespace pointcloud2_cutter
