#include <limits>
#include <memory>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <mutex>
#include <rclcpp/clock.hpp>
#include "rosgraph_msgs/msg/clock.hpp"

class PointCloudToLaserScan : public rclcpp::Node {
public:
  PointCloudToLaserScan() : Node("laser_scan") {
    // ========== 파라미터 선언 ==========
    // 토픽 설정
    this->declare_parameter<std::string>("pcd_topic", "/points");
    this->declare_parameter<std::string>("scan_topic", "/scan");
    
    // Z축 클리핑 범위 (미터)
    this->declare_parameter<float>("clipping_minz", -0.2);
    this->declare_parameter<float>("clipping_maxz", 0.4);
    
    // LaserScan 설정
    this->declare_parameter<std::string>("scan_frame_id", "ouster_base_link");
    this->declare_parameter<double>("angle_min", -M_PI);  // 라디안
    this->declare_parameter<double>("angle_max", M_PI);   // 라디안
    this->declare_parameter<double>("angle_increment", M_PI / 180.0);  // 1도 간격
    this->declare_parameter<double>("range_min", 0.5);    // 미터
    this->declare_parameter<double>("range_max", 200.0);  // 미터
    
    // ========== 파라미터 읽기 ==========
    pcd_topic = this->get_parameter("pcd_topic").as_string();
    scan_topic = this->get_parameter("scan_topic").as_string();
    clipping_minz = this->get_parameter("clipping_minz").as_double();
    clipping_maxz = this->get_parameter("clipping_maxz").as_double();
    scan_frame_id = this->get_parameter("scan_frame_id").as_string();
    angle_min = this->get_parameter("angle_min").as_double();
    angle_max = this->get_parameter("angle_max").as_double();
    angle_increment = this->get_parameter("angle_increment").as_double();
    range_min = this->get_parameter("range_min").as_double();
    range_max = this->get_parameter("range_max").as_double();

    // ========== Publisher/Subscriber 설정 ==========
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    publisher_scan = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, qos_profile);

    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pcd_topic, rclcpp::SensorDataQoS(), 
      std::bind(&PointCloudToLaserScan::pointCloudCallback, this, std::placeholders::_1));
    
    clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), 
      std::bind(&PointCloudToLaserScan::clockCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PointCloudToLaserScan node started");
    RCLCPP_INFO(this->get_logger(), "  Input: %s", pcd_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output: %s", scan_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Z clipping: [%.2f, %.2f] m", clipping_minz, clipping_maxz);
    RCLCPP_INFO(this->get_logger(), "  Range: [%.2f, %.2f] m, Angle: [%.2f, %.2f] rad", 
                range_min, range_max, angle_min, angle_max);
  }

private:
  void clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(clock_mutex_);
    last_clock_ = msg->clock;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // PointCloud2 -> PCL 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      return;
    }

    // Z축 클리핑 필터만 적용
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(clipping_minz, clipping_maxz);
    pass.filter(*cloud_filtered);

    // LaserScan 변환 및 발행
    sensor_msgs::msg::LaserScan laser_scan = convertPointCloudToLaserScan(cloud_filtered);
    publisher_scan->publish(laser_scan);
  }

  sensor_msgs::msg::LaserScan convertPointCloudToLaserScan(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    
    // 빈 포인트클라우드 체크
    if (cloud->empty()) {
      return createEmptyLaserScan();
    }

    // LaserScan 메시지 생성
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = scan_frame_id;
    
    // 타임스탬프 설정
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      if (last_clock_.nanosec > 0 || last_clock_.sec > 0) {
        scan.header.stamp = last_clock_;
      } else {
        scan.header.stamp = this->get_clock()->now();
      }
    }

    // LaserScan 파라미터 설정
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    scan.range_min = range_min;
    scan.range_max = range_max;
    scan.scan_time = 0.1;  // 기본값
    scan.time_increment = scan.scan_time / 
        static_cast<double>(std::round((angle_max - angle_min) / angle_increment));

    // 각도 범위에 따른 배열 크기 계산
    uint32_t ranges_size = static_cast<uint32_t>(
        std::round((angle_max - angle_min) / angle_increment));
    scan.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
    scan.intensities.assign(ranges_size, 0.0);

    // 포인트를 LaserScan으로 변환
    for (const auto& point : cloud->points) {
      float range = std::sqrt(point.x * point.x + point.y * point.y);
      
      // 거리 범위 체크
      if (range < range_min || range > range_max) {
        continue;
      }

      float angle = std::atan2(point.y, point.x);
      
      // 각도 범위 체크
      if (angle < angle_min || angle > angle_max) {
        continue;
      }

      // 각도를 인덱스로 변환
      int index = static_cast<int>((angle - angle_min) / angle_increment);
      if (index >= 0 && index < static_cast<int>(ranges_size)) {
        // 같은 각도에 여러 포인트가 있으면 가장 가까운 거리 사용
        scan.ranges[index] = std::min(range, scan.ranges[index]);
      }
    }

    return scan;
  }

  sensor_msgs::msg::LaserScan createEmptyLaserScan() {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = scan_frame_id;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      if (last_clock_.nanosec > 0 || last_clock_.sec > 0) {
        scan.header.stamp = last_clock_;
      } else {
        scan.header.stamp = this->get_clock()->now();
      }
    }
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    scan.range_min = range_min;
    scan.range_max = range_max;
    scan.scan_time = 0.1;
    uint32_t ranges_size = static_cast<uint32_t>(
        std::round((angle_max - angle_min) / angle_increment));
    scan.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
    scan.intensities.assign(ranges_size, 0.0);
    return scan;
  }

  // ========== 멤버 변수 ==========
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

  std::mutex clock_mutex_;
  builtin_interfaces::msg::Time last_clock_;

  // 파라미터 변수
  std::string pcd_topic;
  std::string scan_topic;
  std::string scan_frame_id;
  float clipping_minz;
  float clipping_maxz;
  double angle_min;
  double angle_max;
  double angle_increment;
  double range_min;
  double range_max;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudToLaserScan>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
