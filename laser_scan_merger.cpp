#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

class LaserScanMerger : public rclcpp::Node
{
public:
    LaserScanMerger() : Node("laser_scan_merger")
    {
        laser1_sub_.subscribe(this, "/lidar1/scan");
        laser2_sub_.subscribe(this, "/lidar2/scan");
        
        sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>>(laser1_sub_, laser2_sub_, 10);
        sync_->registerCallback(&LaserScanMerger::sync_callback, this);

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/merged_scan", 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void sync_callback(const sensor_msgs::msg::LaserScan::SharedPtr laser1, const sensor_msgs::msg::LaserScan::SharedPtr laser2)
    {
        auto transformed_laser1 = transform_scan(*laser1, "laser1");
        auto transformed_laser2 = transform_scan(*laser2, "laser2");

        // Merge the transformed scans
        sensor_msgs::msg::LaserScan merged_scan;
        merged_scan.header.frame_id = "base_link";
        for (size_t i = 0; i < transformed_laser1.ranges.size(); ++i) {
            merged_scan.ranges.push_back(std::min(transformed_laser1.ranges[i], transformed_laser2.ranges[i]));
        }

        scan_pub_->publish(merged_scan);
    }

    sensor_msgs::msg::LaserScan transform_scan(const sensor_msgs::msg::LaserScan& scan, const std::string& target_frame)
    {
        sensor_msgs::msg::LaserScan transformed_scan = scan;

        geometry_msgs::msg::TransformStamped tf_stamped;
        try {
            tf_stamped = tf_buffer_->lookupTransform("base_link", target_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return scan;
        }

        for (size_t i = 0; i < scan.ranges.size(); ++i) {
            if (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i])) {
                continue;
            }

            double angle = scan.angle_min + i * scan.angle_increment;
            geometry_msgs::msg::PointStamped in, out;
            in.point.x = scan.ranges[i] * cos(angle);
            in.point.y = scan.ranges[i] * sin(angle);
            tf2::doTransform(in, out, tf_stamped);

            // Convert back to polar coordinates
            double r = sqrt(out.point.x * out.point.x + out.point.y * out.point.y);
            transformed_scan.ranges[i] = r;
        }

        return transformed_scan;
    }
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser1_sub_;
    message_filters::Subscriber<sensor_msgs::msg::LaserScan> laser2_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanMerger>());
    rclcpp::shutdown();
    return 0;
}
