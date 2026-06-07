/*
 * lidar_ring_preprocessor.cpp
 *
 * Adds a ring field to the organized PointCloud2 output by Gazebo's gpu_lidar
 * sensor.  LIO-SAM (velodyne mode) needs each point labelled with its vertical
 * scan ring index; Gazebo does not include this field.
 *
 * For an organized cloud (height = N_SCAN, width = Horizon_SCAN) the ring
 * index is simply the row number.  An unorganized cloud (height = 1) cannot
 * be processed — a warning is logged and the cloud is forwarded unchanged.
 *
 * Topics (remapped in the launch file):
 *   input  → /lidar/points        (from ros_gz_bridge)
 *   output → /lidar/points_lio    (consumed by LIO-SAM imageProjection)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Velodyne-compatible point type expected by LIO-SAM
struct PointXYZIR
{
    PCL_ADD_POINT4D;
    float    intensity;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
    (float,           x,         x)
    (float,           y,         y)
    (float,           z,         z)
    (float,           intensity, intensity)
    (std::uint16_t,   ring,      ring))

class LidarRingPreprocessor : public rclcpp::Node
{
public:
    LidarRingPreprocessor() : Node("lidar_ring_preprocessor")
    {
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "input", rclcpp::SensorDataQoS(),
            std::bind(&LidarRingPreprocessor::callback, this,
                      std::placeholders::_1));
        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "output", rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "lidar_ring_preprocessor ready");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        const uint32_t H = msg->height;
        const uint32_t W = msg->width;

        if (H == 1) {
            RCLCPP_WARN_ONCE(get_logger(),
                "Received unorganized cloud (height=1) — "
                "ring field cannot be derived from row index. "
                "Forwarding without ring field.");
            pub_->publish(*msg);
            return;
        }

        pcl::PointCloud<pcl::PointXYZI> cloud_in;
        pcl::fromROSMsg(*msg, cloud_in);

        pcl::PointCloud<PointXYZIR> cloud_out;
        cloud_out.width    = W;
        cloud_out.height   = H;
        cloud_out.is_dense = false;
        cloud_out.points.resize(static_cast<size_t>(H) * W);

        for (uint32_t row = 0; row < H; ++row) {
            for (uint32_t col = 0; col < W; ++col) {
                const auto & src = cloud_in.points[row * W + col];
                auto       & dst = cloud_out.points[row * W + col];
                dst.x         = src.x;
                dst.y         = src.y;
                dst.z         = src.z;
                dst.intensity = src.intensity;
                dst.ring      = static_cast<std::uint16_t>(row);
            }
        }

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(cloud_out, out_msg);
        out_msg.header = msg->header;
        pub_->publish(out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarRingPreprocessor>());
    rclcpp::shutdown();
    return 0;
}
