#include <rclcpp/rclcpp.hpp>

#include "imu_filter_madgwick/imu_filter_ros.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::NodeOptions options;

    auto node = std::make_shared<ImuFilterMadgwickRos>(options);
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}
