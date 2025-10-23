#include <rclcpp/rclcpp.hpp>
#include "local_btmap/localmap_from_bt.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalmapFromBt>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}