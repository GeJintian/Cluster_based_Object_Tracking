#include "detection/detection.hpp"


int main(int argc, char * argv[])
{
    std::cout<<"Detection functioning..."<<std::endl;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiVehicleDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}