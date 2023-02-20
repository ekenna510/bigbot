#include "rclcpp/rclcpp.hpp"
class Dummy : public rclcpp::Node
{
public:
    Dummy() : Node("dummy")
    {

  
    RCLCPP_INFO(this->get_logger(), "dummy starting");
    }
};
int main(int argc, char **argv)
{

  //Initialize ROS
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Dummy>();
    rclcpp::spin(node);
    rclcpp::shutdown();
   // moved to destrcutor
   //if (node->UseSlave)
   ///	{
   /// node->CloseSerial();
   /// }
   /// RCLCPP_INFO(node->get_logger(), "r2Serial stopping");

   
    return 0;
}