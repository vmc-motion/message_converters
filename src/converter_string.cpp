#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

// Run for example from command line like this:
//   ros2 run message_converters converter_string --ros-args -p input:=some_topic -p output:=converted_topic

class StringConverter : public rclcpp::Node
{
  public:
  
    StringConverter()
    : Node("converter_string")
    {
      declare_parameter(input_topic_arg_name_, "input_topic");
      declare_parameter(output_topic_arg_name_, "output_topic");
      
      std::string input_topic_name = get_parameter(input_topic_arg_name_).as_string();
      std::string output_topic_name = get_parameter(output_topic_arg_name_).as_string();
      
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        input_topic_name, 10, std::bind(&StringConverter::topic_callback, this, _1));
        
      publisher_ = create_publisher<std_msgs::msg::String>(
        output_topic_name, 10);
    }

  private:
  
    void topic_callback(const std_msgs::msg::String &msg) const
    {
      std_msgs::msg::String msg_converted = msg;
      msg_converted.data = msg.data + "_converted";
      
      // TODO: Do some more interesting string convertion.
      
      RCLCPP_INFO(get_logger(), "Input: '%s' Output: '%s'", msg.data.c_str(), msg_converted.data.c_str());
      
      publisher_->publish(msg_converted);
    }
    
    // Subscriber and Publisher
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    
    // Parameter names
    std::string input_topic_arg_name_{"input"};
    std::string output_topic_arg_name_{"output"};
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringConverter>());
  rclcpp::shutdown();
  return 0;
}
