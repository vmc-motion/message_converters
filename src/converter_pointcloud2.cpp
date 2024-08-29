#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rcpputils/asserts.hpp"

using std::placeholders::_1;

// Converts all data fields of a PointCloud2 from Float64 to Float32.
// Assumes data is a one dimensional array, little endian, only Float64 fields, and densily packed.
// 
// Run for example from command line like this:
//   ros2 run message_converters converter_pointcloud2 --ros-args -p input:=some_topic -p output:=converted_topic
//
class PointCloud2Converter : public rclcpp::Node
{
  public:
  
    PointCloud2Converter()
    : Node("converter_pointcloud2")
    {
      declare_parameter(input_topic_arg_name_, "input_topic");
      declare_parameter(output_topic_arg_name_, "output_topic");
      
      std::string input_topic_name = get_parameter(input_topic_arg_name_).as_string();
      std::string output_topic_name = get_parameter(output_topic_arg_name_).as_string();
      
      subscription_ = this->create_subscription<PointCloud2>(
        input_topic_name, 10, std::bind(&PointCloud2Converter::topic_callback, this, _1));
        
      publisher_ = create_publisher<PointCloud2>(
        output_topic_name, 10);
    }

  private:
    
    using PointCloud2 = sensor_msgs::msg::PointCloud2;
  
    void topic_callback(const PointCloud2 &msg_in) const
    {
      
      RCLCPP_INFO(get_logger(), "Converting pointcloud2 from topic '%s' to '%s'",
        subscription_->get_topic_name(), publisher_->get_topic_name());
      
      // Only allow one dimensional data
      if (msg_in.height != 1)
      {
        RCLCPP_ERROR(get_logger(), "Failed to convert pointcloud2 because height (%i) is not 1.", msg_in.height);      
        return;
      }
      
      // Only allow little endian
      if (msg_in.is_bigendian)
      {
        RCLCPP_ERROR(get_logger(), "Failed to convert pointcloud2 because is_bigendian is true.");
        return;
      }
      
      // Verify data size
      if(msg_in.data.size() != msg_in.point_step * msg_in.width)
      {
        RCLCPP_ERROR(get_logger(), "Failed to convert pointcloud2 because data has unexpected size.");
        return;
      }
      
      sensor_msgs::msg::PointCloud2 msg_out = msg_in;
      
      msg_out.header = msg_in.header;
      msg_out.height = msg_in.height;
      msg_out.width = msg_in.width;
      msg_out.is_bigendian = msg_in.is_bigendian;
      msg_out.point_step = msg_in.point_step / 2;
      msg_out.row_step = msg_in.row_step / 2;
      msg_out.is_dense = msg_in.is_dense;
      msg_out.fields = msg_in.fields;
      for (size_t i = 0; i < msg_out.fields.size(); ++i)
      {
        bool was_float64 = msg_out.fields[i].datatype == sensor_msgs::msg::PointField::FLOAT64;
        if (!was_float64)
        {
          RCLCPP_ERROR(get_logger(), "Failed to convert pointcloud2 because some fields are not FLOAT64.");
          return;
        }
        
        msg_out.fields[i].offset /= 2;
        msg_out.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      }
      
      int num_points = msg_in.data.size() / msg_in.point_step;
      int num_fields = msg_in.data.size() / sizeof(double);
      msg_out.data.resize(num_fields * sizeof(float));
      for (size_t f = 0; f < num_fields; ++f)
      {
        const uint8_t* input_data_ptr = &msg_in.data[f * sizeof(double)];
        double input_data = 0;
        std::memcpy(&input_data, input_data_ptr, sizeof(double));
        
        uint8_t* output_data_ptr = &msg_out.data[f * sizeof(float)];
        float output_data = (float)input_data;
        std::memcpy(output_data_ptr, &output_data, sizeof(float));
        
      }
      
      publisher_->publish(msg_out);
    }
    
    // Subscriber and Publisher
    rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
    
    // Parameter names
    std::string input_topic_arg_name_{"input"};
    std::string output_topic_arg_name_{"output"};
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2Converter>());
  rclcpp::shutdown();
  return 0;
}
