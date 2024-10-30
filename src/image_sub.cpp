#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class ImageSub : public rclcpp::Node
{
  public:
    ImageSub()
    : Node("image_sub")
    {
      bool use_best_effort = this->declare_parameter<bool>("best_effort", true);

      if (use_best_effort)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          "image_raw", rclcpp::QoS(10).best_effort(), std::bind(&ImageSub::topic_callback, this, _1));
      else
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
          "image_raw", rclcpp::QoS(10), std::bind(&ImageSub::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr)
    {
      if (image_count_ == 0)
      {
        first_image_ = this->now();
      }
      else
      {
        double elapsed = (this->now() - first_image_).seconds();
        std::cout << "Recieved " << image_count_ + 1 << " messages at " << 1 / (elapsed / image_count_) << " hz" << std::endl;
      }
      ++image_count_;
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    int image_count_;
    rclcpp::Time first_image_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSub>());
  rclcpp::shutdown();
  return 0;
}
