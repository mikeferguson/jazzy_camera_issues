#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class CloudSub : public rclcpp::Node
{
  public:
    CloudSub()
    : Node("camera_hz")
    {
      bool use_best_effort = this->declare_parameter<bool>("best_effort", true);

      if (use_best_effort)
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "head_camera/depth_registered/points", rclcpp::QoS(10).best_effort(), std::bind(&CloudSub::topic_callback, this, _1));
      else
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "head_camera/depth_registered/points", rclcpp::QoS(10), std::bind(&CloudSub::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr)
    {
      if (cloud_count_ == 0)
      {
        first_cloud_ = this->now();
      }
      else
      {
        double elapsed = (this->now() - first_cloud_).seconds();
        std::cout << "Recieved " << cloud_count_ + 1 << " messages at " << 1 / (elapsed / cloud_count_) << " hz" << std::endl;
      }
      ++cloud_count_;
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    int cloud_count_;
    rclcpp::Time first_cloud_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudSub>());
  rclcpp::shutdown();
  return 0;
}
