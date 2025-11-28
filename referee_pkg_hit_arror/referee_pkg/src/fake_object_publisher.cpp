\
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "referee_pkg/msg/object.hpp"

using ObjectMsg = referee_pkg::msg::Object;

class FakeObjectPublisher : public rclcpp::Node
{
public:
  FakeObjectPublisher()
  : Node("fake_object_publisher")
  {
    pub_ = this->create_publisher<ObjectMsg>("object", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FakeObjectPublisher::timerCallback, this));
  }

private:
  void timerCallback()
  {
    ObjectMsg msg;
    msg.target_type = "armor_1";

    geometry_msgs::msg::Point p;

    // 随便构造 4 个像素角点，模拟画面中的装甲板
    // 顺序：左下 -> 左上 -> 右上 -> 右下
    p.x = 600.0; p.y = 500.0; p.z = 0.0;
    msg.corners.push_back(p);

    p.x = 600.0; p.y = 400.0; p.z = 0.0;
    msg.corners.push_back(p);

    p.x = 700.0; p.y = 400.0; p.z = 0.0;
    msg.corners.push_back(p);

    p.x = 700.0; p.y = 500.0; p.z = 0.0;
    msg.corners.push_back(p);

    pub_->publish(msg);
  }

  rclcpp::Publisher<ObjectMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeObjectPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
