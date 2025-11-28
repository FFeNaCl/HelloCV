\
#include <chrono>
#include <memory>
#include <vector>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "referee_pkg/srv/hit_armor.hpp"

using namespace std::chrono_literals;
using HitArmor = referee_pkg::srv::HitArmor;

class HitArrorClient : public rclcpp::Node
{
public:
  HitArrorClient()
  : Node("hit_arror_client")
  {
    client_ = this->create_client<HitArmor>("/referee/hit_arror");

    request_ = std::make_shared<HitArmor::Request>();

    // 构造一个示例装甲板大小：宽 0.2m, 高 0.12m
    double half_w = 0.1;   // 0.2m
    double half_h = 0.06;  // 0.12m

    geometry_msgs::msg::Point p;
    // 左下
    p.x = -half_w; p.y = 0.0; p.z = -half_h;
    request_->modelpoint.push_back(p);
    // 左上
    p.x = -half_w; p.y = 0.0; p.z =  half_h;
    request_->modelpoint.push_back(p);
    // 右上
    p.x =  half_w; p.y = 0.0; p.z =  half_h;
    request_->modelpoint.push_back(p);
    // 右下
    p.x =  half_w; p.y = 0.0; p.z = -half_h;
    request_->modelpoint.push_back(p);

    request_->g = 9.81;

    timer_ = this->create_wall_timer(
      2s, std::bind(&HitArrorClient::timerCallback, this));
  }

private:
  void timerCallback()
  {
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(),
        "HitArror service /referee/hit_arror not available yet...");
      return;
    }

    auto future = client_->async_send_request(
      request_,
      std::bind(&HitArrorClient::responseCallback, this, std::placeholders::_1)
    );
  }

  void responseCallback(rclcpp::Client<HitArmor>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(),
      "Service response: yaw=%.3f, pitch=%.3f, roll=%.3f",
      response->yaw, response->pitch, response->roll);
  }

  rclcpp::Client<HitArmor>::SharedPtr client_;
  std::shared_ptr<HitArmor::Request> request_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HitArrorClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
