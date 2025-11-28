\
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/header.hpp"

#include "referee_pkg/srv/hit_armor.hpp"
#include "referee_pkg/msg/object.hpp"

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

using HitArmor = referee_pkg::srv::HitArmor;
using ObjectMsg = referee_pkg::msg::Object;

class HitArrorServer : public rclcpp::Node
{
public:
  HitArrorServer()
  : Node("hit_arror_server")
  {
    // TODO: 实际使用时，请从参数服务器或 camera_info 话题读取真实相机内参
    double fx = 800.0;
    double fy = 800.0;
    double cx = 640.0;
    double cy = 360.0;

    camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      fx, 0.0, cx,
      0.0, fy, cy,
      0.0, 0.0, 1.0
    );

    // 假设无畸变，如果有畸变，请改为真实参数
    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    // 弹丸初速，单位 m/s（根据实际情况修改）
    projectile_speed_ = 1.5;

    // TODO: 根据你自己的视觉节点实际发布的 topic 名修改这里
    // 当前默认订阅 "object" 话题
    object_sub_ = this->create_subscription<ObjectMsg>(
      "object",   // <<< 按需改成你的真实话题名，比如 "vision/object"
      10,
      std::bind(&HitArrorServer::objectCallback, this, _1)
    );

    // 服务名统一为 /referee/hit_arror
    service_ = this->create_service<HitArmor>(
      "/referee/hit_arror",
      std::bind(&HitArrorServer::handleService, this, _1, _2)
    );

    RCLCPP_INFO(this->get_logger(),
      "HitArror server started on /referee/hit_arror (with ballistic compensation).");
  }

private:
  void objectCallback(const ObjectMsg::SharedPtr msg)
  {
    // 可选：按 target_type 过滤，例如只处理装甲板
    // if (msg->target_type != "armor_1") return;

    if (msg->corners.size() < 4) {
      RCLCPP_WARN(this->get_logger(),
        "Object.corners has less than 4 points, got %zu",
        msg->corners.size());
      have_image_points_ = false;
      return;
    }

    // 只取前 4 个角点：左下 -> 左上 -> 右上 -> 右下
    image_points_.clear();
    for (size_t i = 0; i < 4; ++i) {
      const auto & p = msg->corners[i];
      image_points_.emplace_back(
        static_cast<float>(p.x),   // 像素 u
        static_cast<float>(p.y)    // 像素 v
      );
    }

    have_image_points_ = true;
  }

  void handleService(
    const std::shared_ptr<HitArmor::Request> request,
    std::shared_ptr<HitArmor::Response> response)
  {
    if (!have_image_points_) {
      RCLCPP_WARN(this->get_logger(),
        "No image points yet, cannot solve PnP.");
      response->yaw = 0.0;
      response->pitch = 0.0;
      response->roll = 0.0;
      return;
    }

    if (request->modelpoint.size() < 4) {
      RCLCPP_WARN(this->get_logger(),
        "Request has less than 4 model points.");
      response->yaw = 0.0;
      response->pitch = 0.0;
      response->roll = 0.0;
      return;
    }

    // 3D 模型点：原点为装甲板中心, x 左右, z 上下, y 垂直平面(这里设为 0)
    std::vector<cv::Point3f> object_points;
    object_points.reserve(4);
    for (size_t i = 0; i < 4; ++i) {
      const auto & p = request->modelpoint[i];
      object_points.emplace_back(
        static_cast<float>(p.x),
        0.0f,
        static_cast<float>(p.z)
      );
    }

    cv::Mat rvec, tvec;
    bool ok = cv::solvePnP(
      object_points,
      image_points_,
      camera_matrix_,
      dist_coeffs_,
      rvec,
      tvec,
      false,
      cv::SOLVEPNP_IPPE_SQUARE
    );

    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "solvePnP failed.");
      response->yaw = 0.0;
      response->pitch = 0.0;
      response->roll = 0.0;
      return;
    }

    // 旋转向量 -> 旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 装甲板中心在相机坐标系下的位置 (Xc, Yc, Zc)
    double Xc = tvec.at<double>(0); // 右
    double Yc = tvec.at<double>(1); // 下
    double Zc = tvec.at<double>(2); // 前

    // 1) yaw：水平平面内方位角
    double yaw = std::atan2(Xc, Zc);

    // 2) 弹道补偿：根据位置 + g + 初速，计算所需 pitch
    // OpenCV 相机坐标：X 右，Y 下，Z 前
    // 约定：向上为正，所以高度 H = -Yc
    // 水平距离 D = sqrt(Xc^2 + Zc^2)
    double D = std::sqrt(Xc * Xc + Zc * Zc); // 水平距离
    double H = -Yc;                          // 高度差 (上为正)

    double g = request->g;
    double v = projectile_speed_;

    double v2 = v * v;
    double term = v2 * v2 - g * (g * D * D + 2.0 * H * v2);

    double pitch = 0.0;
    if (term < 0.0) {
      // 目标在当前初速下不可达，退而求其次用几何直线角度
      RCLCPP_WARN(this->get_logger(),
        "Ballistic: target out of range (discriminant < 0), using geometric pitch.");
      pitch = std::atan2(H, D);
    } else {
      double sqrt_disc = std::sqrt(term);
      // 低弹道解
      pitch = std::atan2(v2 - sqrt_disc, g * D);
    }

    // 3) roll 这里先简单置 0，如有需要可从旋转矩阵中提取
    double roll = 0.0;

    response->yaw = yaw;
    response->pitch = pitch;
    response->roll = roll;

    RCLCPP_INFO(this->get_logger(),
      "HitArror result (with ballistic): yaw=%.3f, pitch=%.3f, roll=%.3f",
      yaw, pitch, roll);
  }

  rclcpp::Subscription<ObjectMsg>::SharedPtr object_sub_;
  rclcpp::Service<HitArmor>::SharedPtr service_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  std::vector<cv::Point2f> image_points_;
  bool have_image_points_{false};

  double projectile_speed_; // m/s
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HitArrorServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
