#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std;
using namespace rclcpp;
using namespace cv;

class TestNode : public rclcpp::Node {
 public:
  TestNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing Ring Detection Node");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&TestNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Ring Detection", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(this->get_logger(), "Ring Detection Node initialized");
  }

  ~TestNode() { cv::destroyWindow("Ring Detection"); }

 private:
  // 稳定取圆周上四个方位点：左、下、右、上（对应1,2,3,4）
  vector<Point2f> calculateStableCirclePoints(const Point2f &center, float radius) {
    vector<Point2f> points;
    points.push_back(Point2f(center.x - radius, center.y));     // 左
    points.push_back(Point2f(center.x, center.y + radius));     // 下
    points.push_back(Point2f(center.x + radius, center.y));     // 右
    points.push_back(Point2f(center.x, center.y - radius));     // 上
    return points;
  }

  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;  // 存放所有有效点的容器
};
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestNode>("ring_detector");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
void TestNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr;
    if (msg->encoding == "rgb8") {
      cv::Mat rgb_image(msg->height, msg->width, CV_8UC3,
                        const_cast<unsigned char *>(msg->data.data()));
      cv::Mat bgr_image;
      cvtColor(rgb_image, bgr_image, cv::COLOR_RGB2BGR);
      cv_ptr = std::make_shared<cv_bridge::CvImage>();
      cv_ptr->header = msg->header;
      cv_ptr->encoding = "bgr8";
      cv_ptr->image = bgr_image;
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    cv::Mat image = cv_ptr->image.clone();
    if (image.empty()) return;
    cv::Mat result = image.clone();
    // 1. HSV 提取红色区域
    cv::Mat hsv;
    cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask1, mask2, mask;
    inRange(hsv, Scalar(0, 100, 70), Scalar(10, 255, 255), mask1);
    inRange(hsv, Scalar(160, 100, 70), Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(mask, mask, MORPH_CLOSE, kernel);
    morphologyEx(mask, mask, MORPH_OPEN, kernel);
    // 2. 找所有轮廓（包括内外）
    vector<vector<Point>> contours;
    vector<Vec4i> h;
    findContours(mask, contours, h, RETR_TREE, CHAIN_APPROX_SIMPLE);
    Point_V.clear();
    // 3. 遍历所有轮廓，寻找圆环
    for (size_t i = 0; i < contours.size(); i++) {
      // 必须有子轮廓才可能是圆环
      if (h[i][2] == -1) continue;
      // 取外轮廓
      vector<Point> outer = contours[i];
      double area1 = contourArea(outer);
      if (area1 < 800) continue;
      // 拟合最小外接圆（外圆）
      Point2f center;
      float radius;
      minEnclosingCircle(outer, center, radius);
      if (radius < 20 || radius > 300) continue;
      // 计算圆形度
      double perimeter = arcLength(outer, true);
      double circularity = 4 * CV_PI * area1 / (perimeter * perimeter);
      if (circularity < 0.75) continue;
      // 检查内轮廓（子轮廓）是否也接近圆形且同心
      int child_idx = h[i][2];
      bool has_good_inner = false;
      Point2f center_inner;
      float radius_inner;
      while (child_idx != -1) {
        vector<Point> inner = contours[child_idx];
        double area_inner = contourArea(inner);
        minEnclosingCircle(inner, center_inner, radius_inner);
        float center_dist = norm(center - center_inner);
        float radius_ratio = radius_inner / radius;
        // 内圆要足够大且与外圆基本同心
        if (area_inner > 200 && center_dist < radius * 0.3 && radius_ratio > 0.3 && radius_ratio < 0.8) {
          has_good_inner = true;
          break;
        }
        child_idx = h[child_idx][0];  // 下一个兄弟内轮廓
      }
      if (!has_good_inner) continue;

      // 取外圆的四个稳定点
      vector<Point2f> ring_points = calculateStableCirclePoints(center, radius);
      vector<Point2f> inner_ring_points = calculateStableCirclePoints(center_inner, radius_inner);

      // 绘制外圆（绿色）
      circle(result, center, (int)radius, Scalar(0, 255, 0), 3);
      circle(result, center, 4, Scalar(0, 0, 255), -1);  // 红心

      // 绘制四个方位点
      vector<string> names = {"左", "下", "右", "上"};
      vector<Scalar> colors = {Scalar(255,0,0), Scalar(0,255,0), 
                               Scalar(0,255,255), Scalar(255,0,255)};

      for (int j = 0; j < 4; j++) {
        circle(result, ring_points[j], 8, colors[j], -1);
        circle(result, ring_points[j], 8, Scalar(0,0,0), 2);
        putText(result, to_string(j+1), 
                Point(ring_points[j].x + 12, ring_points[j].y - 12),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,255,255), 2);
        Point_V.push_back(ring_points[j]);
      }

      for (int j = 0; j < 4; j++) {
        circle(result, inner_ring_points[j], 8, colors[j], -1);
        circle(result, inner_ring_points[j], 8, Scalar(0,0,0), 2);
        putText(result, to_string(j+1), 
                Point(inner_ring_points[j].x + 12, ring_points[j].y - 12),
                FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255,255,255), 2);

        Point_V.push_back(inner_ring_points[j]);
      }
      // 显示半径
      putText(result, "R:" + to_string((int)radius), 
              Point(center.x - 20, center.y), 
              FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 2);

      RCLCPP_INFO(this->get_logger(), "center(%.1f,%.1f) R=%.1f", center.x, center.y, radius);
    }
    imshow("Ring Detection", result);
    waitKey(1);
    // 发布消息
    referee_pkg::msg::MultiObject multi_obj;
    multi_obj.header = msg->header;
    multi_obj.num_objects = Point_V.size() / 4;

    for (int i = 0; i < multi_obj.num_objects; i++) {
      referee_pkg::msg::Object obj;
      obj.target_type = "Ring_red"; 
      for (int j = 0; j < 4; j++) {
        int idx = i * 4 + j;
        geometry_msgs::msg::Point p;
        p.x = Point_V[idx].x;
        p.y = Point_V[idx].y;
        p.z = 0.0;
        obj.corners.push_back(p);
      }
      multi_obj.objects.push_back(obj);
    }

    Target_pub->publish(multi_obj);
    RCLCPP_INFO(this->get_logger(), "Published %d circles", multi_obj.num_objects);

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}