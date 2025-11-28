#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <algorithm> 
#include <vector>
#include <stdexcept>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace cv;
using namespace rclcpp;

class VisionNode : public rclcpp::Node {
 public:
  VisionNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing VisionNode");//输出信息a
    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&VisionNode::callback_carema, this, std::placeholders::_1));
    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);
    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "VisionNode initialized successfully");//输出信息b
  }
  ~VisionNode() { cv::destroyWindow("Detection Result"); }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;

    vector<Point2f> rectangle_points;   // 所有矩形目标的角点
    vector<Point2f> sphere_points; // 所有圆形目标的四个定点
    cv::Mat result_image;//输出图像

    // 稳定计算圆的四个点
    vector<Point2f> calculateStableSpherePoints(const Point2f &center, float radius);

    // 矩形四个点排序
    void sortRectanglePoints(vector<Point2f>& pts);
    //主要实现函数
    void detectRectangles(const cv::Mat& src);
    void detectSpheres(const cv::Mat& src);
    void callback_carema(sensor_msgs::msg::Image::SharedPtr msg);
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionNode>("VisionNode");
  RCLCPP_INFO(node->get_logger(), "Starting VisionNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
vector<Point2f> VisionNode::calculateStableSpherePoints(const Point2f &center, float radius) {
    vector<Point2f> points;
    // 简单稳定的几何计算，避免漂移
    // 左、下、右、上
    points.push_back(Point2f(center.x - radius, center.y));  // 左点 (1)
    points.push_back(Point2f(center.x, center.y + radius));  // 下点 (2)
    points.push_back(Point2f(center.x + radius, center.y));  // 右点 (3)
    points.push_back(Point2f(center.x, center.y - radius));  // 上点 (4)
    return points;
}
void VisionNode::sortRectanglePoints(vector<Point2f>& pts) {
    if (pts.size() != 4) return;
    // 按 x+y 升序排序（左下角通常 x+y 最小，右上角 x+y 最大）
    std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return (a.x + a.y) < (b.x + b.y);
    });
    // 现在 pts[0] 和 pts[1] 是「下侧」两个点（y 较大）
    //      pts[2] 和 pts[3] 是「上侧」两个点（y 较小）
    // Step 2: 下侧两个点按 x 坐标排序：x小的在左 → 左下
    if (pts[0].x > pts[1].x) {
        std::swap(pts[0], pts[1]);
    }
    // 现在 pts[0] = 左下, pts[1] = 右下
    // Step 3: 上侧两个点按 x 坐标排序：x大的在右 → 右上
    if (pts[2].x < pts[3].x) {
        std::swap(pts[2], pts[3]);
    }
    std::swap(pts[0], pts[1]);
    std::swap(pts[1], pts[2]);
    std::swap(pts[2], pts[3]);
}
void VisionNode::detectRectangles(const cv::Mat& src) {
    //青色
    cv::Mat hsv, mask;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(40, 50, 80), cv::Scalar(100, 255, 255), mask);
    //开运算闭运算
    auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    //轮廓
    vector<vector<Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //筛选较小的
    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 600) continue;

        vector<Point> approx;
        double peri = cv::arcLength(c, true);
        cv::approxPolyDP(c, approx, 0.02 * peri, true);//经典多边形趋近

        if (approx.size() != 4) continue;

        vector<Point2f> pts(4);
        for (int i = 0; i < 4; ++i) pts[i] = approx[i];

        sortRectanglePoints(pts);
        // 画框
        for (int i = 0; i < 4; ++i) {
            cv::line(result_image, pts[i], pts[(i+1)%4], cv::Scalar(0,255,0), 3);}
        vector<cv::Scalar> point_colors = {
            cv::Scalar(255, 0, 0),    // 蓝色 - 左下
            cv::Scalar(0, 255, 0),    // 绿色 - 右下
            cv::Scalar(0, 255, 255),  // 黄色 - 右上
            cv::Scalar(255, 0, 255)   // 紫色 - 左上
        };
        for (int i = 0; i < 4; i++) {
            cv::circle(result_image, pts[i], 6, point_colors[i], -1);
            cv::circle(result_image, pts[i], 6, cv::Scalar(0, 0, 0), 2);
            cv::putText(result_image, to_string(i+1), cv::Point(pts[i].x + 10, pts[i].y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 3);//白色描字
            cv::putText(result_image, to_string(i+1), cv::Point(pts[i].x + 10, pts[i].y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[i], 2);//数字标号
            rectangle_points.push_back(pts[i]);
        }

        cv::putText(result_image, "RECT", pts[0] + Point2f(-20,-20),
                   cv::FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0,255,255), 3);
    }
}

void VisionNode::detectSpheres(const cv::Mat& src) {
    cv::Mat hsv, mask1, mask2, mask;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv, cv::Scalar(0, 120, 70),   cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(156, 120, 70), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;
    auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);

    vector<vector<Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 500) continue;//舍弃小圆，需要调
        // 计算最小外接圆
        Point2f center;
        float radius;//半径radius
        minEnclosingCircle(c, center, radius);

        double peri = arcLength(c, true);
        double circularity = 4 * CV_PI * area / (peri * peri);// 计算圆形度circularity

        if (circularity > 0.75 && radius > 15 && radius < 200) {//这里数据可以调
        
            auto pts = calculateStableSpherePoints(center, radius);
            // 绘制检测到的球体，result_image是image复制版
            cv::circle(result_image, center, (int)radius, Scalar(0,255,0), 3);
            cv::circle(result_image, center, 5, Scalar(0,0,255), -1);
            // 绘制球体上的四个点
            vector<cv::Scalar> point_colors = {
            cv::Scalar(255, 0, 0),    // 蓝色 - 左
            cv::Scalar(0, 255, 0),    // 绿色 - 下
            cv::Scalar(0, 255, 255),  // 黄色 - 右
            cv::Scalar(255, 0, 255)   // 紫色 - 上
            };

            for (int i = 0; i < 4; i++) {
                cv::circle(result_image, pts[i], 6, point_colors[i], -1);
                cv::circle(result_image, pts[i], 6, cv::Scalar(0, 0, 0), 2);

                cv::putText(result_image, to_string(i+1), cv::Point(pts[i].x + 10, pts[i].y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255), 3);//白色描字
                cv::putText(result_image, to_string(i+1), cv::Point(pts[i].x + 10, pts[i].y - 10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[i], 2);//数字标号
                sphere_points.push_back(pts[i]);//添加到发送列表
            } 

            // 显示半径信息，输出半径大小
            string info_text = "R:" + to_string((int)radius);
            cv::putText(
            result_image, info_text, cv::Point(center.x - 15, center.y + 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        }
    }
}

void VisionNode::callback_carema(sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 图像转换形式
        cv_bridge::CvImagePtr cv_ptr;
        if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
            cv::Mat image(msg->height, msg->width, CV_8UC3,
                          const_cast<unsigned char *>(msg->data.data()));
            cv::Mat bgr_image;
            cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
            cv_ptr = std::make_shared<cv_bridge::CvImage>();
            cv_ptr->header = msg->header;
            cv_ptr->encoding = "bgr8";
            cv_ptr->image = bgr_image;
        } else {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        cv::Mat image = cv_ptr->image;
        if (image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image");
            return;
        }

        result_image = image.clone();
        rectangle_points.clear();
        sphere_points.clear();
        //调用函数
        detectRectangles(image);
        detectSpheres(image);

        cv::putText(result_image, "RECT:" + to_string(rectangle_points.size()/4) + 
                   "  SPHERE:" + to_string(sphere_points.size()/4),
                   cv::Point(30, 60), cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,255,0), 3);

        cv::imshow("Detection Result", result_image);
        cv::waitKey(1);

        //只发布一次,包含两种目标
        referee_pkg::msg::MultiObject msg_object;
        msg_object.header = msg->header;
        // 先发矩形
        if (!rectangle_points.empty()) {
            referee_pkg::msg::Object obj;
            obj.target_type = "rectangle";
            for (const auto& p : rectangle_points) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.0;
                obj.corners.push_back(pt);
            }
            msg_object.objects.push_back(obj);//添加到发送列表
        }
        // 再发圆形
        if (!sphere_points.empty()) {
            referee_pkg::msg::Object obj;
            obj.target_type = "sphere";
            for (const auto& p : sphere_points) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x; pt.y = p.y; pt.z = 0.0;
                obj.corners.push_back(pt);
            }
            msg_object.objects.push_back(obj);//添加到发送列表
        }

        msg_object.num_objects = msg_object.objects.size();  

        Target_pub->publish(msg_object);//发布消息
        RCLCPP_INFO(this->get_logger(), 
                   "Published %zu targets (RECT:%zu SPHERE:%zu)", 
                   msg_object.objects.size(),
                   rectangle_points.size()/4, sphere_points.size()/4);
    } 
    //记录异常
      catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}