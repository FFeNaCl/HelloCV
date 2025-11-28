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
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace cv;
using namespace rclcpp;

class UnifiedVisionNode : public rclcpp::Node {
public:
    UnifiedVisionNode() : Node("unified_vision_node") {
        RCLCPP_INFO(this->get_logger(), "Initializing UnifiedVisionNode");

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&UnifiedVisionNode::callback_camera, this, std::placeholders::_1));

        pub_ = this->create_publisher<referee_pkg::msg::MultiObject>("/vision/target", 10);

        cv::namedWindow("Unified Detection Result", cv::WINDOW_AUTOSIZE);
        loadNumberTemplates();

        RCLCPP_INFO(this->get_logger(), "UnifiedVisionNode initialized successfully");
    }

    ~UnifiedVisionNode() {
        cv::destroyWindow("Unified Detection Result");
    }

private:
    // 订阅发布
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr pub_;

    // 模板与图像
    std::vector<cv::Mat> number_templates;
    cv::Mat result_image;

    // 存储各类角点（用于发布）
    std::vector<cv::Point2f> armor_points;     // 装甲板角点
    std::vector<cv::Point2f> rect_points;      // 青色矩形角点
    std::vector<cv::Point2f> sphere_points;    // 红色圆环四个定点

    // ==================== 数字模板加载 ====================
    void loadNumberTemplates() {
        number_templates.clear();
        for (int i = 1; i <= 5; ++i) {
            std::string path = "/home/tomori/Vision_Arena_2025/templates/template_" + std::to_string(i) + ".png";
            cv::Mat templ = cv::imread(path, cv::IMREAD_GRAYSCALE);
            if (templ.empty()) {
                RCLCPP_WARN(this->get_logger(), "Failed to load template_%d.png", i);
                continue;
            }
            cv::GaussianBlur(templ, templ, cv::Size(3,3), 0);
            cv::erode(templ, templ, cv::Mat(), cv::Point(-1,-1), 1);
            cv::resize(templ, templ, cv::Size(32,45));
            number_templates.push_back(templ);
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu number templates", number_templates.size());
    }

    // ==================== 数字识别 ====================
    int recognizeNumber(const cv::Mat& armor_roi) {
        if (armor_roi.empty() || number_templates.empty()) return -1;

        cv::Mat gray, bin;
        cv::cvtColor(armor_roi, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, bin, 0, 255, cv::THRESH_OTSU);

        double best_match = 0.3;
        int best_id = -1;

        for (int id = 0; id < (int)number_templates.size(); ++id) {
            const auto& templ = number_templates[id];
            std::vector<double> scales = {0.8, 0.9, 1.0, 1.1, 1.2};
            for (double s : scales) {
                cv::Size sz(cv::saturate_cast<int>(templ.cols * s),
                            cv::saturate_cast<int>(templ.rows * s));
                if (sz.width > bin.cols || sz.height > bin.rows) continue;

                cv::Mat resized;
                cv::resize(templ, resized, sz);
                cv::Mat result;
                cv::matchTemplate(bin, resized, result, cv::TM_CCOEFF_NORMED);
                double max_val;
                cv::minMaxLoc(result, nullptr, &max_val);
                if (max_val > best_match) {
                    best_match = max_val;
                    best_id = id + 1;
                }
            }
        }
        return best_id;
    }

    // ==================== 稳定圆环四点 ====================
    std::vector<cv::Point2f> calculateStableSpherePoints(const cv::Point2f& center, float radius) {
        std::vector<cv::Point2f> pts;
        pts.push_back(cv::Point2f(center.x - radius, center.y));     // 左
        pts.push_back(cv::Point2f(center.x, center.y + radius));     // 下
        pts.push_back(cv::Point2f(center.x + radius, center.y));     // 右
        pts.push_back(cv::Point2f(center.x, center.y - radius));     // 上
        return pts;
    }

    // ==================== 矩形四点排序（左下→右下→右上→左上） ====================
    void sortRectanglePoints(std::vector<cv::Point2f>& pts) {
        if (pts.size() != 4) return;
        std::sort(pts.begin(), pts.end(), [](const Point2f& a, const Point2f& b) {
            return (a.x + a.y) < (b.x + b.y);
        });
        if (pts[0].x > pts[1].x) std::swap(pts[0], pts[1]);
        if (pts[2].x < pts[3].x) std::swap(pts[2], pts[3]);
        std::swap(pts[0], pts[1]);
        std::swap(pts[1], pts[2]);
        std::swap(pts[2], pts[3]);
    }

    // ==================== 检测红色装甲板 ====================
    void detectArmors(const cv::Mat& src) {
        cv::Mat hsv, mask1, mask2, mask;
        cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(156, 100, 100), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel, cv::Point(-1,-1), 1);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::RotatedRect> lights;
        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 10) continue;
            auto r = cv::minAreaRect(c);
            double w = r.size.width, h = r.size.height;
            double ratio = std::max(w, h) / std::min(w, h);
            if (ratio > 1.1 && ratio < 25) lights.push_back(r);
        }

        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armors;
        for (size_t i = 0; i < lights.size(); ++i) {
            for (size_t j = i + 1; j < lights.size(); ++j) {
                auto c1 = lights[i].center, c2 = lights[j].center;
                double dist = cv::norm(c1 - c2);
                if (dist < 15 || dist > 700) continue;
                if (std::abs(c1.y - c2.y) > 60) continue;

                double angle_diff = std::abs(lights[i].angle - lights[j].angle);
                angle_diff = std::min(angle_diff, 180 - angle_diff);
                if (angle_diff > 15) continue;

                if (c1.x < c2.x)
                    armors.emplace_back(lights[i], lights[j]);
                else
                    armors.emplace_back(lights[j], lights[i]);
            }
        }

        for (size_t k = 0; k < armors.size(); ++k) {
            auto [left, right] = armors[k];
            cv::Point2f lpts[4], rpts[4];
            left.points(lpts); right.points(rpts);

            std::vector<cv::Point2f> corners = { lpts[0], rpts[3], rpts[2], lpts[1] };

            // 绘制
            for (int i = 0; i < 4; ++i) {
                cv::line(result_image, corners[i], corners[(i+1)%4], cv::Scalar(0,255,0), 3);
                cv::circle(result_image, corners[i], 8, cv::Scalar(0,0,255), -1);
                cv::putText(result_image, std::to_string(i+1), corners[i] + cv::Point2f(10,-10),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,255,255), 2);
            }

            // ROI + 数字识别
            cv::Rect roi = cv::boundingRect(corners);
            roi.y -= static_cast<int>(roi.height * 0.4);
            roi.height += static_cast<int>(roi.height * 0.8);
            roi.x -= 20; roi.width += 40;
            roi &= cv::Rect(0, 0, src.cols, src.rows);
            if (roi.area() > 0) {
                cv::Mat armor_roi = src(roi);
                int num = recognizeNumber(armor_roi);
                std::string text = num > 0 ? std::to_string(num) : "?";
                cv::putText(result_image, text, cv::Point(roi.x + roi.width/2 - 30, roi.y + 60),
                           cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0,0,255), 5);
                cv::putText(result_image, "Armor" + text, corners[0] + cv::Point2f(-30,-30),
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255), 3);

                // 保存角点
                for (auto& p : corners) armor_points.push_back(p);
            }
        }
    }

    // ==================== 检测青色矩形 ====================
    void detectCyanRectangles(const cv::Mat& src) {
        cv::Mat hsv, mask;
        cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(40, 50, 80), cv::Scalar(100, 255, 255), mask);

        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 600) continue;

            std::vector<cv::Point> approx;
            double peri = cv::arcLength(c, true);
            cv::approxPolyDP(c, approx, 0.02 * peri, true);
            if (approx.size() != 4) continue;

            std::vector<cv::Point2f> pts(4);
            for (int i = 0; i < 4; ++i) pts[i] = approx[i];
            sortRectanglePoints(pts);

            for (int i = 0; i < 4; ++i)
                cv::line(result_image, pts[i], pts[(i+1)%4], cv::Scalar(255,255,0), 4);

            std::vector<cv::Scalar> colors = {cv::Scalar(255,0,0), cv::Scalar(0,255,0),
                                             cv::Scalar(0,255,255), cv::Scalar(255,0,255)};
            for (int i = 0; i < 4; ++i) {
                cv::circle(result_image, pts[i], 10, colors[i], -1);
                cv::circle(result_image, pts[i], 10, cv::Scalar(0,0,0), 3);
                cv::putText(result_image, std::to_string(i+1), pts[i] + cv::Point2f(15,-15),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 3);
                rect_points.push_back(pts[i]);
            }
            cv::putText(result_image, "BASE", pts[0] + cv::Point2f(-40,-40),
                       cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255,255,0), 4);
        }
    }

    // ==================== 检测红色圆环 ====================
    void detectRedSpheres(const cv::Mat& src) {
        cv::Mat hsv, mask1, mask2, mask;
        cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(156, 120, 70), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 500) continue;

            cv::Point2f center; float radius;
            cv::minEnclosingCircle(c, center, radius);
            double circularity = 4 * CV_PI * area / (cv::arcLength(c, true) * cv::arcLength(c, true));

            if (circularity > 0.75 && radius > 15 && radius < 200) {
                auto pts = calculateStableSpherePoints(center, radius);

                cv::circle(result_image, center, (int)radius, cv::Scalar(0,255,255), 5);
                cv::circle(result_image, center, 8, cv::Scalar(0,0,255), -1);

                std::vector<cv::Scalar> colors = {cv::Scalar(255,0,0), cv::Scalar(0,255,0),
                                                 cv::Scalar(0,255,255), cv::Scalar(255,0,255)};
                for (int i = 0; i < 4; ++i) {
                    cv::circle(result_image, pts[i], 10, colors[i], -1);
                    cv::circle(result_image, pts[i], 10, cv::Scalar(0,0,0), 3);
                    cv::putText(result_image, std::to_string(i+1), pts[i] + cv::Point2f(15,-15),
                               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,255,255), 3);
                    sphere_points.push_back(pts[i]);
                }
                cv::putText(result_image, "RING", center + cv::Point2f(-30,-radius-20),
                           cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255), 4);
            }
        }
    }

    // ==================== 主回调 ====================
    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = (msg->encoding == "rgb8") ?
                cv_bridge::toCvCopy(msg, "bgr8") : cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat image = cv_ptr->image;
            if (image.empty()) return;

            result_image = image.clone();
            armor_points.clear();
            rect_points.clear();
            sphere_points.clear();

            detectArmors(image);
            detectCyanRectangles(image);
            detectRedSpheres(image);

            // 显示统计信息
            cv::putText(result_image,
                format("Armor:%zu  Base:%zu  Ring:%zu", armor_points.size()/4, rect_points.size()/4, sphere_points.size()/4),
                cv::Point(30, 70), cv::FONT_HERSHEY_SIMPLEX, 1.8, cv::Scalar(0,255,0), 5);

            cv::imshow("Unified Detection Result", result_image);
            cv::waitKey(1);

            // 发布消息
            referee_pkg::msg::MultiObject target_msg;
            target_msg.header = msg->header;

            auto add_object = [&](const std::string& type, const std::vector<cv::Point2f>& pts) {
                if (pts.size() % 4 != 0) return;
                referee_pkg::msg::Object obj;
                obj.target_type = type;
                for (const auto& p : pts) {
                    geometry_msgs::msg::Point gp;
                    gp.x = p.x; gp.y = p.y; gp.z = 0.0;
                    obj.corners.push_back(gp);
                }
                target_msg.objects.push_back(obj);
            };

            if (!armor_points.empty())   add_object("armor_red", armor_points);
            if (!rect_points.empty())    add_object("base_cyan", rect_points);
            if (!sphere_points.empty())  add_object("ring_red", sphere_points);

            target_msg.num_objects = target_msg.objects.size();
            pub_->publish(target_msg);

            RCLCPP_INFO(this->get_logger(), "Published %zu targets (Armor:%zu Base:%zu Ring:%zu)",
                       target_msg.objects.size(),
                       armor_points.size()/4, rect_points.size()/4, sphere_points.size()/4);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedVisionNode>());
    rclcpp::shutdown();
    return 0;
}