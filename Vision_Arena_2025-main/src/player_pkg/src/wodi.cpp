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
using namespace rclcpp;
using namespace cv;

class WodiNode : public rclcpp::Node {
public:
    WodiNode(string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Initializing WodiNode");

        Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            bind(&WodiNode::callback_camera, this, std::placeholders::_1));

        Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
            "/vision/target", 10);

        cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

        // 关键：构造函数里加载数字模板
        loadNumberTemplates();

        RCLCPP_INFO(this->get_logger(), "WodiNode initialized successfully");
    }

    ~WodiNode() { 
        cv::destroyWindow("Detection Result"); 
    }

private:
    // 成员变量
    vector<Point2f> Point_V;
    std::vector<cv::Mat> number_templates;
    cv::Mat result_image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;

    // 函数声明
    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
    void loadNumberTemplates();
    int recognizeNumber(const cv::Mat& armor_roi);
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WodiNode>("WodiNode");
    RCLCPP_INFO(node->get_logger(), "Starting WodiNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//模板加载
void WodiNode::loadNumberTemplates() {
    number_templates.clear();
    for (int i = 1; i <= 5; i++) {
        std::string path = "/home/tomori/Vision_Arena_2025/templates/template_" + std::to_string(i) + ".png";
        cv::Mat templ = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (templ.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load template_%d.png", i);
            continue;
        }

        // 关键处理：模拟真实数字的模糊和边缘衰减
        cv::GaussianBlur(templ, templ, cv::Size(3,3), 0);  // 轻微模糊
        cv::erode(templ, templ, cv::Mat(), cv::Point(-1,-1), 1);  // 轻微腐蚀，防止过匹配

        cv::resize(templ, templ, cv::Size(32, 45));
        
        // 可选：再加一次小模糊，让边缘更自然
        cv::GaussianBlur(templ, templ, cv::Size(3,3), 0.8);

        number_templates.push_back(templ);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu enhanced number templates", number_templates.size());
}
//数字识别
int WodiNode::recognizeNumber(const cv::Mat& armor_roi) {
    if (armor_roi.empty() || number_templates.empty()) return -1;

    cv::Mat gray;
    cv::cvtColor(armor_roi, gray, cv::COLOR_BGR2GRAY);
    cv::Mat bin;
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 自动反转（解决黑底白字 vs 白底黑字）
    if (cv::countNonZero(bin) > bin.total() / 2) {
        cv::bitwise_not(bin, bin);
    }

    double best_score = 0.42;
    int best_id = -1;

    for (int id = 0; id < number_templates.size(); ++id) {
        cv::Mat templ = number_templates[id].clone();
        if (cv::countNonZero(templ) < templ.total() / 2) {
            cv::bitwise_not(templ, templ);
        }

        for (double scale : {0.5, 0.6, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0, 
                     1.05, 1.1, 1.15, 1.2, 1.25, 1.3, 1.4, 1.5}) {
            int w = cv::saturate_cast<int>(templ.cols * scale);
            int h = cv::saturate_cast<int>(templ.rows * scale);

            if (w > bin.cols || h > bin.rows || w < 15 || h < 15) continue;

            cv::Mat resized;
            cv::resize(templ, resized, cv::Size(w, h));
            cv::Mat result;
            
            cv::matchTemplate(bin, resized, result, cv::TM_CCOEFF_NORMED);
            double max_val;
            cv::minMaxLoc(result, nullptr, &max_val);
            // RCLCPP_INFO(this->get_logger(),"MATCH%i",id);//输出字
            // RCLCPP_INFO(this->get_logger(),"MATCHTHRESHOLD:%lf",max_val);//输出

            if (max_val > best_score) {
                best_score = max_val;
                
                best_id = id + 1;
            }
        }
    }
    
    return best_id;  // 直接返回，不再判断阈值（或者 return best_score > 0.42 ? best_id : -1）
}

// 主回调函数 
void WodiNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 图像转换
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
        Point_V.clear();

        //红色灯条提取
        cv::Mat hsv, mask1, mask2, mask;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 40, 60),    cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(156, 40, 60),  cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        // 原有 mask = mask1 | mask2; 后面
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 2);

        // 新增：轻微膨胀帮助连接断开的灯条（远距离特别有效）
        cv::dilate(mask, mask, kernel, cv::Point(-1,-1), 2);

        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel, cv::Point(-1,-1), 1);

        //找轮廓 + 筛选灯条
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::RotatedRect> light_blobs;//存矩形
        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 1) continue;//这个要改的，不然有的小东西检查不出来

            cv::RotatedRect r = cv::minAreaRect(c);
            double w = r.size.width, h = r.size.height;
            double ratio = max(w, h) / min(w, h);//不知道宽高哪个长
            if (ratio > 1.05 && ratio < 40) {//细长的
                light_blobs.push_back(r);
            }
        }
        //灯条配对 —— 改进版
//灯条配对 —— 超级宽容版（肉眼能看到的基本都能过）
std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armors;
std::vector<bool> used(light_blobs.size(), false);

for (size_t i = 0; i < light_blobs.size(); ++i) {
    if (used[i]) continue;

    double best_score = 1e9;
    int best_j = -1;

    for (size_t j = i + 1; j < light_blobs.size(); ++j) {
        if (used[j]) continue;

        auto& l1 = light_blobs[i];
        auto& l2 = light_blobs[j];

        cv::Point2f c1 = l1.center, c2 = l2.center;
        double dx = std::abs(c1.x - c2.x);
        double dy = std::abs(c1.y - c2.y);
        double dist = cv::norm(c1 - c2);

        // 距离：近到10，远到1200都行
        if (dist < 10 || dist > 1200) continue;

        // 高度差放宽到 180 像素！（远距离两个灯条经常差100+）
        if (dy > 30) continue;

        // 角度差放宽到 25°（很多灯条倾斜严重）
        double angle_diff = std::abs(l1.angle - l2.angle);
        angle_diff = std::min(angle_diff, 180 - angle_diff);
        if (angle_diff > 15.0) continue;

        // 水平距离不能太小（避免同一个灯条被拆）
        if (dx < 15) continue;

        // 评分：主要看水平距离近 + 高度差小 + 角度平行
        double score = dx * 1.0 + dy * 1.5 + angle_diff * 8.0;

        if (score < best_score) {
            best_score = score;
            best_j = j;
        }
    }

    if (best_j != -1) {
        used[i] = true;
        used[best_j] = true;
        if (light_blobs[i].center.x < light_blobs[best_j].center.x)
            armors.emplace_back(light_blobs[i], light_blobs[best_j]);
        else
            armors.emplace_back(light_blobs[best_j], light_blobs[i]);
    }
}
        //处理每个装甲板
            
            referee_pkg::msg::MultiObject target_msg;
            target_msg.header = msg->header;//这里放在循环外

                    //处理每个装甲板
            for (size_t i = 0; i < armors.size(); ++i) {
                auto [left_light, right_light] = armors[i];

                cv::Point2f lpts[4], rpts[4];
                left_light.points(lpts);
                right_light.points(rpts);

                // // ========== 彻底稳健的上下端点提取（关键修改）==========
                // // 左灯条：找出 y 最小的（顶部）和 y 最大的（底部）
                // int l_top_idx = 0, l_bottom_idx = 0;
                // for (int k = 1; k < 4; ++k) {
                //     if (lpts[k].y < lpts[l_top_idx].y)    l_top_idx = k;
                //     if (lpts[k].y > lpts[l_bottom_idx].y) l_bottom_idx = k;
                // }

                // // 右灯条同理
                // int r_top_idx = 0, r_bottom_idx = 0;
                // for (int k = 1; k < 4; ++k) {
                //     if (rpts[k].y < rpts[r_top_idx].y)    r_top_idx = k;
                //     if (rpts[k].y > rpts[r_bottom_idx].y) r_bottom_idx = k;
                // }

                // cv::Point2f left_top     = lpts[l_top_idx];
                // cv::Point2f left_bottom  = lpts[l_bottom_idx];
                // cv::Point2f right_top    = rpts[r_top_idx];
                // cv::Point2f right_bottom = rpts[r_bottom_idx];

                // // ========== 可选：如果左右灯条上下颠倒，强制纠正（极少发生）==========
                // if (left_light.center.y - right_light.center.y > 50) {  // 左灯条明显比右灯条低
                //     std::swap(left_top, left_bottom);
                //     std::swap(right_top, right_bottom);
                // }

                // // ========== 最终标准化顺序：左下 → 右下 → 右上 → 左上 ==========
                // std::vector<cv::Point2f> armor_corners = {
                //     left_bottom,   // 1: 左下
                //     right_bottom,  // 2: 右下
                //     right_top,     // 3: 右上
                //     left_top       // 4: 左上
                // };


            std::vector<cv::Point2f> armor_corners = {
                lpts[2],  // 左灯条下
                rpts[3],  // 右灯条下
                rpts[0],  // 右灯条上
                lpts[1]   // 左灯条上
            };

            // 画装甲板四条边 + 四个角点
            for (int k = 0; k < 4; k++) {
                cv::line(result_image, armor_corners[k], armor_corners[(k+1)%4], cv::Scalar(0,255,0), 2);
                cv::circle(result_image, armor_corners[k], 6, cv::Scalar(0,0,255), 2);

                cv::putText(result_image, to_string(k + 1),
                cv::Point(armor_corners[k].x + 10, armor_corners[k].y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2);//数字标号

                RCLCPP_INFO(this->get_logger(),
                      "Armor %ld, Point %d : (%.2f, %.2f)",
                      i + 1, k + 1,armor_corners[k].x, armor_corners[k].y);//输出字和坐标
            }

                // ROI —— 改进版，向上扩展更多，向下也适当留余量
                cv::Rect roi = cv::boundingRect(armor_corners);
                
                int expand_up    = static_cast<int>(roi.height * 0.6);   // 向上多扩
                int expand_down  = static_cast<int>(roi.height * 0.4);
                int expand_side  = static_cast<int>(roi.width  * 0.4);   // 左右也多扩一点

                roi.y -= expand_up;
                roi.height += expand_up + expand_down;
                roi.x -= expand_side;
                roi.width += 2 * expand_side;

                // 严格防止越界
                roi.x = std::max(0, roi.x);
                roi.y = std::max(0, roi.y);
                roi.width  = std::min(roi.width,  image.cols - roi.x);
                roi.height = std::min(roi.height, image.rows - roi.y);

                if (roi.area() <= 0) continue; // 无效ROI直接跳过

                cv::Mat armor_roi = image(roi).clone();

                int number = recognizeNumber(armor_roi);

            // 显示识别到的数字
            cv::putText(result_image, to_string(number),
                        cv::Point(roi.x + roi.width/2 - 30, roi.y + 50),
                        cv::FONT_HERSHEY_SIMPLEX, 1.8, cv::Scalar(0, 0, 255), 4);
            

            // 填充消息
            
            referee_pkg::msg::Object obj;
            obj.target_type = "armor_red_"+ std::to_string(number);

            for (const auto& pt : armor_corners) {
                geometry_msgs::msg::Point p;
                p.x = pt.x;
                p.y = pt.y;
                p.z = 0.0;
                obj.corners.push_back(p);
                Point_V.push_back(pt);
            }
            target_msg.objects.push_back(obj);
        }

        target_msg.num_objects = Point_V.size() / 4;
        Target_pub->publish(target_msg);
        // 显示
            cv::putText(result_image, "Targets: " + std::to_string(armors.size()),
                        cv::Point(30, 60), cv::FONT_HERSHEY_SIMPLEX, 1.5,
                        cv::Scalar(0, 255, 0), 4);
            cv::imshow("Detection Result", result_image);
            cv::waitKey(1);

        RCLCPP_INFO(this->get_logger(), "Detected %zu armors", armors.size());

    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}
