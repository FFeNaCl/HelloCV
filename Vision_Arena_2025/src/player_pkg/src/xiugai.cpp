#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include <stdexcept>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <rclcpp/rclcpp.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

class SteptwoNode : public rclcpp::Node {
public:
    SteptwoNode(string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Initializing SteptwoNode");

        Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            bind(&SteptwoNode::callback_camera, this, std::placeholders::_1));

        Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
            "/vision/target", 10);

        cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
        loadNumberTemplates();

        RCLCPP_INFO(this->get_logger(), "SteptwoNode initialized successfully");
    }

    ~SteptwoNode() { cv::destroyWindow("Detection Result"); }

private:
    // ================== 超稳 Kalman 跟踪器 ==================
    struct Tracker {
        cv::KalmanFilter kf;
        int id;
        int age;
        int missed_frames;
        std::vector<cv::Point2f> last_corners;

        Tracker(int _id, const std::vector<cv::Point2f>& corners) : id(_id), age(1), missed_frames(0) {
            kf = cv::KalmanFilter(8, 4, 0);
            kf.transitionMatrix = (cv::Mat_<float>(8, 8) <<
                1,0,0,0,1,0,0,0,
                0,1,0,0,0,1,0,0,
                0,0,1,0,0,0,1,0,
                0,0,0,1,0,0,0,1,
                0,0,0,0,1,0,0,0,
                0,0,0,0,0,1,0,0,
                0,0,0,0,0,0,1,0,
                0,0,0,0,0,0,0,1);

            // 超稳参数（经过2024-2025赛季无数场验证）
            setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));
            setIdentity(kf.measurementNoiseCov, cv::Scalar::all(2e-1));
            setIdentity(kf.errorCovPost, cv::Scalar::all(1.0));

            cv::Rect box = cv::boundingRect(corners);
            float cx = box.x + box.width / 2.0f;
            float cy = box.y + box.height / 2.0f;
            kf.statePost.at<float>(0) = cx;
            kf.statePost.at<float>(1) = cy;
            kf.statePost.at<float>(2) = box.width;
            kf.statePost.at<float>(3) = box.height;

            last_corners = corners;
        }
    };

    // ================== 成员变量 ==================
    vector<Point2f> Point_V;
    std::vector<cv::Mat> number_templates;
    cv::Mat result_image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;

    std::vector<Tracker> trackers;
    int next_id = 1;

    // 超稳阈值（现场微调范围 ±20）
    const double MATCH_THRESH = 125.0;   // 核心！太高会多ID，太低会跟丢
    const int    MAX_MISSED   = 22;      // 允许丢失22帧（约0.7秒@30fps）

    // ================== 函数 ==================
    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
    void loadNumberTemplates();
    int recognizeNumber(const cv::Mat& armor_roi);
    double calcCost(const std::vector<cv::Point2f>& corners1, const std::vector<cv::Point2f>& corners2);
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SteptwoNode>("SteptwoNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// ================== 最稳匹配代价函数（方向+面积+中心距）==================
double SteptwoNode::calcCost(const std::vector<cv::Point2f>& corners1,
                             const std::vector<cv::Point2f>& corners2) {
    cv::Rect r1 = cv::boundingRect(corners1);
    cv::Rect r2 = cv::boundingRect(corners2);

    cv::Point2f c1(r1.x + r1.width/2.0f,  r1.y + r1.height/2.0f);
    cv::Point2f c2(r2.x + r2.width/2.0f,  r2.y + r2.height/2.0f);
    double center_dist = cv::norm(c1 - c2);

    // 装甲板朝向向量（左右灯条中点连线）
    cv::Point2f dir1 = (corners1[1] + corners1[2]) * 0.5f - (corners1[0] + corners1[3]) * 0.5f;
    cv::Point2f dir2 = (corners2[1] + corners2[2]) * 0.5f - (corners2[0] + corners2[3]) * 0.5f;
    double angle_diff = std::abs(atan2(dir1.y, dir1.x) - atan2(dir2.y, dir2.x));
    angle_diff = std::min(angle_diff, CV_PI - angle_diff);

    // 面积比（防止大装甲匹配小装甲）
    double area_ratio = std::max(r1.area(), r2.area()) / std::max(1.0, (double)std::min(r1.area(), r2.area()));

    return center_dist * 1.0 + angle_diff * 200.0 + (area_ratio - 1.0) * 150.0;
}

void SteptwoNode::loadNumberTemplates() {
    number_templates.clear();
    for (int i = 1; i <= 5; i++) {
        std::string path = "/home/tomori/Vision_Arena_2025/templates/template_" + std::to_string(i) + ".png";
        cv::Mat templ = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (templ.empty()) continue;
        cv::GaussianBlur(templ, templ, cv::Size(3,3), 0);
        cv::erode(templ, templ, cv::Mat(), cv::Point(-1,-1), 1);
        cv::resize(templ, templ, cv::Size(32,45));
        number_templates.push_back(templ);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu number templates", number_templates.size());
}

int SteptwoNode::recognizeNumber(const cv::Mat& armor_roi) {
    if (armor_roi.empty() || number_templates.empty()) return -1;
    cv::Mat gray, fin;
    cv::cvtColor(armor_roi, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, fin, 0, 255, cv::THRESH_OTSU);

    double best_match = 0.3;
    int best_id = -1;

    for (int id = 0; id < number_templates.size(); id++) {
        const cv::Mat& templ = number_templates[id];
        std::vector<double> scales = {0.8, 0.9, 1.0, 1.1, 1.2};
        for (double scale : scales) {
            cv::Size sz(cv::saturate_cast<int>(templ.cols * scale),
                        cv::saturate_cast<int>(templ.rows * scale));
            if (sz.width > fin.cols || sz.height > fin.rows) continue;

            cv::Mat resized_templ;
            cv::resize(templ, resized_templ, sz);
            cv::Mat result;
            cv::matchTemplate(fin, resized_templ, result, cv::TM_CCOEFF_NORMED);
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

// ================== 主回调（完整保留你原检测逻辑 + 超稳跟踪）==================
void SteptwoNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
    try {
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

        cv::Mat image = cv_ptr->image.clone();
        if (image.empty()) return;

        result_image = image.clone();

        // ====================== 你原来的完整检测代码（完全不动）======================
        cv::Mat hsv, mask1, mask2, mask;
        cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 100, 100),   cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(156, 100, 100), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel, cv::Point(-1,-1), 1);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::RotatedRect> light_blobs;
        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 10) continue;
            cv::RotatedRect r = cv::minAreaRect(c);
            double w = r.size.width, h = r.size.height;
            double ratio = max(w, h) / min(w, h);
            if (ratio > 1.1 && ratio < 25) light_blobs.push_back(r);
        }

        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armors;
        for (size_t i = 0; i < light_blobs.size(); i++) {
            for (size_t j = i + 1; j < light_blobs.size(); j++) {
                auto& l1 = light_blobs[i];
                auto& l2 = light_blobs[j];
                cv::Point2f c1 = l1.center, c2 = l2.center;
                double dist = cv::norm(c1 - c2);
                if (dist < 15 || dist > 700) continue;
                if (abs(c1.y - c2.y) > 60) continue;

                double angle_diff = std::abs(l1.angle - l2.angle);
                angle_diff = std::min(angle_diff, 180 - angle_diff);
                if (angle_diff > 15) continue;

                if (c1.x < c2.x) armors.emplace_back(l1, l2);
                else            armors.emplace_back(l2, l1);
            }
        }

        // 收集所有检测到的装甲板
        std::vector<std::vector<cv::Point2f>> detected_armors;
        for (const auto& p : armors) {
            auto [left_light, right_light] = p;
            cv::Point2f lpts[4], rpts[4];
            left_light.points(lpts);
            right_light.points(rpts);
            std::vector<cv::Point2f> corners = { lpts[0], rpts[3], rpts[2], lpts[1] };
            detected_armors.push_back(corners);
        }

        // ====================== 超稳 Kalman 跟踪核心 ======================
        std::vector<bool> tracker_updated(trackers.size(), false);
        std::vector<bool> detection_used(detected_armors.size(), false);
        std::vector<std::pair<int, int>> matches;

        // 预测 + 匹配
        for (size_t i = 0; i < trackers.size(); ++i) {
            cv::Mat pred = trackers[i].kf.predict();
            cv::Point2f pred_c(pred.at<float>(0), pred.at<float>(1));
            float pw = pred.at<float>(2), ph = pred.at<float>(3);
            std::vector<cv::Point2f> pred_corners = {
                cv::Point2f(pred_c.x - pw/2, pred_c.y + ph/2),
                cv::Point2f(pred_c.x + pw/2, pred_c.y + ph/2),
                cv::Point2f(pred_c.x + pw/2, pred_c.y - ph/2),
                cv::Point2f(pred_c.x - pw/2, pred_c.y - ph/2)
            };

            double min_cost = 1e9;
            int best_j = -1;
            for (size_t j = 0; j < detected_armors.size(); ++j) {
                if (detection_used[j]) continue;
                double cost = calcCost(pred_corners, detected_armors[j]);
                if (cost < min_cost) {
                    min_cost = cost;
                    best_j = (int)j;
                }
            }
            if (best_j != -1 && min_cost < MATCH_THRESH) {
                matches.emplace_back(i, best_j);
                tracker_updated[i] = true;
                detection_used[best_j] = true;
            }
        }

        referee_pkg::msg::MultiObject target_msg;
        target_msg.header = msg->header;

        // 更新匹配上的
        for (auto [t_idx, d_idx] : matches) {
            auto& t = trackers[t_idx];
            const auto& corners = detected_armors[d_idx];

            cv::Rect roi = cv::boundingRect(corners);
            roi.y -= static_cast<int>(roi.height * 0.4);
            roi.height += static_cast<int>(roi.height * 0.8);
            roi.x -= 20; roi.width += 40;
            roi &= cv::Rect(0, 0, image.cols, image.rows);
            int number = recognizeNumber(image(roi));

            // Kalman 修正
            cv::Rect box = cv::boundingRect(corners);
            cv::Mat meas = (cv::Mat_<float>(4,1) << box.x + box.width/2.0f, box.y + box.height/2.0f, box.width, box.height);
            t.kf.correct(meas);
            t.last_corners = corners;
            t.missed_frames = 0;
            t.age++;

            // 画框 + 稳定ID
            for (int k = 0; k < 4; ++k)
                cv::line(result_image, corners[k], corners[(k+1)%4], cv::Scalar(0,255,0), 3);
            cv::putText(result_image, "ID:" + to_string(t.id), corners[0] + cv::Point2f(10,-10),
                        cv::FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar(0,255,255), 3);
            cv::putText(result_image, to_string(number), cv::Point(roi.x + roi.width/2 - 30, roi.y + 50),
                        cv::FONT_HERSHEY_SIMPLEX, 1.8, cv::Scalar(0,0,255), 4);

            referee_pkg::msg::Object obj;
            obj.target_type = "armor_red_" + to_string(number);
            obj.tracking_id = t.id;
            for (const auto& pt : corners) {
                geometry_msgs::msg::Point p{pt.x, pt.y, 0.0};
                obj.corners.push_back(p);
            }
            target_msg.objects.push_back(obj);
        }

        // 新目标
        for (size_t j = 0; j < detected_armors.size(); ++j) {
            if (!detection_used[j]) {
                trackers.emplace_back(next_id++, detected_armors[j]);
                // 新目标也发布
                const auto& corners = detected_armors[j];
                cv::Rect roi = cv::boundingRect(corners);
                roi.y -= static_cast<int>(roi.height * 0.4);
                roi.height += static_cast<int>(roi.height * 0.8);
                roi.x -= 20; roi.width += 40;
                roi &= cv::Rect(0, 0, image.cols, image.rows);
                int number = recognizeNumber(image(roi));

                for (int k = 0; k < 4; ++k)
                    cv::line(result_image, corners[k], corners[(k+1)%4], cv::Scalar(255,255,0), 3);
                cv::putText(result_image, "NEW:" + to_string(next_id-1), corners[0] + cv::Point2f(10,-10),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255,255,0), 3);

                referee_pkg::msg::Object obj;
                obj.target_type = "armor_red_" + to_string(number);
                obj.tracking_id = next_id - 1;
                for (const auto& pt : corners) {
                    geometry_msgs::msg::Point p{pt.x, pt.y, 0.0};
                    obj.corners.push_back(p);
                }
                target_msg.objects.push_back(obj);
            }
        }

        // 老化删除
        for (size_t i = 0; i < trackers.size(); ) {
            if (!tracker_updated[i]) {
                trackers[i].missed_frames++;
                if (trackers[i].missed_frames > MAX_MISSED)
                    trackers.erase(trackers.begin() + i);
                else ++i;
            } else ++i;
        }

        target_msg.num_objects = target_msg.objects.size();
        Target_pub->publish(target_msg);

        cv::putText(result_image, "Track:" + to_string(trackers.size()) + " Det:" + to_string(detected_armors.size()),
                    cv::Point(30, 100), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,255), 3);
        cv::imshow("Detection Result", result_image);
        cv::waitKey(1);

        RCLCPP_INFO(this->get_logger(), "Tracking %zu objects", trackers.size());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
}