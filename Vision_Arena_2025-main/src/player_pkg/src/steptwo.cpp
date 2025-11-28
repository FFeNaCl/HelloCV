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

        // 关键：构造函数里加载数字模板
        loadNumberTemplates();

        RCLCPP_INFO(this->get_logger(), "SteptwoNode initialized successfully");
    }

    ~SteptwoNode() { 
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
    auto node = std::make_shared<SteptwoNode>("SteptwoNode");
    RCLCPP_INFO(node->get_logger(), "Starting SteptwoNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//模板加载
void SteptwoNode::loadNumberTemplates() {
    number_templates.clear();
    for (int i = 1; i <= 5; i++) {
        std::string path = "/home/tomori/Vision_Arena_2025/templates/template_" + std::to_string(i) + ".png";
        cv::Mat templ = cv::imread(path, cv::IMREAD_GRAYSCALE);//转到灰度,所以之前的是原图
        if (templ.empty()) continue;
        cv::GaussianBlur(templ, templ, cv::Size(3,3), 0);  // 轻微模糊
        cv::erode(templ, templ, cv::Mat(), cv::Point(-1,-1), 1);  // 轻微腐蚀，防止过匹配
        cv::resize(templ, templ, cv::Size(32,45));  // 统一大小
        number_templates.push_back(templ);
    }
    RCLCPP_INFO(this->get_logger(), "Loaded %zu number templates", number_templates.size());
}

//数字识别
int SteptwoNode::recognizeNumber(const cv::Mat& armor_roi) {
    if (armor_roi.empty() || number_templates.empty()) return -1;
    //处理图像
    cv::Mat gray,fin;
    cv::cvtColor(armor_roi, gray, cv::COLOR_BGR2GRAY);
    //图像二值化，ostu只是一种方法
    cv::threshold(gray, fin, 0, 255, cv::THRESH_OTSU);

    double match = 0.3;   // 现场调，这个是匹配度阈值match
    int best_id = -1;

    for (int id = 0; id < number_templates.size(); id++) {//id是第i个模板
        const cv::Mat& templ = number_templates[id];//一个一个比

        std::vector<double> scales = {0.8, 0.9, 1.0, 1.1, 1.2};//多尺度匹配
        for (double scale : scales) {//还是：好用
            cv::Size sz(cv::saturate_cast<int>(templ.cols * scale),
                        cv::saturate_cast<int>(templ.rows * scale));//之前有越界问题，这个不会越界
            if (sz.width > fin.cols || sz.height > fin.rows) continue;//查资料偶然看到的，多一步筛选

            cv::Mat resized_templ;
            cv::resize(templ, resized_templ, sz);//模板改大小

            cv::Mat result;
            cv::matchTemplate(fin, resized_templ, result, cv::TM_CCOEFF_NORMED);

            double max_val;
            cv::Point poi;//最佳匹配点
            cv::minMaxLoc(result, nullptr, &max_val, nullptr, &poi);//max_val是实际匹配度

            if (max_val > match) {
                match = max_val;
                best_id = id + 1;
            }
        }
    }
    return best_id; 
}

// 主回调函数 
void SteptwoNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
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
        cv::inRange(hsv, cv::Scalar(0, 100, 100),   cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(156, 100, 100), cv::Scalar(180, 255, 255), mask2);
        mask = mask1 | mask2;

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel, cv::Point(-1,-1), 1);

        //找轮廓 + 筛选灯条
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::RotatedRect> light_blobs;//存矩形
        for (const auto& c : contours) {
            double area = cv::contourArea(c);
            if (area < 10) continue;//这个要改的，不然有的小东西检查不出来

            cv::RotatedRect r = cv::minAreaRect(c);
            double w = r.size.width, h = r.size.height;
            double ratio = max(w, h) / min(w, h);//不知道宽高哪个长
            if (ratio > 1.1 && ratio < 25) {//细长的
                light_blobs.push_back(r);
            }
        }

        //灯条配对
        std::vector<std::pair<cv::RotatedRect, cv::RotatedRect>> armors;//pair为一对，这里存匹配的矩形组
        for (size_t i = 0; i < light_blobs.size(); i++) {
            for (size_t j = i + 1; j < light_blobs.size(); j++) {
                auto& l1 = light_blobs[i];
                auto& l2 = light_blobs[j];
                cv::Point2f c1 = l1.center, c2 = l2.center;//输入中心点
                double dist = cv::norm(c1 - c2);//计算距离
                if (dist < 15 || dist > 700) continue;//这个现场改
                if (abs(c1.y - c2.y) > 60) continue;//纵坐标差太多也不行，这个可以小一点

                double angle_diff = std::abs(l1.angle - l2.angle);
		        angle_diff = std::min(angle_diff, 180 - angle_diff); // 取最小角度
	        	if (angle_diff > 15) continue;//角度筛选

                if (c1.x < c2.x)
                    armors.emplace_back(l1, l2);
                else
                    armors.emplace_back(l2, l1);
            }
        }
        //处理每个装甲板
            
            referee_pkg::msg::MultiObject target_msg;
            target_msg.header = msg->header;//这里放在循环外

        for (size_t i = 0; i < armors.size(); i++) {
            auto [left_light, right_light] = armors[i];//这个是强行拆分pair，用.first什么的也行

            cv::Point2f lpts[4], rpts[4];
            left_light.points(lpts);
            right_light.points(rpts);

            // 左下 → 右下 → 右上 → 左上
            std::vector<cv::Point2f> armor_corners = {
                lpts[0],  // 左灯条下
                rpts[3],  // 右灯条下
                rpts[2],  // 右灯条上
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

            // ROI
            cv::Rect roi = cv::boundingRect(armor_corners);
            roi.y -= static_cast<int>(roi.height * 0.4);//这两个数也经常改
            roi.height += static_cast<int>(roi.height * 0.8);
            roi.x -= 20;
            roi.width += 40;//注意Rect是由什么组成的
            roi &= cv::Rect(0, 0, image.cols, image.rows);//这里有越界的问题

            cv::Mat armor_roi = image(roi).clone();//之前语雀记过三种拷贝的区别
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
