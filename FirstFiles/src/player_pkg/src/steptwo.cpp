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
#include <opencv2/tracking.hpp>
#include <opencv2/tracking/tracking_legacy.hpp> 

using namespace std;
using namespace rclcpp;
using namespace cv;

class SteptwoNode : public rclcpp::Node {
 public:
  SteptwoNode
(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing SteptwoNode");//输出信息1
    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&SteptwoNode::callback_camera, this, std::placeholders::_1));
    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(this->get_logger(), "ArmorNodeinitialized successfully");//输出信息2
  }

  ~SteptwoNode() { cv::destroyWindow("Detection Result"); }

 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SteptwoNode>("SteptwoNode");
  RCLCPP_INFO(node->get_logger(), "Starting SteptwoNode");//输出信息3
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void SteptwoNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
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
    // 创建结果图像
    cv::Mat result_image = image.clone();//原图是image
    // 转换到 HSV 空间
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    // 红色检测 - 使用稳定的范围
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255),mask2);
    mask = mask1 | mask2;
    // 适度的形态学操作，开运算闭运算，处理mask
    cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    Point_V.clear();
    int vaild_rectangle = 0;

    

    for (size_t i = 0; i < contours.size()-1; i++) {
        // 2. 计算最小外接旋转矩形 
       
        vector<vector<Point>> contour(contours.size());
        vector<Point2f> rectangle_points(4);
        float pai=arcLength(contours[i],true);
        approxPolyDP(contours[i],contour[i],0.02*pai,true);

        float pai1=arcLength(contours[i+1],true);
        approxPolyDP(contours[i+1],contour[i+1],0.02*pai1,true);

        rectangle_points[0]=contour[i+1][3];
        rectangle_points[1]=contour[i][2];
        rectangle_points[2]=contour[i][1];
        rectangle_points[3]=contour[i+1][0];

        // 绘制检测到的球体，result_image是image复制版
        cv::rectangle(result_image,rectangle_points[1],rectangle_points[3],cv::Scalar(0, 255, 0), 2);  // 绿色圆圈
        // 绘制球体上的四个点
        vector<string> point_names = {"左下", "右下", "右上", "左上"};
        vector<cv::Scalar> point_colors = {
            cv::Scalar(255, 0, 0),    // 蓝色 - 左下
            cv::Scalar(0, 255, 0),    // 绿色 - 右下
            cv::Scalar(0, 255, 255),  // 黄色 - 右上
            cv::Scalar(255, 0, 255)   // 紫色 - 左上
        };
        for (int j = 0; j < 4; j++) {
          cv::circle(result_image, rectangle_points[j], 6, point_colors[j], -1);//颜色填充
          cv::circle(result_image, rectangle_points[j], 6, cv::Scalar(0, 0, 0), 2);//黑色描边

          // 标注序号
          string point_text = to_string(j + 1);
          cv::putText(
              result_image, point_text,
              cv::Point(rectangle_points[j].x + 10, rectangle_points[j].y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);//白色描字
          cv::putText(
              result_image, point_text,
              cv::Point(rectangle_points[j].x + 10, rectangle_points[j].y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);//数字标号
          // 添加到发送列表
          Point_V.push_back(rectangle_points[j]);
          RCLCPP_INFO(this->get_logger(),
                      "Rectangle %d, Point %d (%s): (%.1f, %.1f)",
                      vaild_rectangle + 1, j + 1, point_names[j].c_str(),
                      rectangle_points[j].x, rectangle_points[j].y);//输出字和坐标
        }

        //数字识别
        Mat img,img1;
        cvtColor(image,img,COLOR_BGR2GRAY);
        threshold(img,img1,180,255,THRESH_BINARY);
        const double MATCH_THRESHOLD = 0.7; //匹配度，这个要改
        for(int j=1;j<=5;j++)
        {
          Mat templ = imread("/home/tomori/Vision_Arena_2025/templates/template_"+to_string(j)+".png", IMREAD_GRAYSCALE);//循环加载
          Mat result;
          matchTemplate(img1, templ, result, TM_CCOEFF_NORMED); //模板匹配
          Point r;double maxScore;
          minMaxLoc(result, nullptr, &maxScore, nullptr, &r);//最佳模板点
          if (maxScore >= MATCH_THRESHOLD) {
          rectangle(result_image, r, Point(r.x + templ.cols, r.y + templ.rows),255, 2);
          putText(result_image, to_string(j), Point(r.x, r.y - 10), 
                 FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 0), 2);} 
          RCLCPP_INFO(this->get_logger(),"MATCHTHRESHOLD:%lf",maxScore);//输出字和坐标
        }

        //目标动态跟踪
        std::vector<cv::Rect> objects;
        objects.push_back(boundingRect(rectangle_points));
        Ptr<legacy::MultiTracker> tracker = legacy::MultiTracker::create();
        for (auto &box : objects)
            tracker->add(legacy::TrackerCSRT::create(), image, box);
        tracker->update(image);
        for (unsigned m = 0; m < tracker->getObjects().size(); ++m)
            rectangle(result_image, tracker->getObjects()[m], Scalar(255, 0, 0), 2);
        putText(result_image, "ID: "+to_string(i/2+1), Point(rectangle_points[0].x, rectangle_points[0].y - 10), 
                 FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);

      vaild_rectangle++;
      i++;
    }

    // 显示结果图像
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 创建并发布消息
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;
    msg_object.num_objects = Point_V.size() / 4;

    vector<string> types = {"rectangle"};

    for (int k = 0; k < msg_object.num_objects; k++) {
      referee_pkg::msg::Object obj;
      obj.target_type = (k < types.size()) ? types[k] : "unknown";

      for (int j = 0; j < 4; j++) {
        int index = 4 * k + j;
        if (index < Point_V.size()) {
          geometry_msgs::msg::Point corner;
          corner.x = Point_V[index].x;
          corner.y = Point_V[index].y;
          corner.z = 0.0;
          obj.corners.push_back(corner);
        }
      }

      msg_object.objects.push_back(obj);
    }

    Target_pub->publish(msg_object);
    RCLCPP_INFO(this->get_logger(), "Published %d rectangle targets",msg_object.num_objects);

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}