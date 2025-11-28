#include<opencv2/opencv.hpp> 
#include<iostream>
using namespace std;
void onMouse(int event, int x, int y, int flags, void* param) { 
     if (event == cv::EVENT_LBUTTONDOWN) { // 左键点击事件 
         cv::Mat* img = static_cast<cv::Mat*>(param); // 获取传入的图像数据 
         // 处理坐标(x,y)和像素值 
         cv::Vec3b pixel = img->at<cv::Vec3b>(y, x); 
         std::cout << "坐标: (" << x << "," << y << "), BGR值: (" 
                   << (int)pixel[0] << "," << (int)pixel[1] << "," << (int)pixel[2] << ")" << std::endl;
     } 
 }
 int main() 
 {
    
 cv::Mat img = cv::imread("/home/tomori/图片/112.png"); 
 cv::namedWindow("Image"); 
 cv::setMouseCallback("Image", onMouse, &img); // 绑定窗口、回调函数和图像数据 
 cv::imshow("Image", img); 
 cv::waitKey(0); // 等待用户操作
    return 0;
 }