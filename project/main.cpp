#include<opencv2/opencv.hpp> 
#include<iostream>
using namespace std;
using namespace cv;
 int main() 
 {
    string path="/home/tomori/图片/114514.png";
    Mat image,img,hsv,fin,ori,hiv,mask;
    image=imread(path);
    cvtColor(image,hsv,COLOR_BGR2HSV);
    namedWindow("sss",(7,7));
    int hmin=0,smin=0,vmin=0,hmax=179,smax=255,vmax=255;
    createTrackbar("hmin","sss",&hmin,179);
    createTrackbar("smin","sss",&smin,255);
    createTrackbar("vmin","sss",&vmin,255);
    createTrackbar("hmax","sss",&hmax,179);
    createTrackbar("smax","sss",&smax,255);
    createTrackbar("vmax","sss",&vmax,255);
    while(true)
    {
      Scalar lower(hmin,smin,vmin);
      Scalar upper(hmax,smax,vmax);
      inRange(hsv,lower,upper,mask);
      imshow("result",mask);
      waitKey (1);
    }
    
    return 0;
 }
 