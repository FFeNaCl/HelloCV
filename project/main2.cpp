#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include<iostream>
 using namespace cv;
 using  namespace std;
  
 int main() {
     Mat image = imread("/home/tomori/图片/114514.png"),img,img1;
     cvtColor(image,img,COLOR_BGR2GRAY); 
     threshold(img,img1,180,255,THRESH_BINARY);
     const double MATCH_THRESHOLD = 0.5; 
     for(int j=1;j<=5;j++)
     {
        Mat templ = imread("/home/tomori/snap/project/templates/template_"+to_string(j)+".png", IMREAD_GRAYSCALE);
        Mat result;
        matchTemplate(img1, templ, result, TM_CCOEFF_NORMED);
        Point r;double maxScore;
        minMaxLoc(result, nullptr, &maxScore, nullptr, &r);
         if (maxScore >= MATCH_THRESHOLD) {
        rectangle(image, r, Point(r.x + templ.cols, r.y + templ.rows), 255, 2);
         putText(image,to_string(j), Point(r.x, r.y - 10), 
                 FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);} 
        putText(image,to_string(maxScore), Point(j*50,j*50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
     }
     imshow("Result", image);
     waitKey(0);
     return 0;
 }
 