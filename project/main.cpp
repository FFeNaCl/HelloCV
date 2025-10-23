#include <opencv2/opencv.hpp> 
 #include <iostream>
 using namespace std;
 using namespace cv;
 int main() {
     VideoCapture cap("/home/tomori/opencvtext111/TrafficLight.mp4");
     int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));
     int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
     double fps = cap.get(CAP_PROP_FPS);
      VideoWriter output_video("result.avi",VideoWriter::fourcc('D', 'I', 'V', 'X'),fps,Size(frame_width, frame_height),true);
     Mat frame, processed_frame;
     while (true) {
         cap >> frame;
         if (frame.empty()) {  break;}

    Mat image,img,hsv,fin,ori,hiv,mask,image1;
    image1=frame.clone();
    cvtColor(image1,image,COLOR_BGR2HSV);
    //Scalar lower(57,143,130);//a
    //Scalar upper(93,255,255);//a
    Scalar lower(0,87,171);//a
    Scalar upper(179,255,255);//a
    inRange(image,lower,upper,mask);
    GaussianBlur(mask,hsv,Size(5,5),10,10);//b
    Canny(hsv,fin,50,300);//c
    hiv=getStructuringElement(MORPH_RECT,Size(3,3));
    dilate(fin,ori,hiv);//ori
    vector<vector<Point>> a;
    vector<Vec4i> b;
    findContours(ori,a,b,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //drawContours(image,a,-1,Scalar(255,0,0),2);
    for(int i=0;i<a.size();i++)
    {
      int area=contourArea(a[i]);
      vector<vector<Point>> d(a.size());
      vector<Rect> c(a.size());
      //cout<<area<<endl;
      if(area>=28000)//11692 jiance
      {
       float p=arcLength(a[i],true);
       approxPolyDP(a[i],d[i],0.02*p,true);
       //drawContours(image1,d,i,Scalar(255,0,0),2);
       //cout<<endl<<d[i].size()<<endl;
        if(d[i].size()==8)
        {
          c[i]=boundingRect(d[i]);
         rectangle(image1,c[i].tl(),c[i].br(),Scalar(0,255,0,2));
         putText(image1,"red",{50,50},FONT_HERSHEY_DUPLEX,0.75,Scalar(255,0,0),1);
        }
      }
    }

    Mat image2,img2,hsv2,fin2,ori2,hiv2,mask2;
    cvtColor(image1,image2,COLOR_BGR2HSV);
    Scalar lower2(57,143,130);//a
    Scalar upper2(93,255,255);//a
    //Scalar lower(0,87,171);//a
    //Scalar upper(179,255,255);//a
    inRange(image2,lower2,upper2,mask2);
    GaussianBlur(mask2,hsv2,Size(5,5),10,10);//b
    Canny(hsv2,fin2,50,300);//c
    hiv=getStructuringElement(MORPH_RECT,Size(3,3));
    dilate(fin2,ori2,hiv2);//ori
    vector<vector<Point>> a2;
    vector<Vec4i> b2;
    findContours(ori2,a2,b2,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    //drawContours(image,a,-1,Scalar(255,0,0),2);
    for(int i=0;i<a2.size();i++)
    {
      int area2=contourArea(a2[i]);
      vector<vector<Point>> d2(a2.size());
      vector<Rect> c2(a2.size());
      //cout<<area<<endl;
      if(area2>=28000)//11692 jiance
      {
       float p2=arcLength(a2[i],true);
       approxPolyDP(a2[i],d2[i],0.02*p2,true);
       //drawContours(image1,d,i,Scalar(255,0,0),2);
       //cout<<endl<<d[i].size()<<endl;
        if(d2[i].size()==8)
        {
          c2[i]=boundingRect(d2[i]);
         rectangle(image1,c2[i].tl(),c2[i].br(),Scalar(0,255,0,2));
         putText(image1,"green",{50,50},FONT_HERSHEY_DUPLEX,0.75,Scalar(255,0,0),1);
        }
      }
    }
    processed_frame=image1.clone();

         output_video << processed_frame;
         imshow("处理后", processed_frame);
         if (waitKey(1) == 27) {break;}
     }
     destroyAllWindows();    // 关闭所有显示窗口
     cout << "视频处理完成！输出文件：result.avi" << endl;
     return 0;
 }