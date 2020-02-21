#include <opencv2/opencv.hpp>
#include <iostream>
#include <librealsense2/rs.hpp>

using namespace cv;
using namespace std;
using namespace rs2;




int times_clicked = 0;
vector<Point2d> clicked;
Mat color;
vector<Point3d> object_point;
vector<double> distCoEff;
Mat point = cv::Mat(3, 3, CV_64FC1);
vector<double> rvec;
vector<double> tvec;




/*
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          
          if (times_clicked < 4) {
            cout << "Point " << times_clicked << ": (" << x << ", " << y << ")" << endl;
            clicked.push_back(Point2d(x,y));
            circle(color, clicked[times_clicked],4,Scalar(100,100,100));
            imshow("Frames", color);
          } 
          
          times_clicked++;
          if (times_clicked == 4) {
            cout << "Finished" << endl;
            solvePnP(object_point,clicked,point,distCoEff,rvec,tvec);
            cout << "RVEC: " << rvec[0] << " " << rvec[1] << " " << rvec[2] << endl;
            cout << "TVEC: " << tvec[0] << " " << tvec[1] << " " << tvec[2] << endl;
            solvePnPRansac(object_point, clicked, point, distCoEff,rvec,tvec);
            cout << "Ransac" << endl;
            cout << "RVEC: " << rvec[0] << " " << rvec[1] << " " << rvec[2] << endl;
            cout << "TVEC: " << tvec[0] << " " << tvec[1] << " " << tvec[2] << endl;
          }
          
     }

}



int main()
{
     
    object_point.push_back(Point3d(0,0,0));
    object_point.push_back(Point3d(0,9,0));
    object_point.push_back(Point3d(14.5,9,0));
    object_point.push_back(Point3d(14.5,0,0));

    point.at<double>(0,0) = 2263.308139034971;
    point.at<double>(0,1) = 0.0;
    point.at<double>(0,2) = 1568.2338734705725;
    point.at<double>(1,0) = 0.0;
    point.at<double>(1,1) = 2264.562609500345;
    point.at<double>(1,2) = 1260.8733211258661;
    point.at<double>(2,0) = 0.0;
    point.at<double>(2,1) = 0.0;
    point.at<double>(2,2) = 1.0;

    distCoEff.push_back(-0.08229109707395792);
    distCoEff.push_back(-0.11081410511303218);
    distCoEff.push_back(0.28965062617980036);
    distCoEff.push_back(-0.28489728805138115);
    

    pipeline pipe;
    config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 960, 540, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);
    frameset frames;
    for(int i = 0; i < 30; i++)
    {
        frames = pipe.wait_for_frames();
    }
    frame color_frame = frames.get_color_frame();
    color = Mat(Size(960, 540), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
    
    namedWindow("Frames", 1);
    imshow("Frames",color);
    setMouseCallback("Frames", CallBackFunc, NULL);
    
    waitKey(0);

    return 0;
}


*/







Point2f findLight(Mat frame,  Scalar low, Scalar high) {

  inRange(frame, low, high,frame);
  erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  dilate(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
  dilate(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5))); 
  erode(frame, frame, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
  
  Moments light_moment = moments(frame);
  double M10 = light_moment.m10;
  double M01 = light_moment.m01;
  double area = light_moment.m00;
  int light_center_x = M10/area;
  int light_center_y = M01/area;
  return Point2f(light_center_x,light_center_y);
}


int main() {  
    
    object_point.push_back(Point3d(0,0,0));
    object_point.push_back(Point3d(0,9,0));
    object_point.push_back(Point3d(14.5,9,0));
    object_point.push_back(Point3d(14.5,0,0));

    point.at<double>(0,0) = 2263.308139034971;
    point.at<double>(0,1) = 0.0;
    point.at<double>(0,2) = 1568.2338734705725;
    point.at<double>(1,0) = 0.0;
    point.at<double>(1,1) = 2264.562609500345;
    point.at<double>(1,2) = 1260.8733211258661;
    point.at<double>(2,0) = 0.0;
    point.at<double>(2,1) = 0.0;
    point.at<double>(2,2) = 1.0;

    distCoEff.push_back(-0.08229109707395792);
    distCoEff.push_back(-0.11081410511303218);
    distCoEff.push_back(0.28965062617980036);
    distCoEff.push_back(-0.28489728805138115);
    
    
    int low_H = 100;
    int low_S = 100;
    int low_V = 100;

    int high_H = 100;
    int high_S = 100;
    int high_V = 100;
    namedWindow("Control");

    createTrackbar("LowH", "Control", &low_H, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &high_H, 179);

    createTrackbar("LowS", "Control", &low_S, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &high_S, 255);

    createTrackbar("LowV", "Control", &low_V, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &high_V, 255);

    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30); //Ensures BGR format
    pipe.start(cfg);

       while (true) {
        rs2::frameset data = pipe.wait_for_frames(); 
        rs2::frame color_frame = data.get_color_frame();
        const int w = color_frame.as<rs2::video_frame>().get_width();
        const int h = color_frame.as<rs2::video_frame>().get_height();
        Mat frame(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat frame_HSV;
        Mat frame_light;

        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);     
        for(int i = 0l; i < 3; i++) {
          findLight(frame_HSV, );
          }

        imshow("Frame", frame);
        imshow("Filtered", frame_light);
        

        if (waitKey(1) >= 0) break; 
    }
    
    return 0;

}
