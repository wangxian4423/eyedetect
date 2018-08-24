/**
 * Author: DHZT_Zouqiang
*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sensor_msgs/Image.h>
using namespace cv;
using namespace std;

int iLowH = 70;
int iHighH = 130;
int iLowS = 90;
int iHighS = 255;
int iLowV = 90;
int iHighV = 255;
int number=3;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  public:
  ImageConverter(): it_(nh_)
  {
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, &ImageConverter::imageCb, this);
    //image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
	cv::namedWindow("Thresholded Image");
	cv::namedWindow("colorImg");

	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);
	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);
	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);
  }

  ~ImageConverter()
  {
	cv::destroyWindow("Thresholded Image");
	cv::destroyWindow("colorImg");
  }
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	cv_bridge::CvImagePtr cv_ptr;
	Mat colorImg;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	  colorImg = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	Mat imgHSV;
	vector<Mat> hsvSplit;
	Mat deff_co;
	Mat pre_co=imread("//home//wxw//catkin_ws//color.jpg");
	imshow("pre_co",pre_co);
	absdiff(pre_co,colorImg,deff_co);

	imshow("deff_co",deff_co);

	cvtColor(deff_co, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	//因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2],hsvSplit[2]);
	merge(hsvSplit,imgHSV);
	Mat imgThresholded;

	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//开操作 (去除一些噪点)
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

	//闭操作 (连接一些连通域)
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

  GaussianBlur(imgThresholded, imgThresholded, Size(3, 3), 0.1, 0, BORDER_DEFAULT);//
  	  blur(imgThresholded, imgThresholded, Size(3, 3)); //
	//canny算子，边缘检测
  	  Mat canny_output;
  	  vector<vector<Point> > contours;
  	  vector<Vec4i> hierarchy;
  	  Canny(imgThresholded, canny_output, 30, 30 * 3, 3);

  	  findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  	  //********************
  	  if (contours.size() == 0)
  	  {
  		  cout << "all targets object missing" << endl;
  		 
  	  }
  	  else
  	  {
  		  Mat result(imgThresholded.size(), CV_8U, Scalar(0));
  		  Mat area(1, contours.size(), CV_32FC1);
  		 // float maxArea =0;
  		//  int max = 0;
			int i=0;
			
  		  for (i = 0; i < (int)contours.size(); i++)
  		  {
  			   
  			 vector<Moments> mu(contours.size());//霍夫向量矩阵
					 vector<Point2f> mc(contours.size());//中心点坐标向量存储矩阵
  			  if ((10 < contourArea(contours[i])) && (contourArea(contours[i])<3200))
  			 	 {
  				 
  			  
  		 			 drawContours(colorImg, contours,i,Scalar(50), 1, 8);
						mu[i] = moments(contours[i], false);
						 mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
						 
						 circle(colorImg, mc[i], 5, Scalar(0, 0, 255), -1, 8, 0);
					char tam1[100];
						sprintf(tam1, "(area=%0.0f)", contourArea(contours[i]));			
						putText(colorImg, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
					
  		  			}
			
			//ROS_INFO("area=%f",maxArea);
		//	drawContours(colorImg, contours,max,Scalar(255), 1, 8);
			}

	imshow("Thresholded Image", imgThresholded); //show the thresholded image
	imshow("colorImg", colorImg); //show the original image
	cv::waitKey(3);

  }
 }
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh;
//	ros::Timer timer = nh.createTimer(ros::Duration(3),TimerCallback);
	ImageConverter ic;
	ros::spin();
	return 0;
}
