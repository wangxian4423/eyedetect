

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>


using namespace cv;
using namespace std;

ros::Publisher pose_arm_pub,pose_car_pub;

geometry_msgs::Vector3 pose_arm,pose_car;

float pose_max_x,pose_max_y,pose_max_z=0;
float prepos_x=0,prepos_y=0,prepos_z=0;


/////////
volatile int poseX;
volatile int poseY;
int count_hole;//记录检测到的此范围孔件数量

struct Object
{
	int obj_id;//0,1,2,3,4,,5,6,7  0是三棱柱，1是四棱柱，3是六棱柱，4是圆柱，5是三棱柱对应装配体，6是四棱柱对应装配体，7是六棱柱对应装配体，8是圆柱对应装配体
	int state;//1立着 0架着 -1躺着 
	struct Pose
	{
		float x, y, z, rx, ry, rz;
	}pose;//pose指示图像坐标系坐标
 } object[10];

 cv::Mat deff_de = cv::Mat(540, 960, CV_8U);
 cv::Mat deff_co ;
////////

////



void TimerCallback(const ros::TimerEvent&)
{
	pose_arm_pub.publish(pose_arm);

}

void TimerCallback1(const ros::TimerEvent&)
{
	pose_car_pub.publish(pose_car);

}

class Receiver
{
public:
  enum Mode
  {
    IMAGE=0 ,
    CLOUD,
	MYDEPTH,
    
  };

private:
  std::mutex lock;

  const std::string topicColor, topicDepth;
  const bool useExact, useCompressed;

  bool updateImage, updateCloud;
  bool save;
  bool running;
  size_t frame;
  const size_t queueSize;

  cv::Mat color, depth;
  cv::Mat cameraMatrixColor, cameraMatrixDepth;
  cv::Mat lookupX, lookupY;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner;
  image_transport::ImageTransport it;
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;

  message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
  message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

  std::thread imageViewerThread;
  Mode mode;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
  pcl::PCDWriter writer;
  std::ostringstream oss;
  std::vector<int> params;

public:
  Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
    : topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
      updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
      nh("~"), spinner(0), it(nh), mode(CLOUD)
  {
    cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
    cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    params.push_back(1);
    params.push_back(cv::IMWRITE_PNG_STRATEGY);
    params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
    params.push_back(0);
  }

  ~Receiver()
  {
  }

void run(const Mode mode)
{
    start(mode);
    stop();
}

private:
  void start(const Mode mode)
  {

      this->mode = mode;
      running = true;

      std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
      std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

      image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
      subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
      subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
      subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
      subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);

      if(useExact)
      {

          syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
          syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
      }
      else
      {
          syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
          syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
      }

      spinner.start();

      std::chrono::milliseconds duration(1);
      while(!updateImage || !updateCloud)
      {
          if(!ros::ok())
              {
              return;
              }
          std::this_thread::sleep_for(duration);
      }
      cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
      cloud->height = color.rows;
      cloud->width = color.cols;
      cloud->is_dense = false;
      cloud->points.resize(cloud->height * cloud->width);
      createLookup(this->color.cols, this->color.rows);

      switch(mode)
      {
     case CLOUD:
          //cloudViewer();
		 mydepthViewer();
      break;
      case IMAGE:
    	  //imageViewer();
		  mydepthViewer();
      break;
	  case MYDEPTH:
	      mydepthViewer();
	  break;
      //case BOTH:
    	  //imageViewerThread = std::thread(&Receiver::imageViewer, this);
    	  //cloudViewer();
     // break;
    }
  }

  void stop()
  {
      spinner.stop();

      if(useExact)
      {
    	  delete syncExact;
      }
      else
      {
    	  delete syncApproximate;
      }
      delete subImageColor;
      delete subImageDepth;
      delete subCameraInfoColor;
      delete subCameraInfoDepth;

      running = false;
     
  }

  void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
                const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
  {
      cv::Mat color, depth;

      readCameraInfo(cameraInfoColor, cameraMatrixColor);
      readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
      readImage(imageColor, color);
      readImage(imageDepth, depth);

      // IR image input
      if(color.type() == CV_16U)

          {
    	  cv::Mat tmp;
          color.convertTo(tmp, CV_8U, 0.02);
          cv::cvtColor(tmp, color, CV_GRAY2BGR);
          }

      lock.lock();
      this->color = color;
      this->depth = depth;
      updateImage = true;
      updateCloud = true;
      lock.unlock();
  }

  void imageViewer()
  {
	  cv::Mat color, depth, depthDisp, combined;
      std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
      double fps = 0;
      size_t frameCount = 0;
      std::ostringstream oss;
      const cv::Point pos(5, 15);
      const cv::Scalar colorText = CV_RGB(255, 255, 255);
      const double sizeText = 0.5;
      const int lineText = 1;
      const int font = cv::FONT_HERSHEY_SIMPLEX;

      cv::namedWindow("Image Viewer");
      oss << "starting...";

      start = std::chrono::high_resolution_clock::now();
      for(; running && ros::ok();)

    	  {
          if(updateImage)
              {
              lock.lock();
              color = this->color;
              depth = this->depth;
              updateImage = false;
              lock.unlock();


              ++frameCount;
              now = std::chrono::high_resolution_clock::now();
              double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
              if(elapsed >= 1.0)

            	  {
                  fps = frameCount / elapsed;
                  oss.str("");
                  oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                  start = now;
                  frameCount = 0;
                  }

              dispDepth(depth, depthDisp, 12000.0f);
              combine(color, depthDisp, combined);

              //combined = color;

              cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
              cv::imshow("Image Viewer combined", combined);
              }

              int key = cv::waitKey(1);
              switch(key & 0xFF)
              {
              case 27:
              case 'q':
                  running = false;
                  break;
              case ' ':
              case 's':

            	  if(mode == IMAGE)
                  	  {
            		  createCloud(depth, color, cloud);
            		  saveCloudAndImages(cloud, color, depth, depthDisp);
                  	  }
            	  else
            	  {
            		  save = true;
            	  }
            	  break;
              }
    	  }
      cv::destroyAllWindows();
          cv::waitKey(100);
  }
  void mydepthViewer() //自己写得深度图像检测
  {
	  
  	 
	    cv::Mat color, depth, depthDisp, combined,depthshow,target;
   //  cv::Mat deff_de = cv::Mat(depth.rows, depth.cols, CV_8U);
	  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
      double fps = 0;
      size_t frameCount = 0;
      std::ostringstream oss;
      const cv::Point pos(5, 15);
      const cv::Scalar colorText = CV_RGB(255, 255, 255);
      const double sizeText = 0.5;
      const int lineText = 1;
      const int font = cv::FONT_HERSHEY_SIMPLEX;

//

Mat pre_co=imread("//home//wxw//catkin_ws//color.jpg");
     
	 cv::FileStorage fs("//home//wxw//catkin_ws//depth.xml", cv::FileStorage::READ);  
     Mat pre_de;  
     fs["vocabulary"] >> pre_de;  

//
      cv::namedWindow("depthshow");
	  cv::namedWindow("combined");
      oss << "starting...";

      start = std::chrono::high_resolution_clock::now();
      for(; running && ros::ok();)

    	  {
          if(updateImage)
              {
              lock.lock();
              color = this->color;
              depth = this->depth;
              updateImage = false;
              lock.unlock();
              ++frameCount;
              now = std::chrono::high_resolution_clock::now();
              double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
              if(elapsed >= 1.0)

            	  {
                  fps = frameCount / elapsed;
                  oss.str("");
                  oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
                  start = now;
                  frameCount = 0;
                  }
               //cv::Mat deff_de = cv::Mat(depth.rows, depth.cols, CV_8U);
			absd(pre_de,pre_co,depth,color);



             depthcontrol(depth,depthshow,800,1000);//深度图像筛选

             // dispDepth(depth, depthDisp, 12000.0f);
              combine(color, depthshow, combined);//深度与彩色结合
             targetdetection(color,depth,combined,target);
             // combined = color;
			
			 




              cv::putText(depthshow, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);             
			  cv::imshow("depthshow", depthshow);
			  cv::imshow("combined",combined);
			  cv::waitKey(1);
              }

    	  }
      cv::destroyAllWindows();
          cv::waitKey(100);
  }
  void cloudViewer()
  {
	  cv::Mat color, depth;
	  std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
	  double fps = 0;
	  size_t frameCount = 0;
	  std::ostringstream oss;
	  std::ostringstream ossXYZ; // 新增一个string流
	  const cv::Point pos(5, 15);
      const cv::Scalar colorText = CV_RGB(255, 0, 0);
      const double sizeText = 0.5;
      const int lineText = 1;
      const int font = cv::FONT_HERSHEY_SIMPLEX;

	  ///////////////////
	  // 从全局变量获取当前像素
	  int img_x = poseX;
      int img_y = poseY;

      geometry_msgs::PointStamped ptMsg;
      ptMsg.header.frame_id = "kinect_link";

      lock.lock();
      color = this->color;
      depth = this->depth;
      updateCloud = false;
      lock.unlock();
     // pose_get(color);//二维点获取
      const std::string window_name = "color viewer";

	  createCloud(depth, color, cloud);


	  for(; running && ros::ok();)
	  {
	      if(updateCloud)
	      {
	          lock.lock();
	          color = this->color;
	          depth = this->depth;
	          updateCloud = false;
	          lock.unlock();
	          //pose_get(color);//二维点获取
	          createCloud(depth, color, cloud);

	          ////////////////////
	          img_x = poseX;
	          img_y = poseY;

	          const pcl::PointXYZRGBA& pt = cloud->points[img_y * depth.cols + img_x];
	          ptMsg.point.x = pt.x;
	          ptMsg.point.y = pt.y;
	          ptMsg.point.z = pt.z;
	          //ros::NodeHandle nh;
	          ptMsg.header.stamp = ros::Time::now();

              // cal fps
	          ++frameCount;
	          now = std::chrono::high_resolution_clock::now();
	          double elapsed =
	          std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;

	          if(elapsed >= 1.0)
	          {
	        	  fps = frameCount / elapsed;
	        	  oss.str("");
	        	  oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
	        	  start = now;
	        	  frameCount = 0;
	          }
	          cv::putText(color, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
	          ossXYZ.str("");
	          ossXYZ << "( " << ptMsg.point.x << ", " << ptMsg.point.y
	                                    << ", " << ptMsg.point.z << " )";
	          cout<<"x= "<<ptMsg.point.x<<"****"<<"y= "<<ptMsg.point.y<<"****"<<"z= "<<ptMsg.point.z<<endl;

	          pose_arm.x=ptMsg.point.x;
		  	  pose_arm.z=ptMsg.point.z*0.877-0.1;
	          pose_arm.y=ptMsg.point.y/0.877+ptMsg.point.z*0.479-0.1;
	          //pose_arm.z=ptMsg.point.z*cos(-0.5);
//cout<<"coscos="<<cos(28.65)<<endl;
cout<<"px= "<<pose_arm.x<<"****"<<"py= "<<pose_arm.y<<"****"<<"pz= "<<pose_arm.z<<endl;
if((ptMsg.point.x-prepos_x<0.20)&&(ptMsg.point.y-prepos_y<0.20)&&(ptMsg.point.z-prepos_z<0.20))
{
	if (ptMsg.point.z>pose_max_z)
		{	 
		  pose_max_z=ptMsg.point.z;
		  pose_car.x=ptMsg.point.x;
	          pose_car.y=ptMsg.point.y;
		  pose_car.z=pose_max_z;	          
	          }
}

	prepos_x=ptMsg.point.x;
	prepos_y=ptMsg.point.y;
	prepos_z=ptMsg.point.z;





	          cv::putText(color, ossXYZ.str(), cv::Point(img_x, img_y), font, 0.5, colorText, 0.3, CV_AA);
	          cv::circle(color, cv::Point(poseX, poseY), 5, cv::Scalar(0, 0, 255), -1);
	          cv::imshow(window_name, color);
	         // cv::imshow(window_name, depth);
	          cv::waitKey(1);
	      }
	  }

	  cv::destroyAllWindows();
	  cv::waitKey(100);
  }

  void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
  {

      if(event.keyUp())
      	  {
    	  switch(event.getKeyCode())
    	  {
          case 27:
          case 'q':
        	  running = false;
          break;
          case ' ':
          case 's':
        	  save = true;
          break;
    	  }
      	  }

  }

  void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }

  void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
  {
      double *itC = cameraMatrix.ptr<double>(0, 0);
      for(size_t i = 0; i < 9; ++i, ++itC)
      {
    	  *itC = cameraInfo->K[i];
      }
  }

 //函数名称：depthcontrol
 //函数功能：控制显示一定范围内的深度图像
 //输入：depth图像，深度min距离,深度max距离
 //输出：depthshow 限定范围内可视化深度图像
 void depthcontrol(const cv::Mat &depth, cv::Mat &depthshow,const int mindis,const int maxdis)
 {
	  cv::Mat tmp = cv::Mat(depth.rows, depth.cols, CV_8U);
      #pragma omp parallel for
	  for(int r = 0; r < depth.rows; ++r)
	  {
		  const uint16_t *itI = depth.ptr<uint16_t>(r);
		  uint8_t *itO = tmp.ptr<uint8_t>(r);

		  for(int c = 0; c < depth.cols; ++c, ++itI, ++itO)
		  {
			  if((*itI>mindis) && (*itI<maxdis))
			  {
				  *itO = (uint8_t)(((*itI -mindis)*10)%256);
			  }
			  
			  else
			  {
				  *itO=(uint8_t)0;
			  }
			 // ROS_INFO("it=%d",*itO);
		  }
		}
	
		 // Mat depthshow=tmp;
		cv::applyColorMap(tmp, depthshow, cv::COLORMAP_JET);

 
 }



  void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
  {
	  cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
	  const uint32_t maxInt = 255;

      #pragma omp parallel for
	  for(int r = 0; r < in.rows; ++r)
	  {
		  const uint16_t *itI = in.ptr<uint16_t>(r);
		  uint8_t *itO = tmp.ptr<uint8_t>(r);

		  for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
		  {
			  *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
		  }
	  }

	  cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
  }

  void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
  {
	  out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

      #pragma omp parallel for
	  for(int r = 0; r < inC.rows; ++r)
	  {
		  const cv::Vec3b
		  *itC = inC.ptr<cv::Vec3b>(r),
		  *itD = inD.ptr<cv::Vec3b>(r);
		  cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

		  for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
		  {
			  itO->val[0] = (uint8_t)(itC->val[0] +0.6* itD->val[0]) >> 1;
			  itO->val[1] = (uint8_t)(itC->val[1] + 0.6*itD->val[1]) >> 1;
			  itO->val[2] = (uint8_t)(itC->val[2] + 0.6*itD->val[2]) >> 1;
		  }
	  }
  }

  //函数名称：targetdetection
  //函数功能：探测所有的目标坐标点，以全局变量模式输出
  //输入：color,depth,combined,target的指针
  //输出：无

void targetdetection(const cv::Mat &color,const cv::Mat &depth,const cv::Mat &combined,cv::Mat &target)
{
	Mat colorin;
	colorin=color.clone();
	Mat depthin=depth;
	bool debug=0;
	Rect myrect(118,71,667,342);
// Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
// 		morphologyEx(deff_de, deff_de, MORPH_OPEN, element);
//   	  morphologyEx(deff_de, deff_de, MORPH_CLOSE, element);
// 		GaussianBlur(deff_de, deff_de, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
// 		blur( deff_de, deff_de, Size(3,3) );  
// 		imshow("blur deffde",deff_de);
 //下面进行深度二值化，筛选出竖直摆放的零件以及孔件
 cv::Mat tmp1 = cv::Mat(depth.rows, depth.cols, CV_8U);
 for(int r = 0; r < deff_de.rows; ++r)
	  {
		  const uint8_t *itI = deff_de.ptr<uint8_t>(r);
		  uint8_t *itO = tmp1.ptr<uint8_t>(r);

		  for(int c = 0; c < deff_de.cols; ++c, ++itI, ++itO)
		  {
			  if((*itI>37) && (*itI<85))
			  {
				  *itO = (uint8_t)(255);
			  }
			  
			  else
			  {
				  *itO=(uint8_t)0;
			  }
			
		  }
	  }
	  
	  Mat canny_output;
  	  vector<vector<Point> > contours;
  	  vector<Vec4i> hierarchy;
	
		Mat element = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
		morphologyEx(tmp1, tmp1, MORPH_OPEN, element);
  	  morphologyEx(tmp1, tmp1, MORPH_CLOSE, element);
		GaussianBlur(tmp1, tmp1, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
		blur( tmp1, tmp1, Size(3,3) );  
  	  Canny(tmp1, canny_output,30, 30* 3, 3);
		//Canny(deff_de, canny_output,30, 30* 3, 3);
imshow("canny_output",canny_output);
  	  findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
if (contours.size() ==1)
  	  {
  		  //cout << "all targets object missing" << endl;
  		  
  	  }
  	  else
  	  {
  		 
  		  
  		  
			int i=0;
			 count_hole=0;//记录检测到的此范围孔件数量
  		  for (i = 0; i < (int)contours.size(); i++)//循环此深度下所有轮廓
  		  {
				
				
				
				
				/**************debug模式*****************/
  			if(debug)//debug模式下检测场景下最大面积
			{
				vector<Moments> mu(contours.size());//霍夫向量矩阵
					 vector<Point2f> mc(contours.size());//中心点坐标向量存储矩阵
  			  if ((10 < contourArea(contours[i])) && (contourArea(contours[i])<5000))
  			  {
  				 
  			  
  		  drawContours(colorin, contours,i,Scalar(50), 1, 8);
			mu[i] = moments(contours[i], false);
						 mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
						 
						 circle(colorin, mc[i], 5, Scalar(0, 0, 255), -1, 8, 0);
					char tam1[100];
						sprintf(tam1, "(area=%0.0f)", contourArea(contours[i]));			
						putText(colorin, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
					
					
				imshow("debugmode",colorin);
				}
			}
			/************debug模式结束,正常工作模式如下*******************/
				if(!debug)
				{
					vector<Moments> mu(contours.size());//霍夫向量矩阵
					 vector<Point2f> mc(contours.size());//中心点坐标向量存储矩阵
					int num_con[7];//0是三棱柱，1是四棱柱，2是六棱柱，3是圆柱，4是三棱柱对应装配体，5是四棱柱对应装配体，6是六棱柱对应装配体，7是圆柱对应装配体
			/*********检测4个大孔件的坐标信息*********/
					
					if((contourArea(contours[i])>2000)&&(contourArea(contours[i])<2600))
					{
						 RotatedRect rect=minAreaRect(contours[i]);//画最小外接矩形
						 if((0.8*rect.size.height<rect.size.width) && (rect.size.width<1.2*rect.size.height))//
					  {
						
						mu[i] = moments(contours[i], false);
						 mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
						  if(myrect.contains(mc[i]))
						 {
							 count_hole++;
						switch(count_hole)
						{//先把孔件初略分类
						case 1:num_con[count_hole+3]=i;break;
						case 2:num_con[count_hole+3]=i;break;
						case 3:num_con[count_hole+3]=i;break;
						case 4:num_con[count_hole+3]=i;break;
						default:ROS_INFO("孔件数量超常");
						}
						 circle(colorin, mc[i], 5, Scalar(0, 0, 255), -1, 8, 0);//在彩色图上画出中心点
						//circle(tmp1, mc[i], 5, Scalar(100, 100, 100), -1, 8, 0);//在深度色图上画出中心点,debug用

						
						 Point2f P[4];
						rect.points(P);
							for(int j=0;j<=3;j++)
							{
								line(colorin,P[j],P[(j+1)%4],Scalar(120),0.9);
							}

							//ROS_INFO("%7.2f",rect.angle);角度
							float angle_hole=-rect.angle;
						//检测中心点深度从而判断姿态
						int posehole;
						//ROS_INFO("%u",depthin.at<uint16_t>(mc[i].y,mc[i].x));//中心点深度
						if(deff_de.at<uint8_t>(mc[i].y,mc[i].x)<55)
						{
							posehole=1;
						}
						else
						{
							posehole=-1;
						}

						/******/
						
						object[count_hole+3].state=posehole;
						object[count_hole+3].pose.x=mc[i].x;
						object[count_hole+3].pose.y=mc[i].y;
						object[count_hole+3].pose.rz=angle_hole;
						
						/****/
						char tam1[100];
						sprintf(tam1, "(hole%d,%d,%d,po=%d,an=%0.1f)", count_hole,(int)mc[i].x, (int)mc[i].y,posehole,angle_hole);			
						putText(colorin, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
					
					}
					  }
				  }

				  /***************检测4个装配件立着时的坐标信息*********************/
				  /************************立四棱柱****************************/
					if((contourArea(contours[i])>170)&&(contourArea(contours[i])<290))
					{
						 RotatedRect rect=minAreaRect(contours[i]);//画最小外接矩形
						 if((0.99*rect.size.width*rect.size.height)>contourArea(contours[i]) &&(0.8*rect.size.width*rect.size.height)<contourArea(contours[i]))
					  {
						  	
						mu[i] = moments(contours[i], false);
						 mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
						 if(myrect.contains(mc[i]))
						 {
								

						 float angle_qu=-rect.angle;
						 circle(colorin, mc[i], 5, Scalar(0, 0, 255), -1, 8, 0);//在彩色图上画出中心点
						char tam1[100];
						int pose=1;
						Point2f P[4];
						rect.points(P);
							for(int j=0;j<=3;j++)
							{
								line(colorin,P[j],P[(j+1)%4],Scalar(120),0.9);
							}
								
								/******/
						object[1].state=1;
						object[1].pose.x=mc[i].x;
						object[1].pose.y=mc[i].y;
						object[1].pose.rz=angle_qu;
						/****/



						sprintf(tam1, "(quadrangular,pose=%d,%d,%d,an=%0.1f)",pose,(int)mc[i].x, (int)mc[i].y,angle_qu);			
						putText(colorin, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
						 }
					  }
					}
					/*************立圆柱************/
					if((contourArea(contours[i])>150)&&(contourArea(contours[i])<240))
					{
						Point2f center; float radius;  
						minEnclosingCircle(contours[i],center,radius);  //画最小外接圆
						if(deff_de.at<uint8_t>(center.y,center.x)<50)
						{
						 if( (0.7*3.14*radius*radius)<contourArea(contours[i]))
					 		 {
						 if(myrect.contains(center))
						 		{
						 circle(colorin, center, 5, Scalar(0, 0, 255), -1, 8, 0);//在彩色图上画出中心点
						char tam1[100];
						int pose=1;
						circle(colorin,center,radius,Scalar(120),0.9);  //画圆
						///////***********///
						object[4].state=1;
						object[4].pose.x=center.x;
						object[4].pose.y=center.y;
						object[4].pose.rz=0;
						
						//*****************/
						
						
						sprintf(tam1, "(cylinder,pose=%d,%d,%d)",pose,(int)center.x, (int)center.y);				
						putText(colorin, tam1, Point(center.x,center.y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
								 }
					  		}
						}
					}
					/**************立六棱柱*****************/
					if((contourArea(contours[i])>110)&&(contourArea(contours[i])<190))
					{
						Point2f center; float radius;  
						minEnclosingCircle(contours[i],center,radius);  //画最小外接圆
						
						if(deff_de.at<uint8_t>(center.y,center.x)>50) //
						{
						 if( (0*3.14*radius*radius)<contourArea(contours[i])&&(3.14*radius*radius)>contourArea(contours[i]))
					 		 {
						 		if(myrect.contains(center))
						 		{
						 circle(colorin, center, 3, Scalar(0, 0, 255), -1, 8, 0);//在彩色图上画出中心点
						char tam1[100];
						int pose=1;
						circle(colorin,center,radius,Scalar(120),0.9);  //画圆
					//	sprintf(tam1, "(hexagonal,pose=%d,%d,%d)",pose,(int)center.x, (int)center.y);
						///////***********///
						object[3].state=1;
						object[3].pose.x=center.x;
						object[3].pose.y=center.y;
						object[3].pose.rz=0;
						
						//*****************/
						
						
						sprintf(tam1, "(hexagonal,pose=%d,%d,%d,%f)",pose,(int)center.x, (int)center.y,3.14*radius*radius/contourArea(contours[i]));//debug			
						putText(colorin, tam1, Point(center.x,center.y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
								 }
					  		}
						}
					}

					/***********立三棱柱***********************/
					if((contourArea(contours[i])>29)&&(contourArea(contours[i])<95))
					{
						mu[i] = moments(contours[i], false);
						 mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
						if(deff_de.at<uint8_t>(mc[i].y,mc[i].x)>50)
						{
						 if(myrect.contains(mc[i]))
						 {
								  	vector<vector<Point> >poly(contours.size());
						approxPolyDP(contours[i],poly[i],5,true);
					
						drawContours(colorin,poly,i,Scalar(50), 1, 8);
						 circle(colorin, mc[i], 1, Scalar(0, 0, 255), -1, 8, 0);//在彩色图上画出中心点
						char tam1[100];
						int pose=1;
					
					///////***********///
						object[0].state=1;
						object[0].pose.x=mc[i].x;
						object[0].pose.y=mc[i].y;
						object[0].pose.rz=0;
						
						//*****************/
						sprintf(tam1, "(triangular,pose=%d,%d,%d)",pose,(int)mc[i].x, (int)mc[i].y);			
						putText(colorin, tam1, Point(mc[i].x,mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
					
					  		}
						}
					}

				}	
		
			imshow("tam1",tmp1);
			}//contours循环结束
			

		}
		
//深度检测卧方块
cv::Mat tmp2 = cv::Mat(deff_de.rows, deff_de.cols, CV_8U);
 for(int r = 0; r < deff_de.rows; ++r)
	  {
		  const uint8_t *itI1 = deff_de.ptr<uint8_t>(r);
		  uint8_t *itO1 = tmp2.ptr<uint8_t>(r);

		  for(int c = 0; c < deff_de.cols; ++c, ++itI1, ++itO1)
		  {
			  if((*itI1>15) && (*itI1<35))
			  {
				  *itO1 = (uint8_t)(255);
			  }
			  
			  else
			  {
				  *itO1=(uint8_t)0;
			  }
			
		  }
	  }
	  
	  Mat canny_output2;
  	  vector<vector<Point> > contours2;
  	  vector<Vec4i> hierarchy2;
	
		Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(2,2));
		morphologyEx(tmp2, tmp2, MORPH_OPEN, element);
  	  morphologyEx(tmp2, tmp2, MORPH_CLOSE, element);
		GaussianBlur(tmp2, tmp2, Size(3, 3), 0.1, 0, BORDER_DEFAULT);
		blur( tmp2, tmp2, Size(3,3) );  
		imshow("tmp2",tmp2);
  	  Canny(tmp2, canny_output2,30, 30* 3, 3);
imshow("canny_output2",canny_output2);
  	  findContours(canny_output2, contours2, hierarchy2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


/**************************************/
				if (contours2.size() <=1)
					{
						//cout << "all targets object missing" << endl;
						
					}
					else
					{
						vector<Moments> mu(contours2.size());//霍夫向量矩阵
						vector<Point2f> mc(contours2.size());//中心点坐标向量存储矩阵
									
						
					for (int i = 0; i < (int)contours2.size(); i++)//循环此深度下所有轮廓
		{

									/**************debug模式,检测卧姿深度面积*****************/
  			if(debug)//debug模式下检测场景下最大面积
			{
				
  			  if ((100< contourArea(contours2[i])) && (contourArea(contours2[i])<1000))
  			  {
  				 
  			  
  		  drawContours(colorin, contours2,i,Scalar(200,100,0), 1, 8);
			mu[i] = moments(contours2[i], false);
						 mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
						 
						 circle(colorin, mc[i], 5, Scalar(255, 0, 255), -1, 8, 0);
					char tam1[100];
						sprintf(tam1, "(area=%0.0f)", contourArea(contours2[i]));			
						putText(colorin, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
					
					
				imshow("debugmode2",colorin);
				}
			}
/************debug模式结束,正常工作模式如下*******************/
							if(!debug)
								{

								/************************卧四棱柱****************************/
									if( (contourArea(contours2[i])>460) && (contourArea(contours2[i])<630) )
									{
										RotatedRect rect=minAreaRect(contours2[i]);//画最小外接矩形
										if(1)
									{
										
										mu[i] = moments(contours2[i], false);
										mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
										 if(myrect.contains(mc[i]))
										 {
										float angle_qu=-rect.angle;
										circle(colorin, mc[i], 5, Scalar(255, 0, 255), -1, 8, 0);//在彩色图上画出中心点
										char tam1[100];
										int pose=-1;
										Point2f P[4];
										rect.points(P);
											for(int j=0;j<=3;j++)
											{
												line(colorin,P[j],P[(j+1)%4],Scalar(120),0.9);
											}
											
										///////***********///
											object[1].state=-1;
											object[1].pose.x=mc[i].x;
											object[1].pose.y=mc[i].y;
											object[1].pose.rz=angle_qu;
											
											//*****************/
										sprintf(tam1, "(quadrangular,pose=%d,%d,%d,an=%0.1f)",pose,(int)mc[i].x, (int)mc[i].y,angle_qu);			
										putText(colorin, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
										 }
									}
									}

									/*********************卧圆柱************************/
									if( (contourArea(contours2[i])>200) && (contourArea(contours2[i])<320) )
									{
										RotatedRect rect=minAreaRect(contours2[i]);//画最小外接矩形
										if((rect.size.width/rect.size.height)<4 && (rect.size.width/rect.size.height)>1/4)
									{
										mu[i] = moments(contours2[i], false);
										mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
										
										 if(myrect.contains(mc[i]))
										 {
										float angle_qu=-rect.angle;
										circle(colorin, mc[i], 5, Scalar(255, 0, 255), -1, 8, 0);//在彩色图上画出中心点
										char tam1[100];
										int pose=-1;
										Point2f P[4];
										rect.points(P);
											for(int j=0;j<=3;j++)
											{
												line(colorin,P[j],P[(j+1)%4],Scalar(120),0.9);
											}
											///////***********///
											object[4].state=-1;
											object[4].pose.x=mc[i].x;
											object[4].pose.y=mc[i].y;
											object[4].pose.rz=angle_qu;
											
											//*****************/
										sprintf(tam1, "(cylinder,pose=%d,%d,%d,an=%0.1f)",pose,(int)mc[i].x, (int)mc[i].y,angle_qu);			
										putText(colorin, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
										 }
									}
									}
									/*********************卧六棱柱************************/
									if( (contourArea(contours2[i])>200) && (contourArea(contours2[i])<400) )
									{
										RotatedRect rect=minAreaRect(contours2[i]);//画最小外接矩形
										if((rect.size.width/rect.size.height)>4 || (rect.size.width/rect.size.height)<1/4)
									{
										mu[i] = moments(contours2[i], false);
										mc[i] = Point2d(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);//求中心点坐标
										 if(myrect.contains(mc[i]))
										 {
										float angle_qu=-rect.angle;
										circle(colorin, mc[i], 5, Scalar(255, 0, 255), -1, 8, 0);//在彩色图上画出中心点
										char tam1[100];
										int pose=-1;
										Point2f P[4];
										rect.points(P);
											for(int j=0;j<=3;j++)
											{
												line(colorin,P[j],P[(j+1)%4],Scalar(120),0.9);
											}

											///////***********///
											object[3].state=-1;
											object[3].pose.x=mc[i].x;
											object[3].pose.y=mc[i].y;
											object[3].pose.rz=angle_qu;
											
											//*****************/
										sprintf(tam1, "(hexagonal,pose=%d,%d,%d,an=%0.1f)",pose,(int)mc[i].x, (int)mc[i].y,angle_qu);			
										putText(colorin, tam1, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);
										 }
									}
									}
							}

						}

	}



//下面进行彩色二值化，筛选出水平摆放的零件 再通过深度进行筛选

	Mat imgHSV;
	vector<Mat> hsvSplit;
	cvtColor(color, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	//因为我们读取的是彩色图，直方图均衡化需要在HSV空间做
	split(imgHSV, hsvSplit);
	equalizeHist(hsvSplit[2],hsvSplit[2]);
	merge(hsvSplit,imgHSV);
	Mat imgThresholded;
/**************彩色参数*********/
		int iLowH = 12;
		int iHighH = 45;
		int iLowS = 0;
		int iHighS = 255;
		int iLowV = 124;
		int iHighV = 255;


	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
	
	//开操作 (去除一些噪点)
	Mat element1 = getStructuringElement(MORPH_RECT, Size(3,3));
	morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element1);

	//闭操作 (连接一些连通域)
	morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element1);

  GaussianBlur(imgThresholded, imgThresholded, Size(3, 3), 0.1, 0, BORDER_DEFAULT);//
  	  blur(imgThresholded, imgThresholded, Size(3, 3)); //
	//canny算子，边缘检测
  	  Mat canny_outputco;
  	  vector<vector<Point> > contoursco;
  	  vector<Vec4i> hierarchyco;
  	  Canny(imgThresholded, canny_outputco, 30, 30 * 3, 3);

  	  findContours(canny_outputco, contoursco, hierarchyco, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
     // drawContours(colorin, contours,i,Scalar(50), 1, 8);
	 imshow("cannycolor",canny_outputco);
  	  //********************
  	  if (contoursco.size() == 0)
  	  {
  		  cout << "all color targets  missing" << endl;
  		 
  	  }
  	  else
  	  {
  		  
			int i=0;
			
  		  for (i = 0; i < (int)contoursco.size(); i++)
  		  {
  			   
  			 vector<Moments> muc(contoursco.size());//霍夫向量矩阵
			 vector<Point2f> mcc(contoursco.size());//中心点坐标向量存储矩阵

				 
/*****************************检测4个装配件卧着时的坐标信息****************************/
				  /************************卧三棱柱****************************/
  			  if ((480< contourArea(contoursco[i])) && (contourArea(contoursco[i])<600))
  			 	 {
  				// cout<<"1"<<endl;
  			  RotatedRect rect=minAreaRect(contoursco[i]);//画最小外接矩形
						 if((rect.size.width/rect.size.height)>1 || (rect.size.width/rect.size.height)<1)
					  {
						  	
						muc[i] = moments(contoursco[i], false);
						 mcc[i] = Point2d(muc[i].m10 / muc[i].m00, muc[i].m01 / muc[i].m00);//求中心点坐标
						 
						 if(myrect.contains(mcc[i]))
						 {
							 //cout<<"1"<<endl;
							if(deff_de.at<uint8_t>(mcc[i].y,mcc[i].x)<10)//

						 {	

						 float angle_qu=-rect.angle;
						 circle(colorin, mcc[i], 5, Scalar(100, 100, 100), -1, 8, 0);//在彩色图上画出中心点
						
						int pose=-1;
						Point2f P[4];
						rect.points(P);
							for(int j=0;j<=3;j++)
							{
								line(colorin,P[j],P[(j+1)%4],Scalar(100),0.9);
							}
  		 			
					 
						 circle(colorin, mcc[i], 5, Scalar(0, 0, 255), -1, 8, 0);

											///////***********///
											object[0].state=-1;
											object[0].pose.x=mcc[i].x;
											object[0].pose.y=mcc[i].y;
											object[0].pose.rz=angle_qu;
											
											//*****************/

					char tam1[100];
						sprintf(tam1, "(triangular,pose=%d,%d,%d,an=%0.1f)",pose,(int)mcc[i].x, (int)mcc[i].y,angle_qu);			
						putText(colorin, tam1, Point(mcc[i].x, mcc[i].y), FONT_HERSHEY_SIMPLEX, 0.4, cvScalar(0, 0,0), 1);}
					  }
					  }
  		  			}
			
			//ROS_INFO("area=%f",maxArea);
		//	drawContours(colorImg, contours,max,Scalar(255), 1, 8);
			}
		}
	imshow("Thresholdedcolor", imgThresholded); //show the thresholded image
	

	imshow("combined1",colorin);


}

void absd(const cv::Mat &pre_de,const cv::Mat &pre_co,const cv::Mat &depth,const cv::Mat &color)
{
	
	
	//absdiff(pre_de,depth,deff_de);
	absdiff(pre_co,color,deff_co);
int desktall=955;
 for(int r = 0; r < depth.rows; ++r)
	  {
		  const uint16_t *itI1 = depth.ptr<uint16_t>(r);
		  const uint16_t *itI2 = pre_de.ptr<uint16_t>(r);
		  uint8_t *itO = deff_de.ptr<uint8_t>(r);
		  for(int c = 0; c <depth.cols; ++c, ++itI1, ++itI2,++itO)
		  {
			if(*itI2>desktall)
			{
			*itO = (uint8_t)abs(*itI1-desktall);
			}
			else
			{
			*itO = (uint8_t)abs(*itI1-*itI2);
			}
				//*itO = (uint8_t)abs(*itI1-*itI2);
		  }
	  }
	
Rect rect(109, 60, 667, 354);
Mat deff_de_roi = deff_de(rect);


imshow("deff_de_roi",deff_de_roi);
	/******************/  
	Mat deffdeshow;
	  cv::applyColorMap(deff_de, deffdeshow, cv::COLORMAP_JET);
	imshow("deff_de",deff_de);
//	imshow("deff_co",deff_co);
	imshow("deffdeshow",deffdeshow);
}

// bool inrect(float x,float y,cv::Rect myrect)
// {
// if(myrect.x<x)
// }


  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
  {
	  const float badPoint = std::numeric_limits<float>::quiet_NaN();

      #pragma omp parallel for
	  for(int r = 0; r < depth.rows; ++r)
	  {
		  pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
		  const uint16_t *itD = depth.ptr<uint16_t>(r);
		  const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
		  const float y = lookupY.at<float>(0, r);
		  const float *itX = lookupX.ptr<float>();

		  for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
		  {
			  register const float depthValue = *itD / 1000.0f;
			  // Check for invalid measurements
			  if(*itD == 0)
			  {
				  // not valid
				  itP->x = itP->y = itP->z = badPoint;
				  itP->rgba = 0;
				  continue;
			  }
			  itP->z = depthValue;
			  itP->x = *itX * depthValue;
			  itP->y = y * depthValue;
			  itP->b = itC->val[0];
			  itP->g = itC->val[1];
			  itP->r = itC->val[2];
			  itP->a = 255;
		  }
	  }
  }

  void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
  {
	  oss.str("");
	  oss << "./" << std::setfill('0') << std::setw(4) << frame;
	  const std::string baseName = oss.str();
	  const std::string cloudName = baseName + "_cloud.pcd";
	  const std::string colorName = baseName + "_color.jpg";
	  const std::string depthName = baseName + "_depth.png";
	  const std::string depthColoredName = baseName + "_depth_colored.png";

	  OUT_INFO("saving cloud: " << cloudName);
	  writer.writeBinary(cloudName, *cloud);
	  OUT_INFO("saving color: " << colorName);
	  cv::imwrite(colorName, color, params);
	  OUT_INFO("saving depth: " << depthName);
	  cv::imwrite(depthName, depth, params);
	  OUT_INFO("saving depth: " << depthColoredName);
	  cv::imwrite(depthColoredName, depthColored, params);
      OUT_INFO("saving complete!");
      ++frame;
  }

  void createLookup(size_t width, size_t height)
  {
	  const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
	  const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
	  const float cx = cameraMatrixColor.at<double>(0, 2);
	  const float cy = cameraMatrixColor.at<double>(1, 2);
	  float *it;

	  lookupY = cv::Mat(1, height, CV_32F);
	  it = lookupY.ptr<float>();
	  for(size_t r = 0; r < height; ++r, ++it)
	  {
		  *it = (r - cy) * fy;
	  }

	  lookupX = cv::Mat(1, width, CV_32F);
	  it = lookupX.ptr<float>();
	  for(size_t c = 0; c < width; ++c, ++it)
	  {
		  *it = (c - cx) * fx;
	  }
  }
};

void help(const std::string &path)
{
	std::cout << path << FG_BLUE " [options]" << std::endl
            << FG_GREEN "  name" NO_COLOR ": " FG_YELLOW "'any string'" NO_COLOR " equals to the kinect2_bridge topic base name" << std::endl
            << FG_GREEN "  mode" NO_COLOR ": " FG_YELLOW "'qhd'" NO_COLOR ", " FG_YELLOW "'hd'" NO_COLOR ", " FG_YELLOW "'sd'" NO_COLOR " or " FG_YELLOW "'ir'" << std::endl
            << FG_GREEN "  visualization" NO_COLOR ": " FG_YELLOW "'image'" NO_COLOR ", " FG_YELLOW "'cloud'" NO_COLOR " or " FG_YELLOW "'both'" << std::endl
            << FG_GREEN "  options" NO_COLOR ":" << std::endl
            << FG_YELLOW "    'compressed'" NO_COLOR " use compressed instead of raw topics" << std::endl
            << FG_YELLOW "    'approx'" NO_COLOR " use approximate time synchronization" << std::endl;
}

int main(int argc, char **argv)
{
	#if EXTENDED_OUTPUT
	ROSCONSOLE_AUTOINIT;
	if(!getenv("ROSCONSOLE_FORMAT"))
	{
    ros::console::g_formatter.tokens_.clear();
    ros::console::g_formatter.init("[${severity}] ${message}");
	}
	#endif

	ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::NodeHandle nh;
	ros::Timer timer = nh.createTimer(ros::Duration(0.1),TimerCallback);
        pose_arm_pub= nh.advertise<geometry_msgs::Vector3>("obj_vel", 1000);
	ros::NodeHandle n;
	ros::Timer timer1 = n.createTimer(ros::Duration(0.1),TimerCallback1);
        pose_car_pub= n.advertise<geometry_msgs::Vector3>("/pose_3d/pose_car_pub", 1000);

	if(!ros::ok())
	{
		return 0;
	}

	std::string ns = K2_DEFAULT_NS;
	std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
	std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
	bool useExact = true;
	bool useCompressed = false;
	Receiver::Mode mode = Receiver::MYDEPTH;
	topicColor = "/" + ns + topicColor;
	topicDepth = "/" + ns + topicDepth;
	OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
	OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);

	Receiver receiver(topicColor, topicDepth, useExact, useCompressed);

	OUT_INFO("starting receiver...");
	/////
	//OUT_INFO("Please click mouse in color viewer...");
	receiver.run(mode);

	ros::shutdown();
	return 0;
}

