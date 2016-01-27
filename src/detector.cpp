#include <ros/ros.h>

//OpenCV and CVbridge includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL specific inclues
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Sidewalk_Filter.h"

static const std::string OPENCV_WINDOW = "Image window";

class SidewalkDetector
{
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber point_cloud_sub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  Sidewalk_Filter sf;
public:
  SidewalkDetector():it_(nh)
  {
  	ros::NodeHandle pnh;
    // Subscriber and publisher for point cloud stream 
     point_cloud_sub = nh.subscribe ("/camera/depth/points", 1, &SidewalkDetector::point_cloud_cb, this);
     pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/depth/segmented", 1);
    //ros::Subscriber rgb_sub = nh.subscribe ("/camera/color/image_raw", 1, rgd_cb);

    // Subscribers & Publishers for RGB stream using cv_bridge
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &SidewalkDetector::rgbCb, this);
    image_pub_ = it_.advertise("/camera/modified_raw/", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~SidewalkDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
  void rgbCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //Rotating the image, transpose and flip is effective   
    cv::transpose(cv_ptr->image, cv_ptr->image);   
    cv::flip(cv_ptr->image, cv_ptr->image,1);   
    cv::transpose(cv_ptr->image, cv_ptr->image);
    cv::flip(cv_ptr->image, cv_ptr->image,1);
    cv::Mat hsi_image;
    cv::Mat filtered_image;
    //int display_dst( int delay );
    //int DELAY_BLUR = 100;

    for ( int i = 1; i < 7; i = i + 2 )
    { 
    	cv::GaussianBlur( cv_ptr->image, filtered_image, cv::Size( i, i ), 0, 0 );
      //if( cv::display_dst( DELAY_BLUR ) != 0 ) { return 0; } 
    }
    hsi_image = sf.rgb_to_hsi(filtered_image);

    cv::Mat mask = cv::Mat::zeros(hsi_image.size(), CV_8UC1);
    cv::Point circleCenter(mask.cols / 2, mask.rows / 2);
    int radius = 50;
    cv::circle(mask, circleCenter, radius, CV_RGB(255, 255, 255));
    cv::Mat imagePart = cv::Mat::zeros(hsi_image.size(), hsi_image.type());
    hsi_image.copyTo(imagePart, mask);
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, hsi_image);
    cv::waitKey(1);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void point_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *msg;

  // Publish the data.
  pub.publish (output);
}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  SidewalkDetector ic;
  ros::spin();
  return 0;
}