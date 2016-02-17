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



static const std::string OPENCV_WINDOW = "Image window";

class SidewalkDetector
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  //image_transport::Subscriber image_sub_;
  //image_transport::Publisher image_pub_;
  
public:
  SidewalkDetector()
    : it_(nh)
  {
    // Subscriber and publisher for point cloud stream 
    ros::Subscriber point_cloud_sub = nh.subscribe ("/camera/depth/points", 1, point_cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("segmented", 1);
    //ros::Subscriber rgb_sub = nh.subscribe ("/camera/color/image_raw", 1, rgd_cb);

    // Subscribers & Publishers for RGB stream using cv_bridge
    image_transport::Subscriber image_sub_ = it_.subscribe("/camera/color/image_raw", 1, 
      &SidewalkDetector::rgbCb, this);
    image_transport::Publisher image_pub_ = it_.advertise("/camera/modified_raw/", 1);

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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
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