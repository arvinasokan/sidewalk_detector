/*
   / \   _ ____   _(_)_ __      / \   ___  ___ | | ____ _ _ __  
  / _ \ | '__\ \ / / | '_ \    / _ \ / __|/ _ \| |/ / _` | '_ \ 
 / ___ \| |   \ V /| | | | |  / ___ \\__ \ (_) |   < (_| | | | |
/_/   \_\_|    \_/ |_|_| |_| /_/   \_\___/\___/|_|\_\__,_|_| |_|
                                                                
*/

#include <ros/ros.h>

//  OpenCV and CVbridge includes
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
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>

// Sidewalk Filter
#include "Sidewalk_Filter.h"

static const std::string OPENCV_WINDOW = "Image window";


class SidewalkDetector
{
  ros::NodeHandle nh;
  ros::Publisher pub_in,pub_out;
  ros::Subscriber point_cloud_sub;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  Sidewalk_Filter sf;
  cv::Mat reduction_mask,additive_mask;
public:
  SidewalkDetector():it_(nh)
  {
    ros::NodeHandle pnh;
    // Subscriber and publisher for point cloud stream 
     point_cloud_sub = nh.subscribe ("/camera/depth/points", 1, &SidewalkDetector::point_cloud_cb, this);
     pub_in = nh.advertise<sensor_msgs::PointCloud2> ("/sidewalk_detector/depth/points_in", 1);
     pub_out = nh.advertise<sensor_msgs::PointCloud2> ("/sidewalk_detector/depth/points_out", 1);
    //ros::Subscriber rgb_sub = nh.subscribe ("/camera/color/image_raw", 1, rgd_cb);

    // Subscribers & Publishers for RGB stream using cv_bridge
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &SidewalkDetector::rgbCb, this);
    image_pub_ = it_.advertise("/sidewalk_detector/color/image_raw", 1);
    
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

    std::vector<cv::Mat> channels; 
    cv::Mat img_hist_equalized;
    //Rotating the image, transpose and flip is effective   
    cv::transpose(cv_ptr->image, cv_ptr->image);   
    cv::flip(cv_ptr->image, cv_ptr->image,1);   
    cv::transpose(cv_ptr->image, cv_ptr->image);
    cv::flip(cv_ptr->image, cv_ptr->image,1);
    
    //Increasing the color contrast for better segmentation
    cv::cvtColor(cv_ptr->image, img_hist_equalized, CV_BGR2YCrCb); 
    cv::split(img_hist_equalized,channels); //split the image into channels
    cv::equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)
    cv::merge(channels,img_hist_equalized); //merge 3 channels including the modified 1st channel into one image
    cv::cvtColor(img_hist_equalized, img_hist_equalized, CV_YCrCb2BGR);
   
    cv::Mat hsi_image;
    cv::Mat filtered_image;
    cv::Mat hsv_image;
    cv::Mat red_mask(cv_ptr->image.rows, cv_ptr->image.cols, cv_ptr->image.type()); 
    cv::Mat simple_filter;
    cv::Mat imageDest;
    
    //Gaussian Blurring
    for ( int i = 1; i < 25; i = i + 2 )
    { 
      cv::GaussianBlur( cv_ptr->image, filtered_image, cv::Size( i, i ), 0, 0 );
      //if( cv::display_dst( DELAY_BLUR ) != 0 ) { return 0; } 
    }
    //HSV Color Conversion
    cv::cvtColor(filtered_image, hsv_image, CV_BGR2HSV);
    cv::cvtColor(filtered_image, simple_filter, CV_RGB2GRAY);
    cv::threshold(simple_filter, simple_filter, 130, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
    cv::Mat reference_mask = sf.trapezoidal_roi(cv_ptr->image);
    cv::Mat background_mask = 255 - reference_mask;
    cv::Mat lucky_mask;
    cv::Mat background_histogram;   
    cv::Mat reference_histogram;
    
    if(!reduction_mask.empty())
    {
      //cv::bitwise_and(background_mask,reduction_mask,background_mask,cv::Mat());  
      background_mask = background_mask + reduction_mask + simple_filter;
     }


 
    background_histogram = sf.histogram_finder(background_mask, filtered_image,false);   
    reference_histogram = sf.histogram_finder(reference_mask, filtered_image,false);  
    lucky_mask = sf.back_projection(background_histogram,reference_histogram,filtered_image,false);  
    
    //Update the detected sidewalk with the training area to find an updated training histogram.
    reference_mask = reference_mask + lucky_mask; 
    reduction_mask = 225 - lucky_mask;
    //}
     cv::bitwise_and(lucky_mask,simple_filter,lucky_mask,cv::Mat());
     
     // Perform Blob Removal
     //cv::Mat image = lucky_mask.clone();
     //IplImage copy = image;
     //IplImage* ipl_image = &copy;
     //IplImage* frame;
     //frame = sf.remove_small_objects(ipl_image,8000);
     //cv::Mat final_mask(frame);
     
     // Dialate & Erode
     cv::Mat se = cv::getStructuringElement(2,cv::Size(5,10));
     //cv::erode( lucky_mask, lucky_mask, se );
     cv::dilate( lucky_mask, lucky_mask, se );
    
    // Red Masking
    cv::Mat new_image; 
    red_mask.setTo(cv::Scalar(0,0,255));
    red_mask.copyTo(new_image,lucky_mask); 
    double alpha,beta;
    alpha = 0.5;
    beta = ( 1.0 - alpha );
    addWeighted(new_image, alpha,cv_ptr->image , beta, 0.0, imageDest);
    //cv::imshow(OPENCV_WINDOW, imageDest);
    //cv::waitKey(1);

    sensor_msgs::ImagePtr rgb_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageDest).toImageMsg();
    // Output modified video stream
    image_pub_.publish(rgb_out);
  }
void point_cloud_cb(const sensor_msgs::PointCloud2ConstPtr& msg){

  
    // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud2 in_points;
  sensor_msgs::PointCloud2 out_points;
  pcl::PCLPointCloud2 voxel;
  //pcl::PointCloud<pcl::PointXYZRGB> input_cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl_conversions::toPCL(*msg, *cloud);
  
  //pcl::PCLPointCloud2 cloud_filtered;
 
  output = *msg;
  pcl::fromROSMsg (*msg,*raw_cloud);
  
  
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (voxel);
  
  pcl::fromPCLPointCloud2(voxel,*cloud_filtered);
  //pcl::removeNaNFromPointCloud(*cloud, *cloud, mapping);
  


  //Smoothing out the point cloud
  // The output will also contain the normals.
  

  /*
  pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>); 

  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> filter;
  
  filter.setInputCloud(raw_cloud);
  // Use all neighbors in a radius of 3cm.
  filter.setSearchRadius(0.03);
  // If true, the surface and normal are approximated using a polynomial estimation
  // (if false, only a tangent one).
  filter.setPolynomialFit(true);
  // We can tell the algorithm to also compute smoothed normals (optional).
  filter.setComputeNormals(true);
  // kd-tree object for performing searches.
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree;
  filter.setSearchMethod(kdtree);
 
  filter.process(*smoothedCloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  copyPointCloud(*smoothedCloud,*mls_cloud);
*/


  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-1.5, -0.2);
  pass.filter (*input_cloud);
  pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.05);
    // Create the filtering object
  seg.setInputCloud (input_cloud);
  seg.segment (*inliers, *coefficients); 

  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (input_cloud);
  extract.setIndices (inliers);
  
  //extracting Points in
  extract.setNegative (false);
  extract.filter (*cloud_p);
  //extracting Point out
  extract.setNegative (true);
  extract.filter (*cloud_f);
  
  //cloud_filtered.swap (cloud_f);
  *cloud_f = *cloud_f + *cloud_filtered;

  pcl::toROSMsg (*cloud_p, in_points);
  pcl::toROSMsg (*cloud_f, out_points);    

  // Publish the data.
 pub_in.publish(in_points);
 pub_out.publish(out_points);

}
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  SidewalkDetector ic;
  ros::spin();
  return 0;
}