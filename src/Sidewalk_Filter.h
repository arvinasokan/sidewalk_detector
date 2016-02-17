#ifndef __SIDEWALK_FILTER_H__
#define __SIDEWALK_FILTER_H__

#include <iostream>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>  


class Sidewalk_Filter
{

  public:
       Sidewalk_Filter();
       ~Sidewalk_Filter();
       cv::Mat rgb_to_hsi(const cv::Mat& src);
       cv::Mat trapezoidal_roi(const cv::Mat& src);
       cv::Mat histogram_finder(const cv::Mat& src, const cv::Mat& original,const bool is_onedimensional);
       std::vector<cv::Mat> hsi_histogram_finder(const cv::Mat& src, const cv::Mat& original,const bool is_onedimensional);    
      // int frequency_finder(const cv::MatND& src, const cv::);
       cv::Mat back_projection(const cv::MatND& src_hist,const cv::MatND& reference_histogram, const cv::Mat& background_image,const bool is_hsi);
       cv::Mat hsi_back_projection(const std::vector<cv::Mat>& src_hist,const std::vector<cv::Mat>& reference_vector, const cv::Mat& background_image);
       cv::Mat asSamplesVectors(const cv::Mat& img); 
       IplImage* remove_small_objects( IplImage* img_in, int size );
  private:
        cv::Mat src;    
        cv::Mat hsi;
        cv::Mat original;  
};
#endif
