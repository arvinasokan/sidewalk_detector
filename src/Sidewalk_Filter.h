#ifndef __SIDEWALK_FILTER_H__
#define __SIDEWALK_FILTER_H__

#include <iostream>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class Sidewalk_Filter
{

  public:
       Sidewalk_Filter();
       ~Sidewalk_Filter();
       cv::Mat rgb_to_hsi(const cv::Mat& src);
       cv::Mat trapezoidal_roi(const cv::Mat& src);
       cv::Mat histogram_finder(const cv::Mat& src, const cv::Mat& original);   
  private:
        cv::Mat src;    
        cv::Mat hsi;
        cv::Mat original;  
};
#endif
