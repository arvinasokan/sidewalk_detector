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

  private:
        cv::Mat src;    

};
#endif
