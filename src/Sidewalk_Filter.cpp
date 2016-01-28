#include "Sidewalk_Filter.h"

Sidewalk_Filter::Sidewalk_Filter()
{
}
Sidewalk_Filter::~Sidewalk_Filter()
{
}

cv::Mat Sidewalk_Filter::rgb_to_hsi(const cv::Mat& src)
 {
  if(src.empty())
    std::cerr << "Error : Could not receive stream" << std::endl;
  cv::Mat hsi(src.rows, src.cols, src.type());

  float r, g, b, h, s, in;

  for(int i = 0; i < src.rows; i++)
    {
      for(int j = 0; j < src.cols; j++)
        {
          b = src.at<cv::Vec3b>(i, j)[0];
          g = src.at<cv::Vec3b>(i, j)[1];
          r = src.at<cv::Vec3b>(i, j)[2];

          in = (b + g + r) / 3;

          int min_val = 0;
          min_val = std::min(r, std::min(b,g));

          s = 1 - 3*(min_val/(b + g + r));
          if(s < 0.00001)
            {
                  s = 0;
            }else if(s > 0.99999){
                  s = 1;
            }

          if(s != 0)
            {
              h = 0.5 * ((r - g) + (r - b)) / sqrt(((r - g)*(r - g)) + ((r - b)*(g - b)));
              h = acos(h);

              if(b <= g)
                {
                  h = h;
                } else{
                  h = ((360 * 3.14159265) / 180.0) - h;
                }
            }

          hsi.at<cv::Vec3b>(i, j)[0] = (h * 180) / 3.14159265;
          hsi.at<cv::Vec3b>(i, j)[1] = s*100;
          hsi.at<cv::Vec3b>(i, j)[2] = in;
        }
    } 
    return hsi;
  }
cv::Mat Sidewalk_Filter::trapezoidal_roi(const cv::Mat& src)
{
  if(src.empty())
    std::cerr << "Error : Could not receive stream" << std::endl;
  cv::Mat mask = cvCreateMat(src.rows, src.cols, CV_8UC1);
  for(int i=0; i<mask.cols; i++)
   for(int j=0; j<mask.rows; j++)
       mask.at<uchar>(cv::Point(i,j)) = 0;
  cv::Mat imageDest = cvCreateMat(src.rows, src.cols, CV_8UC3);
  std::vector<cv::Point> ROI_Poly, ROI_Vertices;
  ROI_Vertices.push_back(cv::Point(220,350));
  ROI_Vertices.push_back(cv::Point(420,350));
  ROI_Vertices.push_back(cv::Point(520,480));
  ROI_Vertices.push_back(cv::Point(120,480));
  approxPolyDP(ROI_Vertices, ROI_Poly, 1.0, true);
  fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0); 
  src.copyTo(imageDest, mask); 
  return imageDest;
}
cv::Mat Sidewalk_Filter::histogram_finder(const cv::Mat& src , const cv::Mat& original)
{
  cv::Mat dst;
  std::vector<cv::Mat> bgr_planes;
  split( src, bgr_planes );

  /// Establish the number of bins
  int histSize = 640;

  /// Set the ranges ( for B,G,R) )
  float range[] = {0, 360};
  float irange[] = {0, 1};
  const float* histRange = { range };
  const float* ihistRange = { irange };
  bool uniform = true; bool accumulate = false;

  cv::Mat b_hist, g_hist, r_hist;

  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  //calcHist( &bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 640; int hist_h = 480;
  int bin_w = cvRound( (double) hist_w/histSize );

  cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );
  int NORM_MINMAX = 32;
  /// Normalize the result to [ 0, histImage.rows ]
  normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat() );
  //normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat() );
  normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, cv::Mat() );

  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
      cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       cv::Scalar( 255, 0, 0), 2, 8, 0  );
      //cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
        //               cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
          //             cv::Scalar( 0, 255, 0), 2, 8, 0  );
     cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                     cv::Scalar( 0, 0, 255), 2, 8, 0  );
  

  }
  cv::Mat sidewalk_mask = original;
  
for(int i = 0; i < src.cols; i++)
    {
      if(cvRound(b_hist.at<float>(i)) > 4 || cvRound(r_hist.at<float>(i)) > 10 )
         {
      for(int j = 0; j < src.rows; j++)
        {
         
          sidewalk_mask.at<cv::Vec3b>(cv::Point(i,j))[0] = 0;
          sidewalk_mask.at<cv::Vec3b>(cv::Point(i,j))[1] = 0;
          sidewalk_mask.at<cv::Vec3b>(cv::Point(i,j))[2] = 0;
        }
        }
      }
  
  /// Display
  return sidewalk_mask;
}