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
  cv::Mat trapezoidal_mask = cvCreateMat(src.rows, src.cols, CV_8UC1);
  for(int i=0; i<trapezoidal_mask.cols; i++)
   for(int j=0; j<trapezoidal_mask.rows; j++)
       trapezoidal_mask.at<uchar>(cv::Point(i,j)) = 0;
  cv::Mat imageDest = cvCreateMat(src.rows, src.cols, CV_8UC3);
  std::vector<cv::Point> ROI_Poly, ROI_Vertices;
  ROI_Vertices.push_back(cv::Point(120,300));
  ROI_Vertices.push_back(cv::Point(520,300));
  ROI_Vertices.push_back(cv::Point(580,480));
  ROI_Vertices.push_back(cv::Point(60,480));
  approxPolyDP(ROI_Vertices, ROI_Poly, 1.0, true);
  fillConvexPoly(trapezoidal_mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0); 
  
  //src.copyTo(imageDest, mask); 
  return trapezoidal_mask;
}

cv::MatND Sidewalk_Filter::histogram_finder(const cv::Mat& mask, const cv::Mat& original, const bool is_onedimensional)
{
    cv::Mat src =  original;
    if(mask.empty()||original.empty())
      std::cerr << "Error : Could not receive stream" << std::endl;                                                              
    // Quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 10, sbins = 10;
    int histSize[] = {hbins, sbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 179 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 255 };
    const float* ranges[] = { hranges, sranges };
    cv::MatND hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1};
    calcHist( &src, 1, channels, mask, hist, 2, histSize, ranges, true, false );
    return hist;
}


//  HSI 1D Histogram finder - Didnt overload for convenience
std::vector<cv::Mat> Sidewalk_Filter::hsi_histogram_finder(const cv::Mat& mask, const cv::Mat& original, const bool is_onedimensional)
{
  cv::Mat src =  original;
    if(mask.empty()||original.empty())
      std::cerr << "Error : Could not receive stream hsi" << std::endl;  
  
  /// Separate the image in 3 places ( B, G and R )
  std::vector<cv::Mat> bgr_planes;
  std::vector<cv::Mat> hsi_hist;
  split( src, bgr_planes );

  /// Establish the number of bins
  int histSize = 10;

  /// Set the ranges ( for B,G,R) )
  float h_range[] = { 0, 360 };
  float i_range[] = { 0, 255 };
  const float* hRange = { h_range };
  const float* iRange = { i_range };

  bool uniform = true; bool accumulate = false;

  cv::Mat b_hist, g_hist, r_hist;
  /// Compute the histograms:
  calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &hRange, uniform, accumulate );
  calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &iRange, uniform, accumulate );
  hsi_hist.push_back(b_hist);
  hsi_hist.push_back(r_hist);
  return hsi_hist;
}

// HSV based back projection
cv::Mat Sidewalk_Filter::back_projection(const cv::MatND& src_hist, const cv::MatND& reference_histogram, const cv::Mat& background_image,const bool is_hsi)
{
  cv::Mat mask = cv::Mat::zeros(background_image.rows, background_image.cols, CV_8UC1); 
  float h_range[] = { 0, 179 };
  float s_range[] = { 0, 255 };
  const float* ranges[] = { h_range, s_range };
  int channels[] = { 0, 1 };
  normalize( src_hist, src_hist, 0, 1, 32, -1, cv::Mat());
  normalize( reference_histogram, reference_histogram, 0, 1, 32, -1, cv::Mat());
  int freq_h;
  int freq_s;
  float pixel_prob;
  
    for(int i = 0; i < background_image.rows; i++)
    {
      for(int j = 0; j < background_image.cols; j++)
        {
         freq_h = background_image.at<cv::Vec3b>(i,j)[0]/(179/10);
         freq_s = background_image.at<cv::Vec3b>(i,j)[1]/(255/10);
         //std::cout<< freq_h << std::endl;
          pixel_prob = reference_histogram.at<float>(freq_h,freq_s)/(reference_histogram.at<float>(freq_h,freq_s)+src_hist.at<float>(freq_h,freq_s));
         if (pixel_prob > 0.4 && i > 250)
         {
          mask.at<uchar>(cv::Point(j,i)) = 255;
         } 

      }
    } 
  return mask;
}

//  Back projection for HSI color space - Didnt oververload for convenience 
cv::Mat Sidewalk_Filter::hsi_back_projection(const std::vector<cv::Mat>& src_hist,const std::vector<cv::Mat>& reference_vector, const cv::Mat& background_image)
{
  cv::Mat mask = cv::Mat::zeros(background_image.rows, background_image.cols, CV_8UC1); 
  cv::Mat h_hist,i_hist;
  h_hist = reference_vector[0];
  i_hist = reference_vector[1];
  normalize( h_hist, h_hist, 0, 360, 32, -1, cv::Mat());
  normalize( i_hist, i_hist, 0, 255, 32, -1, cv::Mat());
  int freq_h;
  int freq_s;
  float pixel_prob;

    for(int i = 0; i < background_image.rows; i++)
    {
      for(int j = 0; j < background_image.cols; j++)
        {
         freq_h = background_image.at<cv::Vec3b>(i,j)[0]/(360/10);
         freq_s = background_image.at<cv::Vec3b>(i,j)[2]/(255/10);

         //std::cout << h_hist.at<float>(freq_h) << std::endl; 
         if(i_hist.at<float>(freq_s) < 200 )
         {
          mask.at<uchar>(cv::Point(j,i)) = 255;
         }
      }
    }
return mask; 
}

// converts the image as sample vector for EGBS
cv::Mat Sidewalk_Filter::asSamplesVectors(const cv::Mat& img ) {
    cv::Mat float_img;
    img.convertTo( float_img, CV_32F );
 
    cv::Mat samples( img.rows * img.cols, 3, CV_32FC1 );
 
    /* Flatten  */
    int index = 0;
    for( int y = 0; y < img.rows; y++ ) {
        cv::Vec3f* row = float_img.ptr<cv::Vec3f>(y);
        for( int x = 0; x < img.cols; x++ )
            samples.at<cv::Vec3f>(index++, 0) = row[x];
        }
        return samples;
    }

// Blob removal

IplImage* Sidewalk_Filter::remove_small_objects( IplImage* img_in, int size )
{
    IplImage* img_out       = cvCloneImage( img_in );  // return image
    CvMemStorage* storage   = cvCreateMemStorage( 0 );    // container of retrieved contours
    CvSeq* contours         = NULL;
    CvScalar black          = CV_RGB( 0, 0, 0 ); // black color
    CvScalar white          = CV_RGB( 255, 255, 255 );   // white color
    double area;

    // find contours in binary image
    cvFindContours( img_in, storage, &contours, sizeof( CvContour ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );  

    while( contours )   // loop over all the contours
    {
        area = cvContourArea( contours, CV_WHOLE_SEQ );
        if( fabs( area ) <= size )  // if the area of the contour is less than threshold remove it
        {
            // draws the contours into a new image
            cvDrawContours( img_out, contours, black, black, -1, CV_FILLED, 8 ); // removes white dots
        }
/*        else
        {
            cvDrawContours( img_out, contours, white, white, -1, CV_FILLED, 8 ); // fills in holes
        }*/
        contours = contours->h_next;    // jump to the next contour
    }

    cvReleaseMemStorage( &storage );
    return img_out;
}

