#include "Sidewalk_Filter"

Sidewalk_Filter::Sidewalk_Filter()
{

}

Sidewalk_Filter::~Sidewalk_Filter()
{
	
}

cv::Mat Sidewalk_Filter::rgb_to_hsi(const cv::Mat& src)
{
  if(src.empty())
    std::cerr << "Error: Loading image" << std::endl;
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