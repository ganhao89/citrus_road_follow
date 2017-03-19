#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
   
    // Applying Gaussian blur
    Mat img_blur;
    GaussianBlur( cv_ptr->image, img_blur, Size( 3, 3 ), 0, 0 );

    // Convert image to gray scale image
    Mat img_gray;  
    cvtColor(img_blur, img_gray, CV_BGR2GRAY );
    

    // Convert image Green-ness image
    Mat img_green;
    Mat ch1,ch2,ch3;
    vector<Mat> channels(3);
    split(img_blur, channels);
    // get the channels (dont forget they follow BGR order in OpenCV)
    ch1 = channels[0]/(channels[0]+channels[1]+channels[2])*255;// 0 is blue
    ch2 = channels[1]/(channels[0]+channels[1]+channels[2])*255;// 1 is green
    ch3 = channels[2]/(channels[0]+channels[1]+channels[2])*255;// 2 is red
    img_green = 2*ch2-ch3-ch1;

    // Crop the upper half of the image 
    Mat img_crop;
    Rect roi;
    roi.x = 0;
    roi.y = 0;
    roi.width = img_green.size().width;
    roi.height = (int) img_green.size().height/2;
    img_crop = img_green(roi);

    Mat img_blur2;
    GaussianBlur(img_green, img_blur2, Size( 5, 5 ), 0, 0 );
    // Thrsholding based on green-ness
    Mat img_green_thresh;
    threshold(img_blur2, img_green_thresh, 20, 255, 0);
    Mat img_green_thresh_inv;
    bitwise_not(img_green_thresh, img_green_thresh_inv);
    
    // Thresholding based on brightness
    Mat img_blur3;
    GaussianBlur(img_gray, img_blur3, Size( 5, 5 ), 0, 0 );
    Mat img_bright_thresh;
    threshold(img_gray, img_bright_thresh, 150, 255,0);

    Mat img_thresh;
    bitwise_and(img_green_thresh_inv, img_bright_thresh, img_thresh);
    
    Mat img_thresh_inv;
    bitwise_not(img_thresh, img_thresh_inv);
    vector<vector<Point> > contours;
    findContours(img_thresh_inv, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    drawContours(img_thresh_inv, contours, -1, Scalar(255), CV_FILLED);
    bitwise_not(img_thresh_inv, img_thresh);
    // Convert the image to Canny edge image
    Mat img_edge;
    Canny(img_thresh,img_edge,50,350);


    // Apply Probabilistic Hough Line Transform
    vector<Vec4i> lines;
    HoughLinesP(img_edge, lines, 4, CV_PI/180, 100, 0, 0);
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      float theta = atan2(abs(l[3]-l[1]),abs(l[2]-l[0]))*180.0/CV_PI;
      if((theta<80 && theta>40) || (theta>100 && theta<140))
      {
        line(cv_ptr->image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
      }
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
