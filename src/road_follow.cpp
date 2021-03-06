#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseAction.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
int image_counter=0;
vector<Point2i> cent_point_pre;
float x_devi_old = 0;
float alpha_old = 0;
double dt = 0;
float vy_filtered;
float robot_yaw_filtered;
//float alpha_filtered = 0;
//float x_devi_filtered =0;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber odom_filtered_sub_;
  ros::Time current_time, last_time;
  tf::TransformBroadcaster odom_broadcaster;
  

public:
  ImageConverter()
    : it_(nh_)
  {
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/kinect2/qhd/image_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("vis_odom", 1);
    goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseGoal>("way_points",1);
    odom_filtered_sub_ = nh_.subscribe("/odometry/filtered/local", 1, &ImageConverter::odomCallback, this);
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
    GaussianBlur(cv_ptr->image, img_blur, Size( 3, 3 ), 0, 0 );
    
    // Get the dimention of the image
    int img_width = cv_ptr->image.size().width;
    int img_height = cv_ptr->image.size().height;

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

    // Crop the upper half of the image 
    Mat img_crop;
    Rect roi;
    roi.x = 0;
    roi.y = 0;
    roi.width = img_edge.size().width;
    roi.height = (int) (img_edge.size().height/1.9);
    img_crop = img_edge(roi);

    // Apply Probabilistic Hough Line Transform
    vector<Vec4i> lines;
    HoughLinesP(img_crop, lines, 4, CV_PI/180, 100, 0, 0);
    Vec4i l0= lines[0];
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      int theta = atan2(abs(l[3]-l[1]),abs(l[2]-l[0]))*180.0/CV_PI;
      if((theta<80 && theta>40))
      {
        if (l[1]>l0[1])
	{
	  l0 = l;
	}
        line(cv_ptr->image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1, CV_AA);
      }  
    }
    //line(cv_ptr->image, Point(l0[0], l0[1]), Point(l0[2], l0[3]), Scalar(255,0,0), 3, CV_AA);

    // Divide the image into left and right based on point l0
    // Store start/end points of lines in the left image to lines_left, and start/end points of lines in the right image to lines_right
    vector<Point2i> lines_left;
    vector<Point2i> lines_right;
    for( size_t i = 0; i < lines.size(); i++ )
    {
      Vec4i l = lines[i];
      int theta = atan2(abs(l[3]-l[1]),abs(l[2]-l[0]))*180.0/CV_PI;
      if(l[0]<=l0[0] && l[2]<=l0[0] && 40<theta && theta<80 && (l[3]-l[1])*(l[2]-l[0])>0)
      {
        lines_left.push_back(Point(l[0],l[1]));
	lines_left.push_back(Point(l[2],l[3]));
      } else if (l[0]>=l0[0] && l[2]>=l0[0] && 40<theta && theta<80 && (l[3]-l[1])*(l[2]-l[0])<0){
	lines_right.push_back(Point(l[0],l[1]));
	lines_right.push_back(Point(l[2],l[3]));
      }
    }
    // Fit two lines to the left points and right points seperately
    Vec4f line_left;
    Vec4f line_right;
    if (lines_left.size() >1)
    {
      fitLine(lines_left, line_left, CV_DIST_L2,0,0.01,0.01);
      line(cv_ptr->image, Point(line_left[2],line_left[3]), Point(line_left[2]+line_left[0]*200,line_left[3]+line_left[1]*200),Scalar(0,255,255), 2, CV_AA);
    }
    if (lines_right.size() >1)
    {
      fitLine(lines_right, line_right, CV_DIST_L2,0,0.01,0.01);
      line(cv_ptr->image, Point(line_right[2],line_right[3]), Point(line_right[2]+line_right[0]*200,line_right[3]+line_right[1]*200),Scalar(0,255,255), 2, CV_AA);
    }
    // Calculate the intersection of the two line.
    Point2i cent_point;
    float slope_left = line_left[1]/line_left[0];
    float slope_right = line_right[1]/line_right[0];
    cent_point.x = ((line_right[3]-line_left[3])+(slope_left*line_left[2] - slope_right*line_right[2]))/(slope_left-slope_right);
    cent_point.y = slope_left*cent_point.x+(line_left[3]-slope_left*line_left[2]);
    if (cent_point.y-img_height/2<-25 || cent_point.x<0 || cent_point.x>img_width)
    {
      if (abs(slope_left)>abs(slope_right))
      {
	cent_point.y = img_height/2;
        cent_point.x = (cent_point.y-line_left[3]+slope_left*line_left[2])/slope_left;
      } else{
	cent_point.y = img_height/2;
	cent_point.x = (cent_point.y-line_right[3]+slope_right*line_right[2])/slope_right;
      }
    }

    // Using median filter to minimize the jumps of the cent_point
    int window_length = 10;
    if (image_counter<window_length)
    {
      cent_point_pre.push_back(cent_point);
      image_counter++;
    } else{
      float cent_point_median_x[10];
//      float cent_point_median_y[10];
      for (size_t i=0;i<window_length-1;i++)
      {
	cent_point_pre[i].x = cent_point_pre[i+1].x;
	cent_point_pre[i].y = cent_point_pre[i+1].y;
        cent_point_median_x[i] = cent_point_pre[i].x;
//	cent_point_median_y[i] = cent_point_pre[i].y;
      }
      cent_point_pre[window_length-1].x = cent_point.x;
      cent_point_pre[window_length-1].y = cent_point.y;
      cent_point_median_x[9] = cent_point.x;
      //cent_point_median_y[9] = cent_point.y;
      cent_point.x = wirth_median(cent_point_median_x, 10);
//      cent_point.y = wirth_median(cent_point_median_y, 10);
    } 
  
    //circle(cv_ptr->image, cent_point, 5, CV_RGB(255,0,0), 5,8,0);

    // Calculate the center line
    Point2i cent_point2;
    cent_point2.y = 0;
    cent_point2.x = 0.5*(0-cent_point.y)*(1/slope_left+1/slope_right)+cent_point.x;
    //line(cv_ptr->image, cent_point, cent_point2,Scalar(0,120,255), 2, CV_AA);

    // Dynamically calculate visual odometry based on the center line
    /*
    The relationships between x_t, x_c and 
    robot's real world deviation from center line x_devi and 
    angle with center line alpha are described as the following
   
    x_c = img_width/2-img_focal*tan(alpha);
    x_t = img_width/2-x_devi*img_height/(tree_height*cos(alpha));
    img_focal = img_width/(2*tan(fov_h/2));
    alpha < fov_h/2;
    arctan(x_devi/(tree_height*cos(alpha))) < fov_v/2;
    */
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec();
    int x_c = cent_point.x;
    int x_t = cent_point2.x;
    float fov_h = 90;
    float fov_v = 72;
    float img_focal = img_width/(2*tan(fov_h/2*M_PI/180));
    float tree_height = 3.7;
    // alpha: the angle between the robot's heading and the center line of the road
    // x_devi: the distance between the center of the robot and the center line of the road
    float alpha = atan((img_width/2-x_c)/img_focal);
    float x_devi = (img_width/2-x_t)*tree_height*cos(alpha)/img_height;
    float robot_yaw = (alpha - alpha_old)*180/M_PI/dt;
    float vy = (x_devi-x_devi_old)/dt;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
    
    /*
    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    */    

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    

    //set the position
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = robot_yaw;
    
    odom_pub_.publish(odom);
    
    //receive updated odom info from ekf
    float alpha_filtered = robot_yaw_filtered*dt*M_PI/180+alpha_old;
    float x_devi_filtered = vy_filtered*dt+x_devi_old;
    
    float alpha_new = 0.05*alpha+0.95*alpha_filtered;
    float x_devi_new = 0.05*x_devi+0.95*x_devi_filtered;
    if (x_c<0 || x_c>img_width || x_t<0 || x_t>img_width){
        alpha_new = alpha_filtered;
        x_devi_new = x_devi_filtered;
    }
    cent_point.x = img_width/2-img_focal*tan(alpha_new);
    cent_point2.x = img_width/2-x_devi_new*img_height/(tree_height*cos(alpha_new));
    


    circle(cv_ptr->image, cent_point, 5, CV_RGB(255,0,0), 5,8,0);
    line(cv_ptr->image, cent_point, cent_point2,Scalar(0,120,255), 2, CV_AA);

    last_time = current_time;
    x_devi_old = x_devi_new;
    alpha_old = alpha_new;
    
    move_base_msgs::MoveBaseGoal goal;
    
    float far_end = 20;
    float tri_1 = sqrt(x_devi_new*x_devi_new+far_end*far_end);
    float beta = atan(x_devi_new/far_end);
    float theta_tri = beta+alpha_new; 
    float goal_y = tri_1*sin(theta_tri);
    float goal_x = tri_1*cos(theta_tri);
    //we'll send a goal to the robot
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.orientation.w = alpha_new*180/M_PI;
    
    goal_pub_.publish(goal);

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
  
  float wirth_median(float a[], uint16_t n)
  {   
    int k = (((n)&1)?((n)/2):(((n)/2)-1));
    uint64_t i,j,l,m ;
    float x,t ;
    l=0 ; m=n-1 ;
    while (l<m) {
      x=a[k] ;
      i=l ;
      j=m ;
      do {
        while (a[i]<x) i++ ;
        while (x<a[j]) j-- ;
        if (i<=j) {
          t=a[i];
          a[i]=a[j];
          a[j]=t;
          i++ ; j-- ;
        }
      } while (i<=j) ;
      if (j<k) l=i ;
      if (k<i) m=j ;
    }
    return a[k] ;
  }
  
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    vy_filtered = msg->twist.twist.linear.y;
    robot_yaw_filtered = msg->twist.twist.angular.z;
    //std::cout<<"odomCallback"<<std::endl;
  }
 
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
