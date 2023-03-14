

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  // ros::NodeHandle private_node_("~");
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat normalized = cv::Mat(480, 640, CV_16UC1, 1);
//   cv::Mat imageEq = cv::Mat(480, 640, CV_8UC1, 1);
cv::Mat image32;
cv::Mat imageEq;
// XXX doesnt work yet not getting param
public:
  ImageConverter() : it_(nh_)
  {
    std::string image_topic_;
    if (!nh_.getParam("image_topic", image_topic_))
    {
      image_topic_ = "cv_camera/image_raw";
    }
    ROS_INFO("XXXXXXXXXXXXXXXXXXx");
    ROS_INFO(image_topic_.c_str());
    std::string viz_topic_ = image_topic_ + "_viz";

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe(image_topic_, 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise(viz_topic_, 1);

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_out;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::normalize(cv_ptr->image, cv_ptr->image, 0, 65535, cv::NORM_MINMAX);
    cv_ptr->image.convertTo(cv_ptr->image,CV_8UC1,1/255.0); //CV_32F
    // cv::cvtColor(cv_ptr->image,cv_ptr_out->image,CV_16UC1);
    // cv::equalizeHist(cv_ptr->image,imageEq);

    // cv::invert(cv_ptr->image,cv_ptr->image);
    cv::bitwise_not(cv_ptr->image,cv_ptr->image);
    
    // cv_ptr_out->header = cv_ptr->header;
    cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;


    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ir_visualization");
  ImageConverter ic;
  ros::spin();
  return 0;
}