// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/capture.h"
#include <sstream>
#include <string>

// #include <iostream>
// #include <fstream>
// #include <libusb-1.0/libusb.h>

// #include <cstdio>
// #include <memory>
// #include <stdexcept>
// #include <array>

#include <regex>

namespace cv_camera
{

namespace enc = sensor_msgs::image_encodings;

Capture::Capture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id,
                 const std::string& camera_name)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id),
      info_manager_(node_, camera_name),
      capture_delay_(ros::Duration(node_.param("capture_delay", 0.0))),
      publish_viz_(node_.param("pub_vizualization", true))
{
}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }

  rescale_camera_info_ = node_.param<bool>("rescale_camera_info", false);

  for (int i = 0;; ++i)
  {
    int code = 0;
    double value = 0.0;
    std::stringstream stream;
    stream << "property_" << i << "_code";
    const std::string param_for_code = stream.str();
    stream.str("");
    stream << "property_" << i << "_value";
    const std::string param_for_value = stream.str();
    if (!node_.getParam(param_for_code, code) || !node_.getParam(param_for_value, value))
    {
      break;
    }
    if (!cap_.set(code, value))
    {
      ROS_ERROR_STREAM("Setting with code " << code << " and value " << value << " failed"
                                            << std::endl);
    }
  }
}

void Capture::rescaleCameraInfo(int width, int height)
{
  double width_coeff = static_cast<double>(width) / info_.width;
  double height_coeff = static_cast<double>(height) / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.K[0] *= width_coeff;
  info_.K[2] *= width_coeff;
  info_.K[4] *= height_coeff;
  info_.K[5] *= height_coeff;

  info_.P[0] *= width_coeff;
  info_.P[2] *= width_coeff;
  info_.P[5] *= height_coeff;
  info_.P[6] *= height_coeff;
}

void Capture::open(int32_t device_id)
{
  cap_.open(device_id,cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "device_id" << device_id << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  if (publish_viz_){pub_viz_ = it_.advertiseCamera(topic_name_+"_viz", buffer_size_);}

  loadCameraInfo();
}

void Capture::open(const std::string &device_path)
{
  // Search USB devices for matching serial number using udevadm output
  std::array<char, 128> buffer;
  std::string result;
  int device_open_id;

  for(int i = 0; i < 100; i++){
  // const char* is = std::to_string(i);
  std::string cmdstr = std::string("udevadm info --name=/dev/video") + std::to_string(i);
  const char* cmd = cmdstr.c_str();
  // const char* cmd = ("udevadm info --name=/dev/video" + std::to_string(i));
  std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe) throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get())) {
      if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
          result += buffer.data();
  }
  bool bus_dev_found = false;
  std::string current_line, device_num, device_open_path;
  std::istringstream lsusboutut{result};

  while (std::getline(lsusboutut, current_line))
  {
    std::smatch m; 
    if(std::regex_search(current_line, m, std::regex("N: video(\\d+)"))){
      device_num = m[1].str();
      printf("%s\n", device_num.c_str());
    }
    std::smatch mm; 
    if(std::regex_search(current_line, mm, std::regex(device_path.c_str()))){
      device_open_id = std::stoi(device_num);
      printf("Found device with matching serial number: /dev/video%s, %s\n", device_num.c_str(), device_path.c_str());
      break;
    }
     
  } 
  // TODO: this doesn't check if multiple devices match the serial, just uses the last
  }

  // using lsusb output
  // std::array<char, 128> buffer;
  // std::string result;
  // const char* cmd = "lsusb -v -d 1e4e:0100";
  // std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  // if (!pipe) throw std::runtime_error("popen() failed!");
  // while (!feof(pipe.get())) {
  //     if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
  //         result += buffer.data();
  // }
  // // printf(result.c_str());
  // // search for iSerial
  // bool bus_dev_found = false;
  // std::string current_line, bus, dev, device_open_path;
  // // std::ifstream lsusboutut (result.c_str());
  // std::istringstream lsusboutut{result};

  // printf(device_path.c_str());

  // while (std::getline(lsusboutut, current_line))
  // {
  //   std::smatch m; 
  //   if(std::regex_search(current_line, m, std::regex("Bus (\\d+) Device (\\d+): ID 1e4e:0100 Cubeternet WebCam"))){
  //     bus = m[1].str();
  //     dev = m[2].str();
  //     printf(bus.c_str());
  //     printf("   ");
  //     printf(dev.c_str());
  //     printf("\n");
  //   }
  //   std::smatch mm; 
  //   if(std::regex_search(current_line, mm, std::regex(device_path.c_str()))){
  //     printf("FOUND DEVICE PATH MATCH:\n");
  //     device_open_path = ("/dev/bus/usb/"+ bus+"/"+ dev);
  //     printf(device_open_path.c_str());
  //     printf("\n");
  //     break;
  //   }
     
  // }


  



  ////////////////////////////////////////// libusb way, needs to open devices to read sn???
  // libusb_context *context = NULL;
  // libusb_device **list = NULL;
  // int rc = 0;
  // int rcs = 0;
  // ssize_t count = 0;

  // rc = libusb_init(&context);
  // assert(rc == 0);

  // count = libusb_get_device_list(context, &list);
  // assert(count > 0);

  // for (size_t idx = 0; idx < count; ++idx) {
  //     libusb_device *device = list[idx];
  //     libusb_device_descriptor desc = {0};
  //     libusb_device_handle *handle;
  //     int status = 0;
  //     // libusb_string_descriptor stdesc = {0};

  //     rc = libusb_get_device_descriptor(device, &desc);
  //     assert(rc == 0);

  //     status = libusb_open(device, &handle);
  //     if (status < 0) {
  //       // usbi_err("could not open device, error %d", status);
  //       printf("err");
  //       free(handle);
  //       // return NULL;
  //       continue;
  //     }


  //     unsigned char *data; 
  //     int length;

  //     rcs = libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, data, length);
  //     // assert(rcs == 0);

  //     printf("Vendor:Device = %04x:%04x\n", desc.idVendor, desc.idProduct);
  //     printf("iSerialNumber = %x\n", desc.iSerialNumber);
  //     // printf("Serial = %s\n", data);


  //     // printf("Serial = %s\n", device->getString(desc.iSerialNumber));
  //     // libusb_get_string_descriptor_ascii(device, desc.iSerialNumber);
  //     // printf("Serial = %s\n", stdesc));
  // }
  ////////////////////////////////////////////////////////////////
  
  // struct usb_bus *bus;
  // struct usb_device *dev;
  // usb_init();
  // usb_find_busses();
  // usb_find_devices();
  // for (bus = usb_busses; bus; bus = bus->next)
  //     for (dev = bus->devices; dev; dev = dev->next){
  //         printf("Trying device %s/%s\n", bus->dirname, dev->filename);
  //         printf("\tID_VENDOR = 0x%04x\n", dev->descriptor.idVendor);
  //         printf("\tID_PRODUCT = 0x%04x\n", dev->descriptor.idProduct);
  //         printf("\tID_SERIAL = %s\n", dev->descriptor.iSerial);
  //     }





  // try
  //   {
  //       std::ifstream file_input;
  //       std::size_t pos;
  //       std::string device_path, current_line, search_str, event_str;
  //       std::string device_list_file = "/proc/bus/input/devices";
  //       bool vid_pid_found = false;
  //       int fd = 0;
  //       bool debug = true;

  //       // 1. open device list file
  //       file_input.open(device_list_file.c_str());
  //       if (!file_input.is_open())
  //       {
  //           std::cerr << "file_input.open >> " << std::strerror(errno) << std::endl;
  //           throw -2;
  //       }

  //       // 2. search for first VID:PID and get event number
  //       search_str = "Vendor=" + device_vid + " Product=" + device_pid;
  //       while (getline(file_input, current_line))
  //       {
  //           if (!vid_pid_found)
  //           {
  //               pos = current_line.find(search_str, 0);
  //               if (pos != std::string::npos)
  //               {
  //                   vid_pid_found = true;
  //                   search_str = "event";
  //               }               
  //           }
  //           else
  //           {
  //               pos = current_line.find(search_str, 0);
  //               if (pos != std::string::npos)
  //               {
  //                   event_str = current_line.substr(pos);
  //                   // find space and substring event##
  //                   pos = event_str.find(' ', 0);
  //                   event_str = event_str.substr(0, pos);
  //                   break;
  //               }
  //           }
  //       }

  //       // 3.  build device path
  //       device_path = "/dev/input/" + event_str;
  //       if (debug) std::cout << "device_path = " << device_path << std::endl;   

  //       // 4.  connect to device
  //       fd = open (device_path.c_str(), O_RDONLY);
  //       if (fd < 0)
  //       {
  //           std::cerr << "open >> errno = " << std::strerror(errno) << std::endl;       
  //           throw -3;
  //       }
  //   }
  //   catch (const std::exception &e)
  //   {
  //       std::cerr << "e.what() = " << e.what() << std::endl;
  //       throw -1;
  //   }














  // TODO: open lepton by serial number or VID/PID for persistent naming
  // struct usb_bus *bus;
  // struct usb_device *dev;
  // libusb_open_device_with_vid_pid();
  // libusb_get_device_address();
  // libusb_get_device_list();
  
  
  // usb_init();
  // usb_find_busses();
  // usb_find_devices();
  // for (bus = usb_busses; bus; bus = bus->next)
  //     for (dev = bus->devices; dev; dev = dev->next){
  //         printf("Trying device %s/%s\n", bus->dirname, dev->filename);
  //         printf("\tID_VENDOR = 0x%04x\n", dev->descriptor.idVendor);
  //         printf("\tID_PRODUCT = 0x%04x\n", dev->descriptor.idProduct);
  //     }

  cap_.open(device_open_id, cv::CAP_V4L2);
  if (!cap_.isOpened())
  {
    throw DeviceError("device_path cannot be opened");
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  if (publish_viz_){pub_viz_ = it_.advertiseCamera(topic_name_+"_viz", buffer_size_);}

  loadCameraInfo();
}

// void Capture::open(const std::string &device_path)
// {
//   cap_.open(device_path, cv::CAP_V4L2);
//   if (!cap_.isOpened())
//   {
//     throw DeviceError("device_path " + device_path + " cannot be opened");
//   }
//   pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
//   if (publish_viz_){pub_viz_ = it_.advertiseCamera(topic_name_+"_viz", buffer_size_);}

//   loadCameraInfo();
// }

void Capture::open()
{
  open(0);
}

void Capture::openFile(const std::string &file_path)
{
  cap_.open(file_path);
  if (!cap_.isOpened())
  {
    std::stringstream stream;
    stream << "file " << file_path << " cannot be opened";
    throw DeviceError(stream.str());
  }
  pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
  if (publish_viz_){pub_viz_ = it_.advertiseCamera(topic_name_+"_viz", buffer_size_);}

  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }
}

bool Capture::capture()
{
  if (cap_.read(bridge_.image))
  {
    // Trim off lepton metadata
    int cropheight = bridge_.image.size().height - 2;
    int cropwidth = bridge_.image.size().width;
    bridge_.image = bridge_.image(cv::Range(0,cropheight),cv::Range(0,cropwidth));

    // cv::imwrite("/home/jch/Pictures/testim.tif", bridge_.image);

    // Construct header
    ros::Time stamp = ros::Time::now() - capture_delay_;
    // bridge_.encoding = bridge_.image.channels() == 3 ? enc::BGR8 : enc::MONO8;
    bridge_.encoding = enc::MONO16;
    bridge_.header.stamp = stamp;
    bridge_.header.frame_id = frame_id_;

    info_ = info_manager_.getCameraInfo();
    if (info_.height == 0 && info_.width == 0)
    {
      info_.height = bridge_.image.rows;
      info_.width = bridge_.image.cols;
    }
    else if (info_.height != bridge_.image.rows || info_.width != bridge_.image.cols)
    {
      if (rescale_camera_info_)
      {
        int old_width = info_.width;
        int old_height = info_.height;
        rescaleCameraInfo(bridge_.image.cols, bridge_.image.rows);
        ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                      old_width, old_height, bridge_.image.cols, bridge_.image.rows);
      }
      else
      {
        ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                      "Use rescale_camera_info param for rescaling",
                      info_.width, info_.height, bridge_.image.cols, bridge_.image.rows);
      }
    }
    info_.header.stamp = stamp;
    info_.header.frame_id = frame_id_;
    
    if (publish_viz_)
    {
      // Make a pretty image
      cv_bridge::CvImage tmp_;
      cv::normalize(bridge_.image, tmp_.image, 0, 65535, cv::NORM_MINMAX);
      tmp_.image.convertTo(tmp_.image,CV_8UC1,1/255.0); //CV_32F
      cv::applyColorMap(tmp_.image, bridge_viz_.image, cv::COLORMAP_HOT);

      bridge_viz_.encoding = enc::BGR8;
      bridge_viz_.header.stamp = stamp;
      bridge_viz_.header.frame_id = frame_id_;
      
      info_viz_ = info_;

      // Publish temperature at picked point
      int16_t temp_mk = bridge_.image.at<int16_t>(pty,ptx);
      sensor_msgs::Temperature tempmsg;
      tempmsg.temperature = static_cast< float >( temp_mk ) / 100 - 271.15;
      tempmsg.header.stamp = stamp;
      pointtemp.publish(tempmsg);

      // Draw crosshair
      // cv::line(bridge_viz_.image, cv::Point(ptx - 2, pty), cv::Point(ptx + 2, pty), (0,255,0), 1);
      // cv::line(bridge_viz_.image, cv::Point(ptx, pty - 2), cv::Point(ptx, pty + 2), (0,255,0), 1);
      cv::line(bridge_viz_.image, cv::Point(ptx - 5, pty), cv::Point(ptx - 2, pty), cvScalar(117,79,142), 1);
      cv::line(bridge_viz_.image, cv::Point(ptx + 5, pty), cv::Point(ptx + 2, pty), cvScalar(117,79,142), 1);
      cv::line(bridge_viz_.image, cv::Point(ptx, pty - 5), cv::Point(ptx, pty - 2), cvScalar(117,79,142), 1);
      cv::line(bridge_viz_.image, cv::Point(ptx, pty + 5), cv::Point(ptx, pty + 2), cvScalar(117,79,142), 1);

    }
    ros::spinOnce();

    return true;
  }
  return false;
}

void Capture::publish()
{
  pub_.publish(*getImageMsgPtr(), info_);
  if(publish_viz_){
    pub_viz_.publish(*getImageVizMsgPtr(), info_viz_);
  }
}

bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
{
  if (cap_.isOpened())
  {
    double value = 0.0;
    if (node_.getParam(param_name, value))
    {
      ROS_INFO("setting property %s = %lf", param_name.c_str(), value);
      return cap_.set(property_id, value);
    }
  }
  return true;
}

bool Capture::setY16()
{
  if (cap_.isOpened())
  {
    double yval = cv::VideoWriter::fourcc('Y','1','6',' ');
    ROS_INFO("setting property Y16 = %lf", yval);
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y','1','6',' '));
  }
  return true;
}

void Capture::vizClickCallback(const geometry_msgs::Point& pt)
{
  ROS_INFO("Monitoring point temperature at: (%f,%f)",pt.x,pt.y);
  ptx = static_cast<int>(pt.x);
  pty = static_cast<int>(pt.y);
}

} // namespace cv_camera
