#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "std_msgs/Float64.h"
#include "img_bucket.h"
#include "std_msgs/Header.h"

img_bucket::img_bucket()
{
  cam_pub = NULL;
  i_current = 0;
  fake_camera_info.height = 1024;
  fake_camera_info.width = 1280;
  fake_camera_info.distortion_model = "plumb_bob";
  fake_camera_info.D.resize(6);
  for (int i = 0; i < 5; i++)
  {
    fake_camera_info.D[i] = dist[i];
  }
  for (int i = 0; i < 9; i++)
  {
    fake_camera_info.K[i] = camera[i];
    fake_camera_info.R[i] = rect[i];
  }
  for (int i = 0; i < 12; i++)
  {
    fake_camera_info.P[i] = proj[i];
  }
}

void img_bucket::load_images()
{
  std::string location, filter;
  ros::param::param<std::string>("bucket_location", location, default_location);
  ros::param::param<std::string>("extension_filter", filter, default_filter);
  ROS_INFO("Looking for images (%s) in folder:  %s", filter.c_str(), location.c_str());
  cv::glob(location + filter, file_names, false);
  for (auto it = file_names.begin(); it != file_names.end(); it++)
  {
    cv::Mat src = cv::imread(*it);
    if (!src.data)
      ROS_WARN("Could not read %s", *it);
    else
      file_data.push_back(cv::imread(*it));
  }
}

bool img_bucket::publish_next()
{
  if (i_current == file_names.size())
    return false;
   std_msgs::Header header;
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", file_data[i_current]).toImageMsg();
  sensor_msgs::CameraInfoPtr cam_msg = boost::shared_ptr<sensor_msgs::CameraInfo>(new sensor_msgs::CameraInfo(fake_camera_info));
  cam_msg->header.stamp = header.stamp;
  cam_msg->header.frame_id = file_names[i_current];
  ROS_INFO_STREAM(file_names[i_current]);
  cam_pub->publish(msg, cam_msg);
  i_current++;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_bucket");
  ros::NodeHandle nh("img_bucket");
  img_bucket img_str;
  img_str.load_images();
  image_transport::ImageTransport imgTrs(nh);
  image_transport::CameraPublisher cam_pub;
  cam_pub = imgTrs.advertiseCamera("image_raw", 1);
  img_str.set_cam_pub(&cam_pub);
  ros::Rate rate(1);
  while (ros::ok())
  {
    if (!img_str.publish_next())
      break;
    ros::spinOnce();
    rate.sleep();
  }
}
