// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "br2_tracking/MultipleObjectDetector.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

namespace br2_tracking
{

using std::placeholders::_1;

MultipleObjectDetector::MultipleObjectDetector()
: Node("object_detector")
{
  image_sub_ = image_transport::create_subscription(
    this, "input_image", std::bind(&MultipleObjectDetector::image_callback, this, _1),
    "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile());

  detection_pub_ = create_publisher<vision_msgs::msg::Detection2D>("detection", 100);

  declare_parameter("hsv_ranges", hsv_filter_ranges_);
  declare_parameter("debug", debug_);

  get_parameter("hsv_ranges", hsv_filter_ranges_);
  get_parameter("debug", debug_);
}

void
MultipleObjectDetector::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  //if (detection_pub_->get_subscription_count() == 0) {
  //  RCLCPP_INFO(get_logger(), "No subscription found");
  //  return;
  //  }

  const float & h = hsv_filter_ranges_[0];
  const float & H = hsv_filter_ranges_[1];
  const float & s = hsv_filter_ranges_[2];
  const float & S = hsv_filter_ranges_[3];
  const float & v = hsv_filter_ranges_[4];
  const float & V = hsv_filter_ranges_[5];

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat img_hsv;
  cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);

  cv::Mat1b filtered;
  cv::inRange(img_hsv, cv::Scalar(h, s, v), cv::Scalar(H, S, V), filtered);

  cv::Mat canny_output;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;

  // closing contours the filtered image
  cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(30, 30));
  cv::morphologyEx( filtered, filtered, cv::MORPH_CLOSE, structuringElement );
  
  // detect edges using canny
  cv::Canny( filtered, canny_output, 50, 300, 3 );
  
  // find contours
  cv::findContours( canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  
  // get the moments
  std::vector<cv::Moments> mu(contours.size());
  for( int i = 0; i<contours.size(); i++ )
  { mu[i] = cv::moments( contours[i], false ); }
  
  // get the centroid of figures.
  std::vector<cv::Point2f> mc(contours.size());
  for( int i = 0; i<contours.size(); i++)
  { mc[i] = cv::Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }
  
  //vision_msgs::msg::Detection2D detection_msg;
  //detection_msg.header = msg->header;
  //detection_msg.bbox.size_x = bbx.width;
  //detection_msg.bbox.size_y = bbx.height;
  //detection_msg.bbox.center.position.x = cx;
  //detection_msg.bbox.center.position.y = cy;
  //detection_pub_->publish(detection_msg);

  if (debug_) {
    // draw contours
    cv::Mat drawing(canny_output.size(), CV_8UC3, cv::Scalar(255,255,255));
    for( int i = 0; i<contours.size(); i++ )
    {
      cv::Rect bbx = cv::boundingRect(contours[i]);
      int cx = bbx.x + bbx.width/2;
      int cy = bbx.y + bbx.height/2;
      
      cv::Scalar red = cv::Scalar(0, 0, 255); // B G R values
      cv::Scalar blue = cv::Scalar(255, 0, 0); // B G R values
      cv::rectangle(cv_ptr->image, bbx, cv::Scalar(0, 0, 255), 3);
      cv::circle(cv_ptr->image, cv::Point(cx, cy), 3, cv::Scalar(255, 0, 0), 3);
    }
    
    cv::imshow("cv_ptr->image", cv_ptr->image);
    cv::waitKey(1);
  }
}

}  // namespace br2_tracking
