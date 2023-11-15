/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/hunter_master_controller/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hunter_master_controller
{
/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv)
{
}

QNode::~QNode()
{
  if (ros::isStarted())
  {
    ros::shutdown();  // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc, init_argv, "hunter_master_controller");
  if (!ros::master::check())
  {
    return false;
  }
  ros::start();  // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  rviz_path = ros::package::getPath("teb_local_planner_tutorials");
  rviz_path = rviz_path + "/cfg/rviz_navigation.rviz";

  // Add your ros communications here.
  image_transport::ImageTransport it(n);
  img_subscriber = it.subscribe("/camera/image", 1, &QNode::imgCallback, this);
  acuro_subscriber = it.subscribe("/result_img", 1, &QNode::acuroImgCallback, this);

  laser_subscriber = n.subscribe("/scan", 1, &QNode::laserCallback, this);
  hunter_subscriber = n.subscribe("/hunter_status", 1, &QNode::hunterCallback, this);

  start();
  return true;
}

void QNode::run()
{
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();  // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::imgCallback(const sensor_msgs::ImageConstPtr& img_raw)
{
  if (!imgIsRcvd)
  {
    imgIsRcvd = true;
    raw_img = new cv::Mat(cv_bridge::toCvCopy(img_raw, sensor_msgs::image_encodings::BGR8)->image);
    clone_img = raw_img->clone();
    cv::resize(clone_img, clone_img, cv::Size(RESIZE_H, RESIZE_W), 0, 0, CV_INTER_LINEAR);
    Q_EMIT updateImageSignal();
  }
}

void QNode::acuroImgCallback(const sensor_msgs::ImageConstPtr& img_raw)
{
  if (!imgIsRcvd)
  {
    acuroImgIsRcvd = true;
    acuro_raw_img = new cv::Mat(cv_bridge::toCvCopy(img_raw, sensor_msgs::image_encodings::BGR8)->image);
    acuro_img = raw_img->clone();
    cv::resize(acuro_img, acuro_img, cv::Size(RESIZE_H, RESIZE_W), 0, 0, CV_INTER_LINEAR);
    Q_EMIT updateAcuroImageSignal();
  }
}

void QNode::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  if (!laserIsRcvd)
  {
    laserIsRcvd = true;
    Q_EMIT updateLaserSignal();
  }
}

void QNode::hunterCallback(const hunter_msgs::HunterStatusConstPtr& msg)
{
  if (!hunterIsRcvd)
  {
    hunterIsRcvd = true;
    Q_EMIT updateHunterSignal();
  }
}

}  // namespace hunter_master_controller
