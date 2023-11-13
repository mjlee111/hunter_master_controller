/**
 * @file /include/hunter_master_controller/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef hunter_master_controller_QNODE_HPP_
#define hunter_master_controller_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include <ros/package.h>
#include "sensor_msgs/LaserScan.h"
#include "hunter_msgs/HunterStatus.h"

#define RESIZE_H 640
#define RESIZE_W 480

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hunter_master_controller
{
/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  bool imgIsRcvd = false;
  bool laserIsRcvd = false;
  bool hunterIsRcvd = false;
  cv::Mat* raw_img;
  cv::Mat clone_img;

  std::string rviz_path;

Q_SIGNALS:
  void rosShutdown();
  void updateImageSignal();
  void updateLaserSignal();
  void updateHunterSignal();

private:
  int init_argc;
  char** init_argv;

  image_transport::Subscriber img_subscriber;
  void imgCallback(const sensor_msgs::ImageConstPtr& img_raw);

  ros::Subscriber laser_subscriber;
  void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

  ros::Subscriber hunter_subscriber;
  void hunterCallback(const hunter_msgs::HunterStatusConstPtr& msg);
};

}  // namespace hunter_master_controller

#endif /* hunter_master_controller_QNODE_HPP_ */
