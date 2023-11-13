/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/hunter_master_controller/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace hunter_master_controller
{
using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), qnode(argc, argv)
{
  ui.setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  setWindowIcon(QIcon(":/images/icon.png"));

  qnode.init();

  _1s_timer = new QTimer(this);

  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(updateImageSignal()), this, SLOT(updateImageSlot()));
  QObject::connect(&qnode, SIGNAL(updateLaserSignal()), this, SLOT(updateLidarStatSlot()));
  QObject::connect(&qnode, SIGNAL(updateHunterSignal()), this, SLOT(updateHunterStatSlot()));

  connect(_1s_timer, SIGNAL(timeout()), SLOT(fpsUpdate()));
  _1s_timer->start(1000);
}

MainWindow::~MainWindow()
{
}

/*****************************************************************************
** Functions
*****************************************************************************/

/*****************************************************************************
** SLOTS
*****************************************************************************/

void MainWindow::on_hunter_btn_clicked()
{
  ROS_INFO("Now Running Hunter Bringup!");
  system("roslaunch hunter_bringup hunter_robot_base.launch &");
}

void MainWindow::on_cam_lidar_btn_clicked()
{
  ROS_INFO("Now Running Cam & Lidar Bringup!");
  system("roslaunch baram_bringup launch_camera.launch &");
  system("roslaunch baram_bringup launch_lidar.launch &");
}

void MainWindow::on_nav_btn_clicked()
{
  ROS_INFO("Now Running Baram Navigation Bringup!");
}

void MainWindow::on_aruco_btn_clicked()
{
  ROS_INFO("Now Running aruco detector Bringup!");
}

void MainWindow::updateImageSlot()
{
  ui.cam_stat->setText("OK!");
  ui.cam_stat->setStyleSheet("color: green;");
  QImage qraw_img((const unsigned char*)(qnode.clone_img.data), qnode.clone_img.cols, qnode.clone_img.rows,
                  QImage::Format_RGB888);
  ui.cam_img->setPixmap(QPixmap::fromImage(qraw_img.rgbSwapped()));
  delete qnode.raw_img;
  qnode.imgIsRcvd = false;
  if (!qnode.raw_img->empty())
  {
    qnode.raw_img = NULL;
  }
  img_fps++;
}

void MainWindow::updateLidarStatSlot()
{
  ui.lidar_stat->setText("OK!");
  ui.lidar_stat->setStyleSheet("color: green;");
  qnode.laserIsRcvd = false;
}

void MainWindow::updateHunterStatSlot()
{
  ui.hunter_stat->setText("OK!");
  ui.hunter_stat->setStyleSheet("color: green;");
  qnode.hunterIsRcvd = false;
}

void MainWindow::fpsUpdate()
{
  ui.fps->setText(QString::number(img_fps));
  if (!qnode.imgIsRcvd)
  {
    ui.cam_stat->setText("DISCONNECTED");
    ui.cam_stat->setStyleSheet("color: red;");
  }
  img_fps = 0;
  if (!qnode.laserIsRcvd)
  {
    ui.lidar_stat->setText("DISCONNECTED");
    ui.lidar_stat->setStyleSheet("color: red;");
  }
  if (!qnode.hunterIsRcvd)
  {
    ui.hunter_stat->setText("DISCONNECTED");
    ui.hunter_stat->setStyleSheet("color: red;");
  }
}
}  // namespace hunter_master_controller
