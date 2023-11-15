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
  QObject::connect(&qnode, SIGNAL(updateAcuroImageSignal()), this, SLOT(updateAcuroImageSlot()));
  QObject::connect(&qnode, SIGNAL(updateLaserSignal()), this, SLOT(updateLidarStatSlot()));
  QObject::connect(&qnode, SIGNAL(updateHunterSignal()), this, SLOT(updateHunterStatSlot()));

  QString pathr = QString::fromStdString(qnode.rviz_path);
  ROS_INFO("Opening RVIZ from path : %s", qnode.rviz_path.c_str());

  rviz_frame_ = new rviz::VisualizationFrame();
  rviz_frame_->setParent(ui.rvizFrame);
  rviz_frame_->initialize(pathr);
  rviz_frame_->setSplashPath("");
  rviz_frame_->setHideButtonVisibility(false);

  rviz_manager_ = rviz_frame_->getManager();
  QVBoxLayout* frameLayout = new QVBoxLayout(ui.rvizFrame);
  frameLayout->addWidget(rviz_frame_);
  rviz_frame_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  rviz_frame_2 = new rviz::VisualizationFrame();
  rviz_frame_2->setParent(ui.rvizFrame_2);
  rviz_frame_2->initialize(pathr);
  rviz_frame_2->setSplashPath("");
  rviz_frame_2->setHideButtonVisibility(false);

  rviz_manager_2 = rviz_frame_2->getManager();
  QVBoxLayout* frameLayout2 = new QVBoxLayout(ui.rvizFrame_2);
  frameLayout2->addWidget(rviz_frame_2);
  rviz_frame_2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

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
  system("roslaunch baram_nav nav.launch &");
}

void MainWindow::on_aruco_btn_clicked()
{
  ROS_INFO("Now Running aruco detector Bringup!");
  system("roslaunch aruco_detector_ocv detector.launch &");
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

void MainWindow::updateAcuroImageSlot()
{
  QImage qraw_img((const unsigned char*)(qnode.acuro_img.data), qnode.acuro_img.cols, qnode.acuro_img.rows,
                  QImage::Format_RGB888);
  ui.aruco_img->setPixmap(QPixmap::fromImage(qraw_img.rgbSwapped()));
  delete qnode.acuro_raw_img;
  qnode.imgIsRcvd = false;
  if (!qnode.raw_img->empty())
  {
    qnode.raw_img = NULL;
  }
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

void MainWindow::on_robit_clicked()
{
  ROS_INFO("ROBIT");
  if (robit)
  {
    ui.robit->setIcon(QIcon());
    ui.robit->setStyleSheet("background-color: transparent;");
    robit = false;
    return;
  }
  else if (!robit)
  {
    QPixmap pixmap(":/images/robit.jpg");
    QPixmap scaledPixmap = pixmap.scaled(ui.robit->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
    ui.robit->setIcon(QIcon(scaledPixmap));
    robit = true;
    return;
  }
}

void MainWindow::on_darkmode_checkbox_clicked()
{
  if (ui.darkmode_checkbox->isChecked())
  {
    QString styleSheet =
        "QMainWindow {"
        "    background-color: rgb(60, 58, 57);"
        "    font: 11pt;"
        "    color: white;"
        "}";
    this->setStyleSheet(styleSheet);
  }
  else if (!ui.darkmode_checkbox->isChecked())
  {
    QString styleSheet =
        "QMainWindow {"
        "    background-color: rgb(255, 255, 255);"
        "    font: 11pt;"
        "    color: white;"
        "}";
    this->setStyleSheet(styleSheet);
  }
}

}  // namespace hunter_master_controller
