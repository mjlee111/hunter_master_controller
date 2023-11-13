/**
 * @file /include/hunter_master_controller/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date November 2010
 **/
#ifndef hunter_master_controller_MAIN_WINDOW_H
#define hunter_master_controller_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <QTimer>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace hunter_master_controller
{
/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget* parent = 0);
  ~MainWindow();

public Q_SLOTS:
  void on_hunter_btn_clicked();
  void on_cam_lidar_btn_clicked();
  void on_nav_btn_clicked();
  void on_aruco_btn_clicked();
  void updateImageSlot();
  void updateLidarStatSlot();
  void updateHunterStatSlot();
  void fpsUpdate();

private:
  Ui::MainWindowDesign ui;
  QNode qnode;

  int img_fps = 0;
  QTimer* _1s_timer;
};

}  // namespace hunter_master_controller

#endif  // hunter_master_controller_MAIN_WINDOW_H