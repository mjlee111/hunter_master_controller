/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/hunter_master_controller/main_window.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv)
{
  /*********************
  ** Qt
  **********************/
  QApplication app(argc, argv);
  hunter_master_controller::MainWindow w(argc, argv);
  w.setWindowOpacity(0.8);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

  return result;
}
