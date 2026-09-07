#include <autolabor_operator_gui/main_window.h>

#include <ros/ros.h>

#include <QApplication>
#include <QFont>

int main(int argc, char** argv)
{
  // The vehicle console is normally used on a high-DPI field display.  These
  // attributes must be set before QApplication so Qt uses logical pixels
  // consistently instead of leaving QSS pixel fonts physically tiny.
  QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

  ros::init(argc, argv, "autolabor_operator_gui",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  QApplication application(argc, argv);
  application.setApplicationName(QStringLiteral("Autolabor Operator Console"));
  application.setOrganizationName(QStringLiteral("Autolabor"));
  QFont application_font = application.font();
  application_font.setPointSizeF(13.0);
  application.setFont(application_font);

  int result = 0;
  {
    autolabor_operator_gui::MainWindow window;
    window.showMaximized();
    result = application.exec();
  }  // Join application callbacks and destroy RViz before shutting ROS down.

  ros::shutdown();
  return result;
}
