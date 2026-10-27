#include <autolabor_operator_gui/main_window.h>

#include <ros/ros.h>

#include <QApplication>
#include <QFont>
#include <QTabWidget>
#include <QTimer>
#include <QPixmap>

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
    ros::NodeHandle startup("~");
    bool initial_tuning_page = false;
    startup.param("initial_tuning_page", initial_tuning_page, false);
    std::string tuning_screenshot;
    startup.param<std::string>("tuning_screenshot", tuning_screenshot, "");
    if (initial_tuning_page) {
      window.setWindowTitle(qgetenv("SURROUND_PREVIEW_URL").isEmpty()
          ? QStringLiteral("Autolabor 导航调参 · 离线预览（未连接实车）")
          : QStringLiteral("Autolabor 合并版 · 三路实景 / 导航离线调参"));
      for (auto* tabs : window.findChildren<QTabWidget*>())
        for (int i=0;i<tabs->count();++i)
          if (tabs->tabText(i)==QStringLiteral("导航调参")) tabs->setCurrentIndex(i);
    }
    std::string window_title;
    startup.param<std::string>("window_title", window_title, "");
    if (!window_title.empty())
      window.setWindowTitle(QString::fromStdString(window_title));
    bool initial_bev_page = false;
    startup.param("initial_bev_page", initial_bev_page, false);
    if (initial_bev_page)
      for (auto* tabs : window.findChildren<QTabWidget*>())
        for (int i=0; i<tabs->count(); ++i)
          if (tabs->tabText(i) == QStringLiteral("BEV 融合识别")) tabs->setCurrentIndex(i);
    window.showMaximized();
    if (!tuning_screenshot.empty())
      QTimer::singleShot(6000, &window, [&window,tuning_screenshot]() {
        window.grab().save(QString::fromStdString(tuning_screenshot));
      });
    result = application.exec();
  }  // Join application callbacks and destroy RViz before shutting ROS down.

  ros::shutdown();
  return result;
}
