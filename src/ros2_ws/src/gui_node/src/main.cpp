#include "mainwindow.hpp"
#include <QApplication>
#include <QFile>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication a(argc, argv);
    QFile styleFile("/home/mechatronics/gui-halie/src/ros2_ws/src/gui_node/Darkeum.qss");
    styleFile.open(QFile::ReadOnly);
    QString style(styleFile.readAll());
    a.setStyleSheet(style);
    MainWindow w;
    w.show();
    a.exec();
    rclcpp::shutdown();
    return 0;
}
