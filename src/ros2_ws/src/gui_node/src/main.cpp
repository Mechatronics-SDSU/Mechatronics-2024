#include "mainwindow.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    QFile styleFile(STYLE_SHEET_FILE_PATH);
    styleFile.open(QFile::ReadOnly);
    QString style(styleFile.readAll());
    app.setStyleSheet(style);
    MainWindow window;
    window.show();
    app.exec();
    rclcpp::shutdown();
    return 0;
}