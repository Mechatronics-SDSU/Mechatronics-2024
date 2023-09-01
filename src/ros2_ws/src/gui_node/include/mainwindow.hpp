#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QWidget>
#include <QMainWindow>
#include <mission_planning.hpp>
#include <pid_controller.hpp>
#include <launch_edit.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include "scion_types/msg/json_string.hpp"
#include <QCheckBox> 

QT_BEGIN_NAMESPACE
namespace Ui
{
    class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pid_controller_clicked();
    void on_mission_planning_clicked();
    //    void moveHome();

    void on_homeButton_clicked();
    // void on_visionButton_clicked();
    // void executeTerminalCommand(const char* command);

    void on_brain_toggled(bool checked);
    // void print_nodes_list();
    void print_nodes_list(const std::string& key, nlohmann::json& json_string, 
                          const nlohmann::json& jsonArray, QTextEdit* output);
    void on_mediator_toggled(bool checked);

    void on_pid_toggled(bool checked);
    void on_start_nodes_clicked();
    void on_new_launch_file_clicked();
    void on_select_file_clicked();
    void update_nodes_list(QString path);
    void launch_nodes_selected();
    void update_json_nodes(bool checked, QCheckBox* node);
    void on_launchNodesEdit_clicked();


    std::vector<std::string> getRosPackageNames(const std::string &directoryPath);

private:
    Ui::MainWindow *ui;
    MissionPlanning _mission_planner;
    PIDController _pid_controller;

    using json = nlohmann::json;
    json jsonMainArray = json::array();
    json main_nodes_string;

    json jsonLaunchArray = json::array();
    json launch_nodes_string;
    QString launchFileName = "";

    rclcpp::Node::SharedPtr json_string;
    rclcpp::Publisher<scion_types::msg::JsonString>::SharedPtr json_string_publisher;

};
#endif // MAINWINDOW_H    
