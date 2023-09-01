#ifndef LAUNCH_EDIT_HPP
#define LAUNCH_EDIT_HPP

#include <QWidget>
#include <mainwindow.hpp>
#include <map>

namespace Ui {
class LaunchEdit;
}

class LaunchEdit : public QWidget
{
    Q_OBJECT

public:
    explicit LaunchEdit(QWidget *parent = nullptr);
    ~LaunchEdit();

    std::string launchDescription = R"("""
    @author Conner Sommerfield - Zix on Discord
    Launch file for all ROS nodes of the Robot
    Currently will turn on 
        - AHRS Orientation node
        - CAN Driver node
        - DVL Velocity Node
        - MS5837 Depth Node
        - PID Controller Node
        - Specific Zed Position Node (filters data from Zed Node for position data)
        - Zed Node which contains various topics
    
    Run using ros2 launch launch.py in terminal (make sure you're in the src/ folder and that you've sourced)
"""
    
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
import subprocess
    
def generate_launch_description():
    """ 
    This is what Ros2 will actually call when you run the launch file,
    all the nodes are placed here as well as Zed Node launch arguments
    """
    return LaunchDescription([
)";

    std::string pythonFilePath = "";
    std::string nodeList = "";
    std::vector<std::map<std::string, std::string>> nodeArray;
    void printLaunchFile(const std::string& content);
    // void addNodeString(const std::string& pkgName, const std::string& execName
    //                    std::vector<std::map<std::string, std::string>> params);

private slots:

    void on_saveLaunchDescription_clicked();
    void on_saveClose_clicked();

private:
    Ui::LaunchEdit *ui;

};

#endif // LAUNCH_EDIT_HPP
