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
    return LaunchDescription([)";

    std::string pythonFilePath = "";
    void printLaunchFile(const std::string& content);
    void printLaunchParamsVec();

private slots:

    void on_saveLaunchDescription_clicked();
    void on_saveClose_clicked();

private:
    Ui::LaunchEdit *ui;
    struct LaunchParameters {
        std::string pkgName;
        std::string execName;
        std::string output;
        std::map<std::string, std::variant <std::string, bool, int>> params = {};

            // Constructor that accepts string literals
        LaunchParameters(const char* pkg, const char* exec, const char* out,
                         const std::map<std::string, std::variant<std::string, bool, int>>& p = {})
            :pkgName(pkg), execName(exec), output(out), params(p){}

    };

    std::string addNodeString(const LaunchParameters& node);   

    std::vector<LaunchParameters> launchParamsVec = {
        {"a50_node",            "a50_exec",            "screen"},
        {"absolute_state_node", "absolute_state_exec", "screen"},
        {"ahrs_node",           "ahrs_exec",           "screen"},
        {"brain_node",          "brain_exec",          "screen"},
        {"desired_state_node",   "desired_state_exec", "screen"},
        {"gui_node",             "gui_exec",           "screen"},
        {"joy",                  "joy_node",           "screen"},
        {"can_listener",         "can_listener",       "screen"},
        {"mediator_node",        "mediator_exec",      "screen"},
        {"pid_node",             "pid_exec",           "screen"},
        {"robot_library",        "robot",              "screen"},
        {"test_node",            "test_exec",          "screen"},
        {"zed_orientation_node", "zed_orientation_exec",  "log"},
        {"zed_pos_node",         "zed_pos_exec",          "log"},
        {"zed_vision_node",      "zed_vision_exec",       "log"},

        {"controller_node",      "controller_exec",     "screen", 
            {
                {"thrust_mapper", std::string("percy")}
            }
        },

        {"current_state_node",   "current_state_exec",  "screen",
            {
                {"use_position_tracking", true},
                {"use_orientation_tracking", true} 
            }
        },

        {"unified_can_driver",  "unified_can_driver",   "screen",
            {
                {"can_bus_interface", std::string("can0")},
                {"do_module_polling", false},
                {"module_bitfield", 0}
            }
        }
    };

    std::string getLaunchParamsVecAsString();

};

#endif // LAUNCH_EDIT_HPP
