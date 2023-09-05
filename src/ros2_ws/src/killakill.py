"""
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
        launch_ros.actions.Node(
            package='a50_node', executable='a50_exec', output='screen',
            ),

    ])
