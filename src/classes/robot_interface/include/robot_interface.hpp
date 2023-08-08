#include <vector>
#include <iostream>
#include <deque>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "scion_types/srv/send_frame.hpp"
#include "scion_types/msg/idea.hpp"
#include "scion_types/msg/state.hpp"
#include "scion_types/msg/position.hpp"
#include "scion_types/msg/orientation.hpp"
#include "scion_types/msg/datapoint.hpp"
#include "scion_types/msg/vision_object.hpp"
#include "scion_types/msg/zed_object.hpp"
#include "scion_types/msg/pid_tuning.hpp"
#include "scion_types/msg/sub_state.hpp"
#include "scion_types/action/pid.hpp"
#include "std_msgs/msg/int32.hpp"

namespace Interface
{
    union Function;      
    union Params; 
    struct Command;

    /* Custom Types to Use In Control System */
    typedef std::vector<float>                                                              current_state_t;
    typedef std::vector<float>                                                              desired_state_t;
    typedef scion_types::msg::Idea                                                          idea_message_t;
    typedef std::vector<scion_types::msg::Idea>                                             idea_vector_t;
    typedef std::deque<Command>                                                             command_queue_t;
    typedef std::vector<Command>                                                            command_vector_t;

    typedef rclcpp::Node::SharedPtr                                                         node_t;
    typedef rclcpp_action::Client<scion_types::action::PID>::SharedPtr                      pid_action_client_t;
    typedef rclcpp_action::Server<scion_types::action::PID>::SharedPtr                      pid_action_server_t;
    typedef rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr                               ros_trigger_client_t;
    typedef rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr                               ros_bool_client_t;
    typedef rclcpp::Client<scion_types::srv::SendFrame>::SharedPtr                          ros_sendframe_client_t;
    typedef rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr                              ros_trigger_service_t;
    typedef rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                              ros_bool_service_t;
    typedef rclcpp::Service<scion_types::srv::SendFrame>::SharedPtr                         ros_sendframe_service_t;
    typedef rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr                           int_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::PidTuning>::SharedPtr                    tune_pid_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Idea>::SharedPtr                         idea_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::State>::SharedPtr                        state_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Position>::SharedPtr                     position_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Orientation>::SharedPtr                  orientation_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::Datapoint>::SharedPtr                    datapoint_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::VisionObject>::SharedPtr                 object_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::ZedObject>::SharedPtr                    vision_sub_t;
    typedef rclcpp::Subscription<scion_types::msg::SubState>::SharedPtr                     sub_state_sub_t;
    typedef rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                              int_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::PidTuning>::SharedPtr                       tune_pid_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Idea>::SharedPtr                            idea_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::State>::SharedPtr                           state_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Position>::SharedPtr                        position_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Orientation>::SharedPtr                     orientation_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::Datapoint>::SharedPtr                       datapoint_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::VisionObject>::SharedPtr                    object_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::ZedObject>::SharedPtr                       vision_pub_t;
    typedef rclcpp::Publisher<scion_types::msg::SubState>::SharedPtr                        sub_state_pub_t;
    typedef std::vector<std::vector<float>>                                                 matrix_t;                                                                            
    typedef rclcpp::TimerBase::SharedPtr                                                    ros_timer_t;
}