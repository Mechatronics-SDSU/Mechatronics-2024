/*
 * @author Conner Sommerfield - Zix on Discord
 * Brain node will send sequence of commands to the PIDs 
 * Commands can be pre-loaded or create with navigation logic
 * 
 * Robot will keep track of a "command queue"
 * This command queue will be abstracted as an vector/queue of Commands
 * Each command will be a function pointer to the action to perform 
 * and any parameters to pass to the function
 * 
 * The main ROS2 purpose of this node is to send a desired state to the
 * PIDs. It's the brain that tells the PIDs where the robot wants to go.
 * 
 * It will do this by taking the next out of the queue
 * 
 * A decision maker will be responsible for loading commands into the queue
 * 
 * PIDs will have to send command completion status for the queue mediator
 * to take out the next command.
 * 
 */

#include <vector>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>

#include "scion_types/action/pid.hpp"
#include "control_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "vector_operations.hpp"
#include "filter.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

#define TO_THE_RIGHT 15.0f
#define TO_THE_LEFT -15.0f
#define PIXEL_ERROR_THRESHOLD 50
#define SLEEP_TIME 50ms
#define SMOOTH_TURN_DEGREE 90.0f
#define SMOOTH_MOVE_DEGREE 1.0f
#define SUBMERGE_DISTANCE 1.5f
#define MID_X_PIXEL 640
#define MID_Y_PIXEL 360
#define NUM_CORNERS 4


class Brain : public rclcpp::Node
{
    typedef bool (Brain::*condition_t)();
    typedef void (Brain::*action_t)(float);
    typedef void (Brain::*cleanup_t)();

    public:
        explicit Brain(): Node("brain_node")
        {
            idea_pub_ = this->create_publisher<scion_types::msg::Idea>("brain_idea_data", 10);
            can_client_ = this->create_client<scion_types::srv::SendFrame>("send_can_raw");
            pid_ready_service_ = this->create_service<std_srvs::srv::Trigger>("pid_ready", std::bind(&Brain::ready, this, _1, _2));
        }
    private:
        Interface::idea_pub_t                       idea_pub_;
        Interface::ros_timer_t                      decision_timer_;
        Interface::idea_vector_t                    idea_sequence_;
        Interface::object_sub_t                     object_sub_;
        Interface::int_sub_t                        commands_in_queue_sub_;
        Interface::ros_sendframe_client_t           can_client_;
        Interface::ros_trigger_service_t            pid_ready_service_;
        std::string                                 mode_param_;
        bool                                        gate_seen_ = false;

        ////////////////////////////////////////////////////////////////////////////////
        //                               INIT MISSION                                 //
        ////////////////////////////////////////////////////////////////////////////////

        void ready(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, 
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
            this->performMission();
        }

        ////////////////////////////////////////////////////////////////////////////////
        //                               ASYNC FUNCTIONS                              //
        ////////////////////////////////////////////////////////////////////////////////

        void doUntil(action_t action, condition_t condition, cleanup_t cleanup, bool& condition_global, float parameter)
        {
            auto condition_met = std::bind(condition, this);
            std::thread(condition_met).detach();

            while(!condition_global) //this->gateSeen()
            {
                (this->*action)(parameter);
            }
            condition_global = false;
            (this->*cleanup)();
        }

        bool gateSeen()
        {
            std::promise<bool> gate_seen;
            std::shared_future<bool> future  = gate_seen.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("zed_object_subscriber");;
            Interface::object_sub_t object_sub = temp_node->create_subscription<scion_types::msg::VisionObject>
            ("zed_object_data", 10, [&temp_node, &gate_seen](const scion_types::msg::VisionObject::SharedPtr msg) {
                    if (msg->object_name == "Underwater-Gate") {
                        gate_seen.set_value(true);
                        RCLCPP_INFO(temp_node->get_logger(), "Gate seen");
                    }
            });
            rclcpp::spin_until_future_complete(temp_node, future);
            this->gate_seen_ = true;
            return true;
        }

        float getDistanceFromCamera(string object)
        {
            float distance;
            std::promise<bool> gate_seen;
            std::shared_future<bool> future  = gate_seen.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("zed_object_subscriber");;
            Interface::object_sub_t object_sub = temp_node->create_subscription<scion_types::msg::VisionObject>
            ("zed_object_data", 10, [&temp_node, &gate_seen, &object, &distance](const scion_types::msg::VisionObject::SharedPtr msg) {
                    if (msg->object_name == object) {
                        gate_seen.set_value(true);
                        distance = msg->distance;
                    }
            });
            rclcpp::spin_until_future_complete(temp_node, future);
            return distance;
        }

        unique_ptr<Filter> populateFilterBuffer()
        {
            size_t data_streams_num = 1;
            string coeff_file = "/home/mechatronics/master/ros2_ws/src/brain_node/coefficients.txt";
            unique_ptr<Filter> moving_average_filter = std::make_unique<Filter>(data_streams_num, coeff_file);

            std::promise<bool> buffer_filled;
            std::shared_future<bool> future  = buffer_filled.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_subscriber");
            Interface::vision_sub_t object_sub = temp_node->create_subscription<scion_types::msg::ZedObject>
            ("zed_vision_data", 10, [this, &temp_node, &buffer_filled, &moving_average_filter](const scion_types::msg::ZedObject::SharedPtr msg) {
                    unique_ptr<vector<vector<uint32_t>>>ros_bounding_box = zed_to_ros_bounding_box(msg->corners);
                    vector<uint32_t> bounding_box_midpoint = findMidpoint(*ros_bounding_box);
                    moving_average_filter->smooth(moving_average_filter->data_streams[0], (float)bounding_box_midpoint[0]);
                    if (moving_average_filter->data_streams[0][10] != 0)
                    {
                        buffer_filled.set_value(true);              // jump out of async spin
                    }
            });
            rclcpp::spin_until_future_complete(temp_node, future);
            return moving_average_filter;
        }

        void centerRobot()
        {
            unique_ptr<Filter> moving_average_filter = populateFilterBuffer();
            RCLCPP_INFO(this->get_logger(), "Filter Buffer Filled");
            vector<uint32_t> camera_frame_midpoint {MID_X_PIXEL, MID_Y_PIXEL};

            std::promise<bool> robot_centered;
            std::shared_future<bool> future  = robot_centered.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("zed_vision_subscriber");
            Interface::vision_sub_t object_sub = temp_node->create_subscription<scion_types::msg::ZedObject>
            ("zed_vision_data", 10, [this, &temp_node, &camera_frame_midpoint, &robot_centered, &moving_average_filter](const scion_types::msg::ZedObject::SharedPtr msg) {
                    unique_ptr<vector<vector<uint32_t>>>ros_bounding_box = zed_to_ros_bounding_box(msg->corners);
                    vector<uint32_t> bounding_box_midpoint = findMidpoint(*ros_bounding_box);
                    float filtered_bounding_box_midpoint = moving_average_filter->smooth(moving_average_filter->data_streams[0], (float)bounding_box_midpoint[0]);
                    
                    if (areEqual(bounding_box_midpoint, camera_frame_midpoint)) {robot_centered.set_value(true);}
                    else {
                        if (isCommandQueueEmpty())
                        {
                            RCLCPP_INFO(this->get_logger(), "Looking at bounding box with value %d", (*ros_bounding_box)[0][0]);
                            this->adjustToCenter(bounding_box_midpoint, camera_frame_midpoint);
                        }
                    }
            });
            rclcpp::spin_until_future_complete(temp_node, future);
        }

        std::unique_ptr<vector<vector<uint32_t>>> zed_to_ros_bounding_box(std::array<scion_types::msg::Keypoint2Di, 4>& zed_bounding_box)
        {
            vector<vector<uint32_t>> ros_bounding_box;                   // this is to convert from ros object to vector object
            for (int i = 0; i < NUM_CORNERS; i++)
            {
                vector<uint32_t> corner_point {((zed_bounding_box[i]).kp)[i], ((zed_bounding_box[i]).kp)[i+1]};
                ros_bounding_box.push_back(corner_point);
            }
            return std::make_unique<vector<vector<uint32_t>>>(ros_bounding_box);
        }

        vector<uint32_t> findMidpoint(vector<vector<uint32_t>>& bounding_box)
        {
            vector<uint32_t> cornerUL = bounding_box[0];
            vector<uint32_t> cornerUR = bounding_box[1];
            vector<uint32_t> cornerBL = bounding_box[2];
            uint32_t midpoint_x = ((cornerUR + cornerUL)/((uint32_t)2))[0];
            uint32_t midpoint_y = (720 - ((cornerUL + cornerBL)/((uint32_t)2))[1]);
            vector<uint32_t> midpoint {midpoint_x, midpoint_y};
            return midpoint;
        }

        bool areEqual(vector<uint32_t> point_a, vector<uint32_t> point_b)
        {
            return (abs((int)((point_a - point_b))[0]) < PIXEL_ERROR_THRESHOLD);
        }

        void adjustToCenter(vector<uint32_t> bounding_box_midpoint, vector<uint32_t> camera_frame_midpoint)
        {   
            bool bounding_box_is_to_the_right_of_center_pixel = bounding_box_midpoint[0] > camera_frame_midpoint[0];
            if (bounding_box_is_to_the_right_of_center_pixel) {this->turn(TO_THE_RIGHT);}
            else {this->turn(TO_THE_LEFT);}
        }

        void waitForCommandQueueEmpty()
        {
            std::promise<bool> command_queue_empty;
            std::shared_future<bool> future  = command_queue_empty.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("command_queue_empty");
            Interface::int_sub_t command_queue_empty_sub = temp_node->create_subscription<std_msgs::msg::Int32>
            ("is_command_queue_empty", 10, [this, &temp_node, &command_queue_empty](const std_msgs::msg::Int32::SharedPtr msg) {
                    if (msg->data) {command_queue_empty.set_value(true);}
            });
            rclcpp::spin_until_future_complete(temp_node, future);
        }

        bool isCommandQueueEmpty()
        {
            bool isEmpty;
            std::promise<bool> command_queue_empty;
            std::shared_future<bool> future  = command_queue_empty.get_future();
            Interface::node_t temp_node = rclcpp::Node::make_shared("command_queue_empty");
            Interface::int_sub_t command_queue_empty_sub = temp_node->create_subscription<std_msgs::msg::Int32>
            ("is_command_queue_empty", 10, [this, &temp_node, &isEmpty, &command_queue_empty](const std_msgs::msg::Int32::SharedPtr msg) {
                    command_queue_empty.set_value(msg->data);
                    isEmpty = msg->data;
            });
            rclcpp::spin_until_future_complete(temp_node, future);
            return isEmpty;
        }

        ////////////////////////////////////////////////////////////////////////////////
        //                               MOVEMENT IDEAS                               //
        ////////////////////////////////////////////////////////////////////////////////


        void stop()
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::STOP;
            idea_pub_->publish(idea);
        }

        void turn(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::TURN;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
            RCLCPP_INFO(this->get_logger(), "Turning %f Degrees", degree);
        }

        void pitch(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::PITCH;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void roll(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::ROLL;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void moveForward(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::MOVE;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void translate(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::TRANSLATE;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void levitate(float degree)
        {
            scion_types::msg::Idea idea = scion_types::msg::Idea();
            idea.code = Interface::Idea::LEVITATE;
            idea.parameters = std::vector<float>{degree};
            idea_pub_->publish(idea);
        }

        void keepTurning(float power)
        {
            this->turn(power);
            RCLCPP_INFO(this->get_logger(), "Turning");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        void keepMoving(float power)
        {
            this->moveForward(power);
            RCLCPP_INFO(this->get_logger(), "Moving Forward");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        ////////////////////////////////////////////////////////////////////////////////
        //                                  MISSION                                   //
        ////////////////////////////////////////////////////////////////////////////////

        void performMission()
        {
            this->centerRobot();
            doUntil(&Brain::keepTurning, &Brain::gateSeen, &Brain::stop, this->gate_seen_, SMOOTH_TURN_DEGREE);
            this->centerRobot();
            levitate(SUBMERGE_DISTANCE);
            moveForward(this->getDistanceFromCamera("Underwater-Gate")/2);
            this->centerRobot();
            exit(0);
        }

        ////////////////////////////////////////////////////////////////////////////////
        //                                  CALLBACK                                  //
        ////////////////////////////////////////////////////////////////////////////////



}; // class Brain


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}














































        ////////////////////////////////////////////////////////////////////////////////
        //                                  ARCHIVE                                   //
        ////////////////////////////////////////////////////////////////////////////////

        // auto logger = rclcpp::get_logger("my_logger");
        // RCLCPP_INFO(logger, "turning with power of %d", power);



/*      doUntil(&Brain::gateSeen, [](int power)
        {
            auto logger = rclcpp::get_logger("my_logger");
            RCLCPP_INFO(logger, "sent CAN Command of power %d", power);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        },  this->gate_seen_, 20); */

/* 
    void moveUntil(int power, condition_t condition, bool& condition_global)
    {
        // vector<unsigned char> motor_power{power, 0, power, 0, power, 0, power, 0};
        auto condition_met = std::bind(condition, this);
        std::thread(condition_met).detach();

        while(!condition_global) //this->gateSeen()
        {
            RCLCPP_INFO(this->get_logger(), "sent CAN Command");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        condition_global = false;
    } */


/* void initSequence(Interface::idea_vector_t& idea_sequence)
    {
        using namespace Interface;
        scion_types::msg::Idea idea1 = scion_types::msg::Idea();
        idea1.code = Idea::RELATIVE_POINT;
        idea1.parameters = std::vector<float>{0.0F,-0.3F};

        scion_types::msg::Idea idea2 = scion_types::msg::Idea();
        idea2.code = Idea::RELATIVE_POINT;
        idea2.parameters = std::vector<float>{0.0F,-0.3F};
    } */


/* void publishSequence(Interface::idea_vector_t& idea_sequence)
    {   
            using namespace Interface;
            for (idea_message_t& idea_message : idea_sequence)
            {
                sleep(1);
                this->idea_pub_->publish(idea_message);
            }
            RCLCPP_INFO(this->get_logger(), "Publishing Idea" );
    } */