#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

#include "yasmin/cb_state.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/publisher_state.hpp"
#include "yasmin_ros/ros_logs.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace yasmin;

class PublishTakeoffState
    : public yasmin_ros::PublisherState<std_msgs::msg::Empty> {

public:
  PublishTakeoffState()
      : yasmin_ros::PublisherState<std_msgs::msg::Empty>(
            "takeoff", // topic name
            std::bind(&PublishTakeoffState::create_empty_msg, this,
                      _1) // create msg handler callback
        ){};

  std_msgs::msg::Empty
  create_empty_msg(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
    YASMIN_LOG_INFO("Publishing takeoff message");
    std_msgs::msg::Empty msg;
    return msg;
  };
};

std::string
check_takeoff(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {

  // Sleep for 1 second to simulate some processing time
  rclcpp::sleep_for(std::chrono::seconds(1));
  YASMIN_LOG_INFO("Checking takeoff...");

  if (blackboard->get<int>("counter") >= blackboard->get<int>("max_count")) {
    return "taking_off";
  } else {
    return "waiting_connection";
  }
}

int main(int argc, char *argv[]) {

  YASMIN_LOG_INFO("yasmin_dji_tello");
  rclcpp::init(argc, argv);

  yasmin_ros::set_ros_loggers();

  auto sm = std::make_shared<yasmin::StateMachine>(
      std::initializer_list<std::string>{yasmin_ros::basic_outcomes::SUCCEED});

  rclcpp::on_shutdown([sm]() {
    if (sm->is_running()) {
      sm->cancel_state();
    }
  });

  sm->add_state("PUBLISHING_TAKEOFF", std::make_shared<PublishTakeoffState>(),
                {
                    {yasmin_ros::basic_outcomes::SUCCEED,
                     "PUBLISHING_TAKEOFF"}, // Transition back to itself
                });
  sm->add_state("CHECKING_TAKEOFF_STATUS",
                std::make_shared<yasmin::CbState>(
                    std::initializer_list<std::string>{"taking_off", "waiting_connection"},
                    check_takeoff),
                {{"waiting_connection", yasmin_ros::basic_outcomes::SUCCEED},
                 {"taking_off", "PUBLISHING_TAKEOFF"}});

  yasmin_viewer::YasminViewerPub yasmin_pub("YASMIN_DJI_TELLO", sm);

  std::shared_ptr<yasmin::blackboard::Blackboard> blackboard =
      std::make_shared<yasmin::blackboard::Blackboard>();
  try {
    std::string outcome = (*sm.get())(blackboard);
    YASMIN_LOG_INFO(outcome.c_str());
  } catch (const std::exception &e) {
    YASMIN_LOG_WARN(e.what());
  }

  rclcpp::shutdown();

  return 0;
}
