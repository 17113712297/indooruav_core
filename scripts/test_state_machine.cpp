#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>

namespace {
constexpr double kDefaultEventWaitTimeoutSec = 2.0;
constexpr double kDefaultAutoStepIntervalSec = 1.0;
}

class StateMachineTester {
public:
    StateMachineTester()
        : private_nh_("~"),
          event_service_wait_timeout_sec_(kDefaultEventWaitTimeoutSec),
          auto_step_interval_sec_(kDefaultAutoStepIntervalSec),
          advertise_dummy_action_services_(true) {
        LoadParameters();
        InitializeClients();
        InitializeActionServers();
    }

    void RunInteractiveLoop() {
        PrintHelp();

        std::string command;
        while (ros::ok()) {
            std::cout << "\n[test_state_machine] command> " << std::flush;
            if (!std::getline(std::cin, command)) {
                ROS_INFO("Standard input closed. Exit test node.");
                break;
            }

            if (command.empty()) {
                continue;
            }

            if (command == "q" || command == "quit" || command == "exit") {
                ROS_INFO("Exit test node.");
                break;
            }

            if (command == "h" || command == "help") {
                PrintHelp();
                continue;
            }

            if (command == "a" || command == "auto") {
                RunAutoSequence();
                continue;
            }

            if (!DispatchCommand(command)) {
                ROS_WARN_STREAM("Unknown command [" << command << "]. Enter h for help.");
            }
        }
    }

private:
    typedef bool (StateMachineTester::*ActionServiceCallback)(std_srvs::Empty::Request&,
                                                              std_srvs::Empty::Response&);

    void LoadParameters() {
        nh_.param<std::string>("/indooruav_core/state_machine_event/takeoff_command",
                               takeoff_command_service_name_,
                               "indooruav_core/state_machine_event/takeoff_command");
        nh_.param<std::string>("/indooruav_core/state_machine_event/check_passed",
                               check_passed_service_name_,
                               "indooruav_core/state_machine_event/check_passed");
        nh_.param<std::string>("/indooruav_core/state_machine_event/takeoff_complete",
                               takeoff_complete_service_name_,
                               "indooruav_core/state_machine_event/takeoff_complete");
        nh_.param<std::string>("/indooruav_core/state_machine_event/cruise_complete",
                               cruise_complete_service_name_,
                               "indooruav_core/state_machine_event/cruise_complete");
        nh_.param<std::string>("/indooruav_core/state_machine_event/land_complete",
                               land_complete_service_name_,
                               "indooruav_core/state_machine_event/land_complete");
        nh_.param<std::string>("/indooruav_core/state_machine_event/charge_complete",
                               charge_complete_service_name_,
                               "indooruav_core/state_machine_event/charge_complete");
        nh_.param<std::string>("/indooruav_core/state_machine_event/data_collection_start",
                               data_collection_start_service_name_,
                               "indooruav_core/state_machine_event/data_collection_start");
        nh_.param<std::string>("/indooruav_core/state_machine_event/data_collection_complete",
                               data_collection_complete_service_name_,
                               "indooruav_core/state_machine_event/data_collection_complete");

        nh_.param<std::string>("/indooruav_core/action/await",
                               action_await_service_name_,
                               "indooruav_core/action/await");
        nh_.param<std::string>("/indooruav_core/action/check_before_takeoff",
                               action_check_before_takeoff_service_name_,
                               "indooruav_core/action/check_before_takeoff");
        nh_.param<std::string>("/indooruav_core/action/takeoff",
                               action_takeoff_service_name_,
                               "indooruav_controller/controller_hardware/takeoff");
        nh_.param<std::string>("/indooruav_core/action/cruise",
                               action_cruise_service_name_,
                               "indooruav_controller/waypoint_tracker/start");
        nh_.param<std::string>("/indooruav_core/action/land",
                               action_land_service_name_,
                               "indooruav_controller/controller_hardware/land");
        nh_.param<std::string>("/indooruav_core/action/charge",
                               action_charge_service_name_,
                               "indooruav_core/action/charge");
        nh_.param<std::string>("/indooruav_core/action/data_collection",
                               action_data_collection_service_name_,
                               "indooruav_core/action/data_collection");
        nh_.param<std::string>("/indooruav_core/action/notify_waypoint_tracker_disable",
                               action_notify_waypoint_tracker_disable_service_name_,
                               "indooruav_controller/waypoint_tracker/stop");
        nh_.param<std::string>("/indooruav_core/action/notify_uav_open_light",
                               action_notify_uav_open_light_service_name_,
                               "indooruav_controller/controller_hardware/light_open");
        nh_.param<std::string>("/indooruav_core/action/notify_uav_close_light",
                               action_notify_uav_close_light_service_name_,
                               "indooruav_controller/controller_hardware/light_close");
        nh_.param<std::string>("/indooruav_core/action/notify_uav_video_recording_start",
                               action_notify_uav_video_recording_start_service_name_,
                               "indooruav_controller/controller_hardware/camera_video_start");
        nh_.param<std::string>("/indooruav_core/action/notify_uav_video_recording_stop",
                               action_notify_uav_video_recording_stop_service_name_,
                               "indooruav_controller/controller_hardware/camera_video_stop");

        private_nh_.param("event_service_wait_timeout_sec",
                          event_service_wait_timeout_sec_,
                          kDefaultEventWaitTimeoutSec);
        private_nh_.param("auto_step_interval_sec",
                          auto_step_interval_sec_,
                          kDefaultAutoStepIntervalSec);
        private_nh_.param("advertise_dummy_action_services",
                          advertise_dummy_action_services_,
                          true);
    }

    void InitializeClients() {
        takeoff_command_client_ = nh_.serviceClient<std_srvs::Empty>(takeoff_command_service_name_);
        check_passed_client_ = nh_.serviceClient<std_srvs::Empty>(check_passed_service_name_);
        takeoff_complete_client_ = nh_.serviceClient<std_srvs::Empty>(takeoff_complete_service_name_);
        cruise_complete_client_ = nh_.serviceClient<std_srvs::Empty>(cruise_complete_service_name_);
        land_complete_client_ = nh_.serviceClient<std_srvs::Empty>(land_complete_service_name_);
        charge_complete_client_ = nh_.serviceClient<std_srvs::Empty>(charge_complete_service_name_);
        data_collection_start_client_ = nh_.serviceClient<std_srvs::Empty>(data_collection_start_service_name_);
        data_collection_complete_client_ = nh_.serviceClient<std_srvs::Empty>(data_collection_complete_service_name_);
    }

    void InitializeActionServers() {
        action_await_server_ =
            MaybeAdvertiseActionService(action_await_service_name_, &StateMachineTester::HandleActionAwait);
        action_check_before_takeoff_server_ =
            MaybeAdvertiseActionService(action_check_before_takeoff_service_name_,
                                        &StateMachineTester::HandleActionCheckBeforeTakeoff);
        action_takeoff_server_ =
            MaybeAdvertiseActionService(action_takeoff_service_name_, &StateMachineTester::HandleActionTakeoff);
        action_cruise_server_ =
            MaybeAdvertiseActionService(action_cruise_service_name_, &StateMachineTester::HandleActionCruise);
        action_land_server_ =
            MaybeAdvertiseActionService(action_land_service_name_, &StateMachineTester::HandleActionLand);
        action_charge_server_ =
            MaybeAdvertiseActionService(action_charge_service_name_, &StateMachineTester::HandleActionCharge);
        action_data_collection_server_ =
            MaybeAdvertiseActionService(action_data_collection_service_name_,
                                        &StateMachineTester::HandleActionDataCollection);
        action_notify_waypoint_tracker_disable_server_ =
            MaybeAdvertiseActionService(action_notify_waypoint_tracker_disable_service_name_,
                                        &StateMachineTester::HandleActionNotifyWaypointTrackerDisable);
        action_notify_uav_open_light_server_ =
            MaybeAdvertiseActionService(action_notify_uav_open_light_service_name_,
                                        &StateMachineTester::HandleActionNotifyUavOpenLight);
        action_notify_uav_close_light_server_ =
            MaybeAdvertiseActionService(action_notify_uav_close_light_service_name_,
                                        &StateMachineTester::HandleActionNotifyUavCloseLight);
        action_notify_uav_video_recording_start_server_ =
            MaybeAdvertiseActionService(action_notify_uav_video_recording_start_service_name_,
                                        &StateMachineTester::HandleActionNotifyUavVideoRecordingStart);
        action_notify_uav_video_recording_stop_server_ =
            MaybeAdvertiseActionService(action_notify_uav_video_recording_stop_service_name_,
                                        &StateMachineTester::HandleActionNotifyUavVideoRecordingStop);
    }

    ros::ServiceServer MaybeAdvertiseActionService(const std::string& service_name,
                                                   ActionServiceCallback callback) {
        if (!advertise_dummy_action_services_) {
            ROS_INFO_STREAM("Skip dummy action service [" << service_name
                            << "] because ~advertise_dummy_action_services is false.");
            return ros::ServiceServer();
        }

        if (ros::service::exists(service_name, false)) {
            ROS_INFO_STREAM("Detected existing action service [" << service_name
                            << "], skip dummy server.");
            return ros::ServiceServer();
        }

        ROS_INFO_STREAM("Advertising dummy action service [" << service_name << "].");
        return nh_.advertiseService(service_name, callback, this);
    }

    bool DispatchCommand(const std::string& command) {
        if (command == "1" || command == "takeoff_command") {
            return CallEventService("takeoff_command",
                                    takeoff_command_service_name_,
                                    takeoff_command_client_);
        }

        if (command == "2" || command == "check_passed") {
            return CallEventService("check_passed",
                                    check_passed_service_name_,
                                    check_passed_client_);
        }

        if (command == "3" || command == "takeoff_complete") {
            return CallEventService("takeoff_complete",
                                    takeoff_complete_service_name_,
                                    takeoff_complete_client_);
        }

        if (command == "4" || command == "cruise_complete") {
            return CallEventService("cruise_complete",
                                    cruise_complete_service_name_,
                                    cruise_complete_client_);
        }

        if (command == "5" || command == "land_complete") {
            return CallEventService("land_complete",
                                    land_complete_service_name_,
                                    land_complete_client_);
        }

        if (command == "6" || command == "charge_complete") {
            return CallEventService("charge_complete",
                                    charge_complete_service_name_,
                                    charge_complete_client_);
        }

        if (command == "7" || command == "data_collection_start") {
            return CallEventService("data_collection_start",
                                    data_collection_start_service_name_,
                                    data_collection_start_client_);
        }

        if (command == "8" || command == "data_collection_complete") {
            return CallEventService("data_collection_complete",
                                    data_collection_complete_service_name_,
                                    data_collection_complete_client_);
        }

        return false;
    }

    bool CallEventService(const std::string& event_name,
                          const std::string& service_name,
                          ros::ServiceClient& client) {
        ROS_INFO_STREAM("Calling event [" << event_name << "] via service [" << service_name << "].");

        if (!client.waitForExistence(ros::Duration(event_service_wait_timeout_sec_))) {
            ROS_ERROR_STREAM("Event service [" << service_name
                             << "] is not available within "
                             << event_service_wait_timeout_sec_ << " seconds.");
            return false;
        }

        std_srvs::Empty service;
        if (!client.call(service)) {
            ROS_ERROR_STREAM("Failed to call event service [" << service_name << "].");
            return false;
        }

        ROS_INFO_STREAM("Event [" << event_name << "] call succeeded.");
        return true;
    }

    void RunAutoSequence() {
        const std::vector<std::string> commands = {
            "takeoff_command",
            "check_passed",
            "takeoff_complete",
            "data_collection_start",
            "data_collection_complete",
            "cruise_complete",
            "land_complete",
            "charge_complete"
        };

        ROS_INFO("Start auto test sequence.");
        for (std::size_t index = 0; index < commands.size() && ros::ok(); ++index) {
            if (!DispatchCommand(commands[index])) {
                ROS_ERROR_STREAM("Auto test sequence stopped at command ["
                                 << commands[index] << "].");
                return;
            }

            if (index + 1 < commands.size()) {
                ros::Duration(auto_step_interval_sec_).sleep();
            }
        }

        ROS_INFO("Auto test sequence completed.");
    }

    void PrintHelp() const {
        std::cout << "\nAvailable commands:\n"
                  << "  1 / takeoff_command   -> trigger Event::TakeoffCommand\n"
                  << "  2 / check_passed      -> trigger Event::CheckPassed\n"
                  << "  3 / takeoff_complete  -> trigger Event::TakeoffComplete\n"
                  << "  4 / cruise_complete   -> trigger Event::CruiseComplete\n"
                  << "  5 / land_complete     -> trigger Event::LandComplete\n"
                  << "  6 / charge_complete   -> trigger Event::ChargeComplete\n"
                  << "  7 / data_collection_start    -> trigger Event::DataCollectionStart\n"
                  << "  8 / data_collection_complete -> trigger Event::DataCollectionComplete\n"
                  << "  a / auto              -> run the full event sequence once\n"
                  << "  h / help              -> show this help\n"
                  << "  q / quit              -> exit\n"
                  << "\nPrivate params:\n"
                  << "  ~event_service_wait_timeout_sec   default: " << kDefaultEventWaitTimeoutSec << "\n"
                  << "  ~auto_step_interval_sec           default: " << kDefaultAutoStepIntervalSec << "\n"
                  << "  ~advertise_dummy_action_services  default: true\n"
                  << std::endl;
    }

    bool HandleActionRequest(const std::string& action_name,
                             std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response) {
        (void)request;
        (void)response;
        ROS_INFO_STREAM("Received state machine action request [" << action_name << "].");
        return true;
    }

    bool HandleActionAwait(std_srvs::Empty::Request& request,
                           std_srvs::Empty::Response& response) {
        return HandleActionRequest("await", request, response);
    }

    bool HandleActionCheckBeforeTakeoff(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response) {
        return HandleActionRequest("check_before_takeoff", request, response);
    }

    bool HandleActionTakeoff(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response) {
        return HandleActionRequest("takeoff", request, response);
    }

    bool HandleActionCruise(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response) {
        return HandleActionRequest("cruise", request, response);
    }

    bool HandleActionLand(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response) {
        return HandleActionRequest("land", request, response);
    }

    bool HandleActionCharge(std_srvs::Empty::Request& request,
                            std_srvs::Empty::Response& response) {
        return HandleActionRequest("charge", request, response);
    }

    bool HandleActionDataCollection(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response) {
        return HandleActionRequest("data_collection", request, response);
    }

    bool HandleActionNotifyWaypointTrackerDisable(std_srvs::Empty::Request& request,
                                                  std_srvs::Empty::Response& response) {
        return HandleActionRequest("notify_waypoint_tracker_disable", request, response);
    }

    bool HandleActionNotifyUavOpenLight(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response) {
        return HandleActionRequest("notify_uav_open_light", request, response);
    }

    bool HandleActionNotifyUavCloseLight(std_srvs::Empty::Request& request,
                                         std_srvs::Empty::Response& response) {
        return HandleActionRequest("notify_uav_close_light", request, response);
    }

    bool HandleActionNotifyUavVideoRecordingStart(std_srvs::Empty::Request& request,
                                                  std_srvs::Empty::Response& response) {
        return HandleActionRequest("notify_uav_video_recording_start", request, response);
    }

    bool HandleActionNotifyUavVideoRecordingStop(std_srvs::Empty::Request& request,
                                                 std_srvs::Empty::Response& response) {
        return HandleActionRequest("notify_uav_video_recording_stop", request, response);
    }

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    std::string takeoff_command_service_name_;
    std::string check_passed_service_name_;
    std::string takeoff_complete_service_name_;
    std::string cruise_complete_service_name_;
    std::string land_complete_service_name_;
    std::string charge_complete_service_name_;
    std::string data_collection_start_service_name_;
    std::string data_collection_complete_service_name_;

    std::string action_await_service_name_;
    std::string action_check_before_takeoff_service_name_;
    std::string action_takeoff_service_name_;
    std::string action_cruise_service_name_;
    std::string action_land_service_name_;
    std::string action_charge_service_name_;
    std::string action_data_collection_service_name_;
    std::string action_notify_waypoint_tracker_disable_service_name_;
    std::string action_notify_uav_open_light_service_name_;
    std::string action_notify_uav_close_light_service_name_;
    std::string action_notify_uav_video_recording_start_service_name_;
    std::string action_notify_uav_video_recording_stop_service_name_;

    double event_service_wait_timeout_sec_;
    double auto_step_interval_sec_;
    bool advertise_dummy_action_services_;

    ros::ServiceClient takeoff_command_client_;
    ros::ServiceClient check_passed_client_;
    ros::ServiceClient takeoff_complete_client_;
    ros::ServiceClient cruise_complete_client_;
    ros::ServiceClient land_complete_client_;
    ros::ServiceClient charge_complete_client_;
    ros::ServiceClient data_collection_start_client_;
    ros::ServiceClient data_collection_complete_client_;

    ros::ServiceServer action_await_server_;
    ros::ServiceServer action_check_before_takeoff_server_;
    ros::ServiceServer action_takeoff_server_;
    ros::ServiceServer action_cruise_server_;
    ros::ServiceServer action_land_server_;
    ros::ServiceServer action_charge_server_;
    ros::ServiceServer action_data_collection_server_;
    ros::ServiceServer action_notify_waypoint_tracker_disable_server_;
    ros::ServiceServer action_notify_uav_open_light_server_;
    ros::ServiceServer action_notify_uav_close_light_server_;
    ros::ServiceServer action_notify_uav_video_recording_start_server_;
    ros::ServiceServer action_notify_uav_video_recording_stop_server_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_state_machine");

    StateMachineTester tester;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    tester.RunInteractiveLoop();

    spinner.stop();
    return 0;
}
