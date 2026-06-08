#include "indooruav_core/action_request.h"

#include <std_srvs/Empty.h>

namespace {
constexpr double kDefaultActionServiceWaitTimeoutSec = 1.0;
constexpr double kServiceLogThrottleSec = 5.0;
}

ActionRequester::ActionRequester() {
    LoadParameters();
    InitializeClients();
}

ActionRequester::~ActionRequester() {
    // No dynamic memory to clean up.
}

void ActionRequester::LoadParameters() {
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
    nh_.param<std::string>("/indooruav_core/action/notify_http_post_land_workflow",
                           action_notify_http_post_land_workflow_service_name_,
                           "indooruav_http/run_post_land_workflow");
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
    nh_.param<std::string>("/indooruav_core/action/notify_uav_switch_video_mode",
                           action_notify_uav_switch_video_mode_service_name_,
                           "indooruav_controller/controller_hardware/camera_mode_video");
    nh_.param<std::string>("/indooruav_core/action/notify_uav_switch_photo_mode",
                           action_notify_uav_switch_photo_mode_service_name_,
                           "indooruav_controller/controller_hardware/camera_mode_photo");
    nh_.param<double>("/indooruav_core/action/wait_timeout_sec",
                      action_service_wait_timeout_sec_,
                      kDefaultActionServiceWaitTimeoutSec);
    if (action_service_wait_timeout_sec_ < 0.0) {
        ROS_WARN_STREAM("Invalid parameter [/indooruav_core/action/wait_timeout_sec]: "
                        << action_service_wait_timeout_sec_
                        << ". Falling back to default timeout "
                        << kDefaultActionServiceWaitTimeoutSec
                        << " seconds.");
        action_service_wait_timeout_sec_ = kDefaultActionServiceWaitTimeoutSec;
    }
}

void ActionRequester::InitializeClients() {
    action_await_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_await_service_name_);
    action_check_before_takeoff_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_check_before_takeoff_service_name_);
    action_takeoff_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_takeoff_service_name_);
    action_cruise_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_cruise_service_name_);
    action_land_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_land_service_name_);
    action_charge_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_charge_service_name_);
    action_data_collection_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_data_collection_service_name_);
    action_notify_http_post_land_workflow_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_http_post_land_workflow_service_name_);
    action_notify_waypoint_tracker_disable_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_waypoint_tracker_disable_service_name_);
    action_notify_uav_open_light_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_uav_open_light_service_name_);
    action_notify_uav_close_light_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_uav_close_light_service_name_);
    action_notify_uav_video_recording_start_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_uav_video_recording_start_service_name_);
    action_notify_uav_video_recording_stop_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_uav_video_recording_stop_service_name_);
    action_notify_uav_switch_video_mode_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_uav_switch_video_mode_service_name_);
    action_notify_uav_switch_photo_mode_client_ =
        nh_.serviceClient<std_srvs::Empty>(action_notify_uav_switch_photo_mode_service_name_);
}

bool ActionRequester::Call_Action_Await() {
    return CallActionService(action_await_client_, action_await_service_name_);
}

bool ActionRequester::Call_Action_CheckBeforeTakeOff() {
    return CallActionService(action_check_before_takeoff_client_, action_check_before_takeoff_service_name_);
}

bool ActionRequester::Call_Action_TakeOff() {
    return CallActionService(action_takeoff_client_, action_takeoff_service_name_);
}

bool ActionRequester::Call_Action_Cruise() {
    return CallActionService(action_cruise_client_, action_cruise_service_name_);
}

bool ActionRequester::Call_Action_Land() {
    return CallActionService(action_land_client_, action_land_service_name_);
}

bool ActionRequester::Call_Action_Charge() {
    return CallActionService(action_charge_client_, action_charge_service_name_);
}

bool ActionRequester::Call_Action_DataCollection() {
    return CallActionService(action_data_collection_client_, action_data_collection_service_name_);
}

bool ActionRequester::Call_Action_NotifyHttpPostLandWorkflow() {
    return CallActionService(action_notify_http_post_land_workflow_client_,
                             action_notify_http_post_land_workflow_service_name_);
}

bool ActionRequester::Call_Action_NotifyWaypointTrackerDisable()
{
    return CallActionService(action_notify_waypoint_tracker_disable_client_, action_notify_waypoint_tracker_disable_service_name_);
}

bool ActionRequester::Call_Action_NotifyUavOpenLight()
{
    return CallActionService(action_notify_uav_open_light_client_, action_notify_uav_open_light_service_name_);
}

bool ActionRequester::Call_Action_NotifyUavCloseLight()
{
    return CallActionService(action_notify_uav_close_light_client_, action_notify_uav_close_light_service_name_);
}

bool ActionRequester::Call_Action_NotifyUavVideoRecordingStart()
{
    return CallActionService(action_notify_uav_video_recording_start_client_, action_notify_uav_video_recording_start_service_name_);
}

bool ActionRequester::Call_Action_NotifyUavVideoRecordingStop()
{
    return CallActionService(action_notify_uav_video_recording_stop_client_, action_notify_uav_video_recording_stop_service_name_);
}

bool ActionRequester::Call_Action_NotifyUavSwitchVideoMode()
{
    return CallActionService(action_notify_uav_switch_video_mode_client_, action_notify_uav_switch_video_mode_service_name_);
}

bool ActionRequester::Call_Action_NotifyUavSwitchPhotoMode()
{
    return CallActionService(action_notify_uav_switch_photo_mode_client_, action_notify_uav_switch_photo_mode_service_name_);
}

bool ActionRequester::CallActionService(ros::ServiceClient &client, const std::string &service_name)
{

    ROS_INFO("Calling action service: [%s]", service_name.c_str());

    const bool service_exists =
        action_service_wait_timeout_sec_ > 0.0
        ? client.waitForExistence(ros::Duration(action_service_wait_timeout_sec_))
        : client.exists();

    if (!service_exists) {
        ROS_WARN_STREAM_THROTTLE(kServiceLogThrottleSec,
                                 "Action service [" << service_name
                                 << "] is not available. wait_timeout_sec="
                                 << action_service_wait_timeout_sec_);
        return false;
    }

    std_srvs::Empty service;
    if (!client.call(service)) {
        ROS_ERROR_STREAM_THROTTLE(kServiceLogThrottleSec, "Failed to call action service [" << service_name << "].");
        return false;
    }

    return true;
}
