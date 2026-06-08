#ifndef ACTION_REQUEST_H
#define ACTION_REQUEST_H

#include <string>

#include <ros/ros.h>

class ActionRequester {

public:
    ActionRequester();
    ~ActionRequester();
    
    // 切入状态后可以调用这些action
    bool Call_Action_Await();
    bool Call_Action_CheckBeforeTakeOff();
    bool Call_Action_TakeOff();
    bool Call_Action_Cruise();
    bool Call_Action_Land();
    bool Call_Action_Charge();
    bool Call_Action_DataCollection();

    // 从cruise切走时会调用Disable
    bool Call_Action_NotifyWaypointTrackerDisable();
    
    // 请求无人机开启补光灯
    bool Call_Action_NotifyUavOpenLight();
    
    // 请求无人机关闭补光灯
    bool Call_Action_NotifyUavCloseLight();
    
    // 请求无人机开始录像
    bool Call_Action_NotifyUavVideoRecordingStart();
    
    // 请求无人机停止录像
    bool Call_Action_NotifyUavVideoRecordingStop();

    // 请求无人机切换到视频模式
    bool Call_Action_NotifyUavSwitchVideoMode();

    // 请求无人机切换到拍照模式
    bool Call_Action_NotifyUavSwitchPhotoMode();
private:
    void LoadParameters();
    void InitializeClients();
    bool CallActionService(ros::ServiceClient& client, const std::string& service_name);

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
    std::string action_notify_uav_switch_video_mode_service_name_;
    std::string action_notify_uav_switch_photo_mode_service_name_;
    double action_service_wait_timeout_sec_; 

    ros::NodeHandle nh_;
    ros::ServiceClient action_await_client_;
    ros::ServiceClient action_check_before_takeoff_client_;
    ros::ServiceClient action_takeoff_client_;
    ros::ServiceClient action_cruise_client_;
    ros::ServiceClient action_land_client_;
    ros::ServiceClient action_charge_client_;
    ros::ServiceClient action_data_collection_client_;  
    ros::ServiceClient action_notify_waypoint_tracker_disable_client_;
    ros::ServiceClient action_notify_uav_open_light_client_;
    ros::ServiceClient action_notify_uav_close_light_client_;
    ros::ServiceClient action_notify_uav_video_recording_start_client_;
    ros::ServiceClient action_notify_uav_video_recording_stop_client_;
    ros::ServiceClient action_notify_uav_switch_video_mode_client_;
    ros::ServiceClient action_notify_uav_switch_photo_mode_client_;
};


#endif // ACTION_REQUEST_H
