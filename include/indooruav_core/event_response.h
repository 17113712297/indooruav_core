#ifndef EVENT_RESPONSE_H
#define EVENT_RESPONSE_H

#include <string>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <indooruav_core/state_machine.h>

class EventResponder {

public:
    EventResponder();
    ~EventResponder();

private:
    void LoadParameters();
    void InitializeServers();
    
    bool HandleTakeoffCommand(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HandleCheckPassed(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HandleTakeoffComplete(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HandleCruiseComplete(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HandleLandComplete(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HandleChargeComplete(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HandleDataCollectionStart(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool HandleDataCollectionComplete(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

private:
    IndooruavStateMachine state_machine_;

    std::string event_takeoff_command_service_name_;
    std::string event_check_passed_service_name_;
    std::string event_takeoff_complete_service_name_;
    std::string event_cruise_complete_service_name_;
    std::string event_land_complete_service_name_;
    std::string event_charge_complete_service_name_;
    std::string event_data_collection_start_service_name_;
    std::string event_data_collection_complete_service_name_;

    ros::NodeHandle nh_;
    ros::ServiceServer event_takeoff_command_server_;
    ros::ServiceServer event_check_passed_server_;
    ros::ServiceServer event_takeoff_complete_server_;
    ros::ServiceServer event_cruise_complete_server_;
    ros::ServiceServer event_land_complete_server_; 
    ros::ServiceServer event_charge_complete_server_;
    ros::ServiceServer event_data_collection_start_server_;
    ros::ServiceServer event_data_collection_complete_server_;
};


#endif // EVENT_RESPONSE_H
