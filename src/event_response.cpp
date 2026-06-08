#include "indooruav_core/event_response.h"
#include "indooruav_core/event.h"

EventResponder::EventResponder() {
    LoadParameters();
    InitializeServers();
}

EventResponder::~EventResponder() {
    // No dynamic memory to clean up.
}

void EventResponder::LoadParameters() {
    nh_.param<std::string>("/indooruav_core/state_machine_event/takeoff_command",
                           event_takeoff_command_service_name_,
                           "indooruav_core/state_machine_event/takeoff_command");
    nh_.param<std::string>("/indooruav_core/state_machine_event/check_passed",
                           event_check_passed_service_name_,
                           "indooruav_core/state_machine_event/check_passed");
    nh_.param<std::string>("/indooruav_core/state_machine_event/takeoff_complete",
                           event_takeoff_complete_service_name_,
                           "indooruav_core/state_machine_event/takeoff_complete");
    nh_.param<std::string>("/indooruav_core/state_machine_event/cruise_complete",
                           event_cruise_complete_service_name_,
                           "indooruav_core/state_machine_event/cruise_complete");
    nh_.param<std::string>("/indooruav_core/state_machine_event/land_complete",
                           event_land_complete_service_name_,
                           "indooruav_core/state_machine_event/land_complete");
    nh_.param<std::string>("/indooruav_core/state_machine_event/charge_complete",
                           event_charge_complete_service_name_,
                           "indooruav_core/state_machine_event/charge_complete");
    nh_.param<std::string>("/indooruav_core/state_machine_event/data_collection_start",
                           event_data_collection_start_service_name_,
                           "indooruav_core/state_machine_event/data_collection_start");
    nh_.param<std::string>("/indooruav_core/state_machine_event/data_collection_complete",
                           event_data_collection_complete_service_name_,
                           "indooruav_core/state_machine_event/data_collection_complete");
    nh_.param<std::string>("/indooruav_core/state_machine_event/check_failed",
                           event_check_failed_service_name_,
                           "indooruav_core/state_machine_event/check_failed");
}

void EventResponder::InitializeServers() {
    event_takeoff_command_server_ =
        nh_.advertiseService(event_takeoff_command_service_name_,
                             &EventResponder::HandleTakeoffCommand,
                             this);
    event_check_passed_server_ =
        nh_.advertiseService(event_check_passed_service_name_,
                             &EventResponder::HandleCheckPassed,
                             this);
    event_takeoff_complete_server_ =
        nh_.advertiseService(event_takeoff_complete_service_name_,
                             &EventResponder::HandleTakeoffComplete,
                             this);
    event_cruise_complete_server_ =
        nh_.advertiseService(event_cruise_complete_service_name_,
                             &EventResponder::HandleCruiseComplete,
                             this);
    event_land_complete_server_ =
        nh_.advertiseService(event_land_complete_service_name_,
                             &EventResponder::HandleLandComplete,
                             this);
    event_charge_complete_server_ =
        nh_.advertiseService(event_charge_complete_service_name_,
                             &EventResponder::HandleChargeComplete,
                             this);
    event_data_collection_start_server_ =
        nh_.advertiseService(event_data_collection_start_service_name_,
                             &EventResponder::HandleDataCollectionStart,
                             this);
    event_data_collection_complete_server_ =
        nh_.advertiseService(event_data_collection_complete_service_name_,
                             &EventResponder::HandleDataCollectionComplete,
                             this);
    event_check_failed_server_ =
        nh_.advertiseService(event_check_failed_service_name_,
                             &EventResponder::HandleCheckFailed,
                             this);
}

bool EventResponder::HandleTakeoffCommand(std_srvs::Empty::Request& request,
                                          std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement takeoff command event handling.
    state_machine_.HandleEvent(Event::TakeoffCommand);
    return true;
}

bool EventResponder::HandleCheckPassed(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement check passed event handling.
    state_machine_.HandleEvent(Event::CheckPassed);
    return true;
}

bool EventResponder::HandleTakeoffComplete(std_srvs::Empty::Request& request,
                                           std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement takeoff complete event handling.
    state_machine_.HandleEvent(Event::TakeoffComplete);
    return true;
}

bool EventResponder::HandleCruiseComplete(std_srvs::Empty::Request& request,
                                          std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement cruise complete event handling.
    state_machine_.HandleEvent(Event::CruiseComplete);
    return true;
}

bool EventResponder::HandleLandComplete(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement land complete event handling.
    state_machine_.HandleEvent(Event::LandComplete);
    return true;
}

bool EventResponder::HandleChargeComplete(std_srvs::Empty::Request& request,
                                          std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement charge complete event handling.
    state_machine_.HandleEvent(Event::ChargeComplete);
    return true;
}

bool EventResponder::HandleDataCollectionStart(std_srvs::Empty::Request& request,
                                               std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement data collection start event handling.
    state_machine_.HandleEvent(Event::DataCollectionStart);
    return true;
}

bool EventResponder::HandleDataCollectionComplete(std_srvs::Empty::Request& request,
                                                  std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    // TODO: implement data collection complete event handling.
    state_machine_.HandleEvent(Event::DataCollectionComplete);
    return true;
}

bool EventResponder::HandleCheckFailed(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response) {
    (void)request;
    (void)response;
    state_machine_.HandleEvent(Event::CheckFailed);
    return true;
}
