#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <iostream>
#include <string>

#include "indooruav_core/action_request.h"
#include "indooruav_core/event.h"

enum class State {
    Await,                  // 待机状态          
    CheckBeforeTakeOff,     // 自检状态
    TakeOff,                // 起飞状态
    Cruise,                 // 巡航状态
    DataCollection,         // 数据采集状态
    Land,                   // 降落状态
    Charge                  // 充电状态
};

inline std::string ToString(State state) {
    switch (state) {
        case State::Await:                  return "Await";
        case State::CheckBeforeTakeOff:     return "CheckBeforeTakeOff";
        case State::TakeOff:                return "TakeOff";
        case State::Cruise:                 return "Cruise";
        case State::DataCollection:         return "DataCollection";
        case State::Land:                   return "Land";
        case State::Charge:                 return "Charge";
        default:                            return "Unknown";
    }
}

inline std::string ToString(Event event) {
    switch (event) {
        case Event::TakeoffCommand:         return "TakeoffCommand";
        case Event::CheckPassed:            return "CheckPassed";
        case Event::TakeoffComplete:        return "TakeoffComplete";
        case Event::CruiseComplete:         return "CruiseComplete";
        case Event::LandComplete:           return "LandComplete";
        case Event::ChargeComplete:         return "ChargeComplete";
        case Event::DataCollectionStart:    return "DataCollectionStart";
        case Event::DataCollectionComplete: return "DataCollectionComplete";
        case Event::CheckFailed:            return "CheckFailed";
        default:                            return "Unknown";
    }
}

class IndooruavStateMachine {

public:
    IndooruavStateMachine() : state_(State::Await) {}
    void HandleEvent(Event event); 

private:    
    State state_;
    ActionRequester action_request_;

private:
    void Handle_Await(Event event);
    void Handle_CheckBeforeTakeOff(Event event);
    void Handle_TakeOff(Event event);
    void Handle_Cruise(Event event);
    void Handle_DataCollection(Event event);
    void Handle_Land(Event event);
    void Handle_Charge(Event event);

    void Action_Await();
    void Action_CheckBeforeTakeOff();
    void Action_TakeOff();
    void Action_Cruise();
    void Action_DataCollection();
    void Action_Land();
    void Action_Charge();

    void Action_NotifyWaypointTrackerDisable();
    
    void Action_NotifyUavOpenLight();
    void Action_NotifyUavCloseLight();
    
    void Action_NotifyUavVideoRecordingStart();
    void Action_NotifyUavVideoRecordingStop();
};

#endif //STATE_MACHINE_H
