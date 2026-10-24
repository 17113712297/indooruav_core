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
    Paused,                 // 暂停状态（悬停，保留航点索引与方向）
    Return,                 // 返航状态（反方向追踪航点回到第一个航点）
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
        case State::Paused:                 return "Paused";
        case State::Return:                 return "Return";
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
        case Event::PauseCommand:           return "PauseCommand";
        case Event::ResumeCommand:          return "ResumeCommand";
        case Event::ReturnCommand:          return "ReturnCommand";
        case Event::ReturnComplete:         return "ReturnComplete";
        default:                            return "Unknown";
    }
}

class IndooruavStateMachine {

public:
    IndooruavStateMachine() : state_(State::Await) {}
    void HandleEvent(Event event);
    State GetState() const { return state_; }

private:
    State state_;
    ActionRequester action_request_;

private:
    void Handle_Await(Event event);
    void Handle_CheckBeforeTakeOff(Event event);
    void Handle_TakeOff(Event event);
    void Handle_Cruise(Event event);
    void Handle_DataCollection(Event event);
    void Handle_Paused(Event event);
    void Handle_Return(Event event);
    void Handle_Land(Event event);
    void Handle_Charge(Event event);

    void Action_Await();
    void Action_CheckBeforeTakeOff();
    void Action_TakeOff();
    void Action_Cruise();
    void Action_DataCollection();
    void Action_Paused();
    void Action_Resume();
    void Action_Return();
    void Action_Land();
    void Action_Charge();
    void Action_NotifyHttpPostLandWorkflow();

    void Action_NotifyWaypointTrackerDisable();
    
    void Action_NotifyUavOpenLight();
    void Action_NotifyUavCloseLight();
    
    void Action_NotifyUavVideoRecordingStart();
    void Action_NotifyUavVideoRecordingStop();

    void Action_NotifyUavSwitchVideoMode();
    void Action_NotifyUavSwitchPhotoMode();

    void Action_SetGimbalAngle();
};

#endif //STATE_MACHINE_H
