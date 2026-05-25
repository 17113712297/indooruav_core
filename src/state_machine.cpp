#include "indooruav_core/state_machine.h"

void IndooruavStateMachine::HandleEvent(Event event) 
{
    std::cout << "\n[Event] " << ToString(event) << ", [Current State] " << ToString(this->state_) << "\n";

    switch(this->state_) {
        case State::Await:
            Handle_Await(event);
            break;
        case State::CheckBeforeTakeOff:
            Handle_CheckBeforeTakeOff(event);
            break; 
        case State::TakeOff:
            Handle_TakeOff(event);
            break;
        case State::Cruise:
            Handle_Cruise(event);
            break;
        case State::DataCollection:
            Handle_DataCollection(event);
            break;
        case State::Land:
            Handle_Land(event);
            break;
        case State::Charge:
            Handle_Charge(event);
            break;
    }

    std::cout << "[State] " << ToString(state_) << "\n";
}

void IndooruavStateMachine::Handle_Await(Event event)
{
    switch (event) {
        case Event::TakeoffCommand:
            state_ = State::CheckBeforeTakeOff;
            Action_CheckBeforeTakeOff();
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Handle_CheckBeforeTakeOff(Event event)
{
    switch (event) {
        case Event::CheckPassed:
            state_ = State::TakeOff;
            Action_TakeOff();
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Handle_TakeOff(Event event)
{
    switch (event) {
        case Event::TakeoffComplete:
            state_ = State::Cruise;
            Action_NotifyUavOpenLight(); //起飞完成后，开启补光灯
            Action_Cruise();
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Handle_Cruise(Event event)
{
    switch (event) {
        case Event::DataCollectionStart:
            state_ = State::DataCollection;
            Action_NotifyWaypointTrackerDisable(); //如果是脱离Cruise状态的事件（除Land事件以外），进行此处理：通知失能waypoint_tracker
            Action_NotifyUavVideoRecordingStart(); //进入数据采集状态后，开始录像
            Action_DataCollection();
            break;
        case Event::CruiseComplete:
            state_ = State::Land;
            Action_Land();
            break;
        default:
            break;
    }
    

}

void IndooruavStateMachine::Handle_DataCollection(Event event)
{
    switch (event) {
        case Event::DataCollectionComplete:
            state_ = State::Cruise;
            Action_NotifyUavVideoRecordingStop(); //离开数据采集状态后，停止录像
            Action_Cruise();
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Handle_Land(Event event)
{
    switch (event) {
        case Event::LandComplete:
            state_ = State::Charge;
            Action_NotifyUavCloseLight(); //降落完成后，关闭补光灯
            Action_Charge();
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Handle_Charge(Event event)
{
    switch (event) {
        case Event::ChargeComplete:
            state_ = State::Await;
            Action_Await();
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Action_Await()
{
    action_request_.Call_Action_Await();
}

void IndooruavStateMachine::Action_CheckBeforeTakeOff()
{
    action_request_.Call_Action_CheckBeforeTakeOff();
}

void IndooruavStateMachine::Action_TakeOff()
{
    action_request_.Call_Action_TakeOff();
}

void IndooruavStateMachine::Action_Cruise()
{
    action_request_.Call_Action_Cruise();
}

void IndooruavStateMachine::Action_DataCollection()
{
    action_request_.Call_Action_DataCollection();
}

void IndooruavStateMachine::Action_Land()
{
    action_request_.Call_Action_Land();
}

void IndooruavStateMachine::Action_Charge()
{
    action_request_.Call_Action_Charge();
}

void IndooruavStateMachine::Action_NotifyWaypointTrackerDisable()
{
    action_request_.Call_Action_NotifyWaypointTrackerDisable();
}

void IndooruavStateMachine::Action_NotifyUavOpenLight()
{
    action_request_.Call_Action_NotifyUavOpenLight();
}

void IndooruavStateMachine::Action_NotifyUavCloseLight()
{
    action_request_.Call_Action_NotifyUavCloseLight();
}

void IndooruavStateMachine::Action_NotifyUavVideoRecordingStart()
{
    action_request_.Call_Action_NotifyUavVideoRecordingStart();
}

void IndooruavStateMachine::Action_NotifyUavVideoRecordingStop()
{
    action_request_.Call_Action_NotifyUavVideoRecordingStop();
}
