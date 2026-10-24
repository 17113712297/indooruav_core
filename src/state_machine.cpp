#include "indooruav_core/state_machine.h"

void  IndooruavStateMachine::HandleEvent(Event event) 
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
        case State::Paused:
            Handle_Paused(event);
            break;
        case State::Return:
            Handle_Return(event);
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
        case Event::CheckFailed:
            state_ = State::Await;
            Action_Await();
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
            Action_NotifyUavOpenLight();
            Action_NotifyUavSwitchVideoMode();
            Action_NotifyUavVideoRecordingStart();
            Action_SetGimbalAngle();  // 新增：起飞后设置云台角度
            Action_Cruise();
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Handle_Cruise(Event event)
{
    switch (event) {
        case Event::PauseCommand:
            // 暂停：只冻结追踪器（悬停 + 保留航点索引），不改变录像/云台状态
            state_ = State::Paused;
            Action_Paused();
            break;
        case Event::ReturnCommand:
            // 飞行中不允许直接返航：必须"先暂停、再返航"，
            // 否则返航的起点航点索引不可控，容易在半途掉头穿墙。
            std::cout << "[Reject] ReturnCommand in Cruise: 必须先暂停再返航\n";
            break;
        case Event::DataCollectionStart:
            state_ = State::DataCollection;
            Action_NotifyWaypointTrackerDisable(); //如果是脱离Cruise状态的事件（除Land事件以外），进行此处理：通知失能waypoint_tracker
            Action_NotifyUavVideoRecordingStart(); //进入数据采集状态后，开始录像
            Action_DataCollection();
            break;
        case Event::CruiseComplete:
            state_ = State::Land;
            // Action_NotifyUavCloseLight();
            Action_NotifyUavVideoRecordingStop();
            ros::Duration(0.5).sleep(); // 等待 stopRecord ACK 回来再切模式
            Action_NotifyUavSwitchPhotoMode();
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
        case Event::PauseCommand:
        case Event::ReturnCommand:
            // 数据采集期间不接管航点追踪（此时 tracker 已被 disable），
            // 人工干预请先结束采集回到 Cruise。
            std::cout << "[Reject] " << ToString(event)
                      << " in DataCollection: 请先结束数据采集\n";
            break;
        default:
            break;
    }
}

// 暂停态：悬停等待，可继续、可返航
void IndooruavStateMachine::Handle_Paused(Event event)
{
    switch (event) {
        case Event::ResumeCommand:
            // 继续：tracker 从暂停时保留的 current_index_ 接着飞
            state_ = State::Cruise;
            Action_Resume();
            break;
        case Event::ReturnCommand:
            // 返航：tracker 反向遍历航点回到第一个航点，到达后发 ReturnComplete
            state_ = State::Return;
            Action_Return();
            break;
        case Event::PauseCommand:
            // 幂等：重复暂停不改变状态
            std::cout << "[Ignore] PauseCommand in Paused: 已经处于暂停状态\n";
            break;
        case Event::CruiseComplete:
            // 暂停中理论上不会收到巡航完成事件，收到则忽略并提示
            std::cout << "[Ignore] CruiseComplete in Paused\n";
            break;
        default:
            break;
    }
}

// 返航态：等 tracker 反向到达第一个航点后切入降落
void IndooruavStateMachine::Handle_Return(Event event)
{
    switch (event) {
        case Event::ReturnComplete:
            state_ = State::Land;
            Action_NotifyUavVideoRecordingStop();
            ros::Duration(0.5).sleep(); // 等待 stopRecord ACK 回来再切模式
            Action_NotifyUavSwitchPhotoMode();
            Action_Land();
            break;
        case Event::PauseCommand:
            // 返航是终止流程，不支持中途暂停（要中断请直接接管/降落）
            std::cout << "[Reject] PauseCommand in Return: 返航过程中不支持暂停\n";
            break;
        case Event::ReturnCommand:
            std::cout << "[Ignore] ReturnCommand in Return: 已经在返航中\n";
            break;
        case Event::CruiseComplete:
            std::cout << "[Ignore] CruiseComplete in Return\n";
            break;
        default:
            break;
    }
}

void IndooruavStateMachine::Handle_Land(Event event)
{
    switch (event) {
        case Event::LandComplete:
            state_ = State::Await;
            Action_NotifyUavCloseLight(); //降落完成后，关闭补光灯
            // 这里在“降落完成并切入 Charge”时触发前端回传流程。
            // 该流程由 http 功能包异步执行，不阻塞状态机继续执行后续动作。
            Action_NotifyHttpPostLandWorkflow();
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

void IndooruavStateMachine::Action_Paused()
{
    action_request_.Call_Action_PauseWaypointTracker();
}

void IndooruavStateMachine::Action_Resume()
{
    action_request_.Call_Action_ResumeWaypointTracker();
}

void IndooruavStateMachine::Action_Return()
{
    action_request_.Call_Action_ReturnWaypointTracker();
}

void IndooruavStateMachine::Action_Land()
{
    action_request_.Call_Action_Land();
}

void IndooruavStateMachine::Action_Charge()
{
    action_request_.Call_Action_Charge();
}

void IndooruavStateMachine::Action_NotifyHttpPostLandWorkflow()
{
    action_request_.Call_Action_NotifyHttpPostLandWorkflow();
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

void IndooruavStateMachine::Action_NotifyUavSwitchVideoMode()
{
    action_request_.Call_Action_NotifyUavSwitchVideoMode();
}

void IndooruavStateMachine::Action_NotifyUavSwitchPhotoMode()
{
    action_request_.Call_Action_NotifyUavSwitchPhotoMode();
}

void IndooruavStateMachine::Action_SetGimbalAngle()
{
    action_request_.Call_Action_SetGimbalAngle();
}
