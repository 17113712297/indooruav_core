#ifndef EVENT_H
#define EVENT_H

enum class Event : int {
    TakeoffCommand,         // 起飞指令事件   
    CheckPassed,            // 自检通过事件
    CheckFailed,            // 自检失败事件
    TakeoffComplete,        // 起飞完成事件
    CruiseComplete,         // 巡航完成事件
    LandComplete,           // 降落完成事件
    ChargeComplete,         // 充电完成事件
    DataCollectionStart,    // 数据采集开始事件
    DataCollectionComplete  // 数据采集完成事件
};

#endif // EVENT_H