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
    DataCollectionComplete, // 数据采集完成事件

    // ── 人工干预事件（遥控器下发） ─────────────────────────
    PauseCommand,           // 暂停：Cruise → Paused，无人机悬停并保留航点索引
    ResumeCommand,          // 继续：Paused → Cruise，从未完成的航点继续
    ReturnCommand,          // 返航：Paused → Return，反方向追踪航点回到第一个航点
    ReturnComplete          // 返航完成：Return → Land
};

#endif // EVENT_H