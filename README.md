# indooruav_core

## 项目简介

`indooruav_core` 是一个基于 ROS Service 的巡检任务状态机核心包。

这个包本身不直接执行无人机动作，而是完成两件事：

1. 对外提供一组“事件服务（event services）”，用于接收任务过程中的关键事件。
2. 在状态迁移后调用对应的“动作服务（action services）”，把下一步该做什么通知给下游模块。

从代码实现上看，它更像是一个“任务编排器”或“流程控制核心”：

- 上游模块负责触发事件，例如“起飞指令到达”“自检通过”“起飞完成”。
- 本包负责维护当前状态，并在状态变化后调用下一步动作服务。
- 下游模块负责真正实现动作，例如自检、起飞、巡航、降落、充电。

## 核心流程

当前实现包含 7 个状态：

- `Await`
- `CheckBeforeTakeOff`
- `TakeOff`
- `Cruise`
- `DataCollection`
- `Land`
- `Charge`

当前实现包含 8 个事件：

- `TakeoffCommand`
- `CheckPassed`
- `TakeoffComplete`
- `CruiseComplete`
- `LandComplete`
- `ChargeComplete`
- `DataCollectionStart`
- `DataCollectionComplete`

状态迁移关系如下：

```text
Await
  --TakeoffCommand--> CheckBeforeTakeOff
  --call action-->    check_before_takeoff

CheckBeforeTakeOff
  --CheckPassed-->    TakeOff
  --call action-->    takeoff

TakeOff
  --TakeoffComplete--> Cruise
  --call action-->     notify_uav_open_light
  --call action-->     notify_uav_switch_video_mode
  --call action-->     notify_uav_video_recording_start
  --call action-->     set_gimbal_angle
  --call action-->     cruise

Cruise
  --DataCollectionStart--> DataCollection
  --call action-->         notify_waypoint_tracker_disable
  --call action-->         notify_uav_video_recording_start
  --call action-->         data_collection
  --CruiseComplete-->      Land
  --call action-->         land

DataCollection
  --DataCollectionComplete--> Cruise
  --call action-->            notify_uav_video_recording_stop
  --call action-->            cruise

Land
  --LandComplete-->   Charge
  --call action-->    notify_uav_close_light
  --call action-->    charge

Charge
  --ChargeComplete--> Await
  --call action-->    await
```

也可以理解为一条完整任务链：

```text
待机 -> 起飞前自检 -> 起飞 -> 巡航 -> 数据采集 -> 巡航 -> 降落 -> 充电 -> 回到待机
```

## 节点说明

### 1. 主节点

- 节点名：`indooruav_core`
- 可执行文件：`indooruav_core_node`
- launch 文件：`launch/bringup_indooruav_core.launch`

主节点启动后会：

- 加载 `config/config.yaml`
- 创建事件服务端
- 在收到事件服务请求后驱动内部状态机
- 状态迁移成功后调用对应动作服务

### 2. 测试节点

- 节点名：`test_state_machine`
- 可执行文件：`test_state_machine`

测试节点用于联调状态机逻辑，支持两种模式：

- 手动输入命令，逐步触发事件
- 自动触发完整事件序列

如果当前系统里没有真实的动作服务，测试节点默认还会自动挂一组 dummy action service，方便单独验证状态机流程。

## 接口说明

### 事件服务

主节点对外提供以下事件服务，服务类型均为 `std_srvs/Empty`：

| 事件 | 默认服务名 | 作用 |
| --- | --- | --- |
| `TakeoffCommand` | `indooruav_core/state_machine_event/takeoff_command` | 从 `Await` 进入 `CheckBeforeTakeOff` |
| `CheckPassed` | `indooruav_core/state_machine_event/check_passed` | 从 `CheckBeforeTakeOff` 进入 `TakeOff` |
| `TakeoffComplete` | `indooruav_core/state_machine_event/takeoff_complete` | 从 `TakeOff` 进入 `Cruise` |
| `CruiseComplete` | `indooruav_core/state_machine_event/cruise_complete` | 从 `Cruise` 进入 `Land` |
| `LandComplete` | `indooruav_core/state_machine_event/land_complete` | 从 `Land` 进入 `Charge` |
| `ChargeComplete` | `indooruav_core/state_machine_event/charge_complete` | 从 `Charge` 回到 `Await` |
| `DataCollectionStart` | `indooruav_core/state_machine_event/data_collection_start` | 从 `Cruise` 进入 `DataCollection` |
| `DataCollectionComplete` | `indooruav_core/state_machine_event/data_collection_complete` | 从 `DataCollection` 回到 `Cruise` |

### 动作服务

状态机在发生状态迁移后，会调用以下动作服务，服务类型同样为 `std_srvs/Empty`：

| 状态迁移后进入的状态 | 调用的动作服务 | 默认服务名 |
| --- | --- | --- |
| `CheckBeforeTakeOff` | `check_before_takeoff` | `indooruav_core/action/check_before_takeoff` |
| `TakeOff` | `takeoff` | `indooruav_controller/controller_hardware/takeoff` |
| `Cruise` | `cruise` | `indooruav_controller/waypoint_tracker/start` |
| `DataCollection` | `data_collection` | `indooruav_core/action/data_collection` |
| `Land` | `land` | `indooruav_controller/controller_hardware/land` |
| `Charge` | `charge` | `indooruav_core/action/charge` |
| `Await` | `await` | `indooruav_core/action/await` |

另外，状态机还会在特定迁移过程中调用这些通知型动作服务：

| 触发时机 | 调用的动作服务 | 默认服务名 |
| --- | --- | --- |
| `Cruise -> DataCollection` | `notify_waypoint_tracker_disable` | `indooruav_controller/waypoint_tracker/stop` |
| `Cruise -> DataCollection` | `notify_uav_video_recording_start` | `indooruav_controller/controller_hardware/camera_video_start` |
| `DataCollection -> Cruise` | `notify_uav_video_recording_stop` | `indooruav_controller/controller_hardware/camera_video_stop` |
| `TakeOff -> Cruise` | `notify_uav_open_light` | `indooruav_controller/controller_hardware/light_open` |
| `TakeOff -> Cruise` | `notify_uav_switch_video_mode` | `indooruav_controller/controller_hardware/camera_mode_video` |
| `TakeOff -> Cruise` | `notify_uav_video_recording_start` | `indooruav_controller/controller_hardware/camera_video_start` |
| `TakeOff -> Cruise` | `set_gimbal_angle` | `indooruav_controller/controller_hardware/gimbal_angle` |
| `Land -> Charge` | `notify_uav_close_light` | `indooruav_controller/controller_hardware/light_close` |

## 参数说明

### 主节点参数

主节点通过 `config/config.yaml` 加载以下参数：

```yaml
indooruav_core:
  event:
    takeoff_command: "indooruav_core/state_machine_event/takeoff_command"
    check_passed: "indooruav_core/state_machine_event/check_passed"
    takeoff_complete: "indooruav_core/state_machine_event/takeoff_complete"
    cruise_complete: "indooruav_core/state_machine_event/cruise_complete"
    land_complete: "indooruav_core/state_machine_event/land_complete"
    charge_complete: "indooruav_core/state_machine_event/charge_complete"
    data_collection_start: "indooruav_core/state_machine_event/data_collection_start"
    data_collection_complete: "indooruav_core/state_machine_event/data_collection_complete"

  action:
    wait_timeout_sec: 1.0
    await: "indooruav_core/action/await"
    check_before_takeoff: "indooruav_core/action/check_before_takeoff"
    takeoff: "indooruav_controller/controller_hardware/takeoff"
    cruise: "indooruav_controller/waypoint_tracker/start"
    land: "indooruav_controller/controller_hardware/land"
    charge: "indooruav_core/action/charge"
    data_collection: "indooruav_core/action/data_collection"
    notify_waypoint_tracker_disable: "indooruav_controller/waypoint_tracker/stop"
    notify_uav_open_light: "indooruav_controller/controller_hardware/light_open"
    notify_uav_close_light: "indooruav_controller/controller_hardware/light_close"
    notify_uav_video_recording_start: "indooruav_controller/controller_hardware/camera_video_start"
    notify_uav_video_recording_stop: "indooruav_controller/controller_hardware/camera_video_stop"
    set_gimbal_angle: "indooruav_controller/controller_hardware/gimbal_angle"

    # 起飞完成后设置云台角度的参数
    gimbal_angle_after_takeoff:
      mode: 0          # 0=绝对角度, 1=相对角度
      pitch: -60.0     # 俯仰角，单位：度
      roll: 0.0        # 横滚角，单位：度
      yaw: 0.0         # 偏航角，单位：度
      duration: 3.0    # 云台运动时间，单位：秒（必须 > 0）
```

其中：

- `event/*`：主节点对外提供的事件服务名。
- `action/*`：状态迁移后要调用的下游动作服务名。
- `action/wait_timeout_sec`：等待动作服务存在的超时时间，单位为秒。

说明：

- 当 `wait_timeout_sec > 0` 时，调用动作服务前会等待服务出现。
- 当 `wait_timeout_sec <= 0` 时，不等待，直接按 `client.exists()` 检查。
- 如果该参数被配置为负数，代码会回退到默认值 `1.0` 秒。
- `gimbal_angle_after_takeoff` 参数用于起飞完成后自动设置云台角度：
  - `mode`：云台角度模式，`0` 为绝对角度（默认），`1` 为相对角度。非法值会回退到 `0`。
  - `pitch`：俯仰角，单位为度，默认 `-60.0`。
  - `roll`：横滚角，单位为度，默认 `0.0`。
  - `yaw`：偏航角，单位为度，默认 `0.0`。
  - `duration`：云台运动到目标角度所需时间，单位为秒，默认 `3.0`。必须大于 `0`，否则回退到默认值。
  - `set_gimbal_angle` 动作服务使用的类型为 `indooruav_msgs/GimbalAngle`，与其他 `std_srvs/Empty` 类型的动作服务不同。

### 测试节点私有参数

测试节点还支持以下私有参数：

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `~event_service_wait_timeout_sec` | `2.0` | 等待事件服务可用的超时时间 |
| `~auto_step_interval_sec` | `1.0` | 自动测试模式下相邻两步事件之间的时间间隔 |
| `~advertise_dummy_action_services` | `true` | 若系统中不存在真实动作服务，则是否自动创建 dummy action service |

## 构建方式

在工作空间根目录执行：

```bash
cd /home/lxy/indooruav_ws
catkin_make --pkg indooruav_core
source devel/setup.bash
```

## 运行方式

### 1. 启动主节点

```bash
roslaunch indooruav_core bringup_indooruav_core.launch
```

### 2. 启动测试节点

如果你只是想快速验证状态机流程，可以在另一个终端启动测试节点：

```bash
rosrun indooruav_core test_state_machine
```

如果你已经有真实动作服务，不希望测试节点创建 dummy service，可以这样启动：

```bash
rosrun indooruav_core test_state_machine _advertise_dummy_action_services:=false
```

## 测试方法

### 1. 交互式测试

运行测试节点后，可以在终端输入以下命令：

| 输入命令 | 含义 |
| --- | --- |
| `1` 或 `takeoff_command` | 触发 `TakeoffCommand` |
| `2` 或 `check_passed` | 触发 `CheckPassed` |
| `3` 或 `takeoff_complete` | 触发 `TakeoffComplete` |
| `4` 或 `cruise_complete` | 触发 `CruiseComplete` |
| `5` 或 `land_complete` | 触发 `LandComplete` |
| `6` 或 `charge_complete` | 触发 `ChargeComplete` |
| `7` 或 `data_collection_start` | 触发 `DataCollectionStart` |
| `8` 或 `data_collection_complete` | 触发 `DataCollectionComplete` |
| `a` 或 `auto` | 自动执行完整事件链一次 |
| `h` 或 `help` | 打印帮助信息 |
| `q` 或 `quit` | 退出测试节点 |

推荐先按顺序执行一遍：

```text
1 -> 2 -> 3 -> 7 -> 8 -> 4 -> 5 -> 6
```

这样可以完整验证一次状态流转。

### 2. 使用 `rosservice` 手动触发事件

也可以不用测试节点，直接调用事件服务：

```bash
rosservice call /indooruav_core/state_machine_event/takeoff_command
rosservice call /indooruav_core/state_machine_event/check_passed
rosservice call /indooruav_core/state_machine_event/takeoff_complete
rosservice call /indooruav_core/state_machine_event/data_collection_start
rosservice call /indooruav_core/state_machine_event/data_collection_complete
rosservice call /indooruav_core/state_machine_event/cruise_complete
rosservice call /indooruav_core/state_machine_event/land_complete
rosservice call /indooruav_core/state_machine_event/charge_complete
```

## 代码结构

```text
indooruav_core/
├── config/
│   └── config.yaml                    # 服务名与超时参数
├── include/indooruav_core/
│   ├── action_request.h               # 动作服务客户端封装
│   ├── event.h                        # 事件枚举
│   ├── event_response.h               # 事件服务端封装
│   └── state_machine.h                # 状态机定义
├── launch/
│   └── bringup_indooruav_core.launch
├── node/
│   └── indooruav_core_node.cpp
├── src/
│   ├── action_request.cpp             # 调用动作服务
│   ├── event_response.cpp             # 响应事件服务
│   └── state_machine.cpp              # 状态迁移逻辑
└── scripts/
    └── test_state_machine.cpp    # 交互式测试节点
```

## 当前实现特点与注意事项

为了便于后续联调，这里特别说明几个与当前代码实现一致的行为：

1. 初始状态是 `Await`

- 状态机对象构造后默认处于 `Await`。
- 启动时不会主动调用 `await` 动作服务。
- 只有在收到 `ChargeComplete` 并从 `Charge` 返回 `Await` 时，才会调用一次 `await` 动作服务。

2. 非法事件会被忽略

- 例如在 `Await` 状态下收到 `CruiseComplete`，当前实现不会报错，也不会迁移状态。
- 这类情况目前没有额外告警日志。

3. 状态切换先发生，再调用动作服务

- 当前代码的顺序是：先修改内部状态，再调用对应动作服务。
- 如果动作服务调用失败，状态不会自动回滚。
- 联调时如果发现状态已经前进但下游动作没有执行，这通常是需要重点关注的地方。

4. 所有接口都使用 `std_srvs/Empty`

- 当前版本的事件服务和动作服务都没有请求体和响应体载荷。
- 如果后续需要携带任务 ID、错误码、位姿、检查结果等信息，可能需要扩展为自定义 srv/msg。

5. 事件服务当前不表达业务失败

- 事件服务回调函数当前都会直接返回 `true`。
- 也就是说，只要 service callback 被正常执行，调用方通常会看到服务调用成功。
- 即使该事件在当前状态下被忽略，或者后续动作服务调用失败，现阶段也不会通过服务响应显式反馈给上游。

## 适用场景

这个包适合放在巡检任务系统的“流程控制层”，常见用法包括：

- 上层调度系统通过事件服务驱动任务推进
- 下游飞控或业务模块通过动作服务执行具体动作
- 开发阶段先用测试节点和 dummy action service 验证状态机逻辑

如果后续要继续扩展，比较自然的方向包括：

- 增加异常事件和错误恢复分支
- 为非法事件补充日志或拒绝响应
- 为动作调用失败增加回滚、重试或超时处理
- 把 `std_srvs/Empty` 升级为带业务字段的自定义接口
