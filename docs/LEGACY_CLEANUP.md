# 旧阿克曼逻辑清理记录（2026-10-05）

本次按用户“尽量剔除旧阿克曼逻辑”的指令，删除 74 个旧文件。
完整逐文件清单见 [legacy_removed_files.txt](legacy_removed_files.txt)。
此前的任务板识别改动和用户已有报警 service/工具改动均保留。
工作区修改尚未提交或推送；当前 HEAD 为
`81d67fb470b0bde6e90a3a796f497ae3f8bff087`。

## 删除范围

| 范围 | 数量 | 内容 |
|---|---:|---|
| 旧任务入口/组件 | 14 | task1/task2、AckermannDrive、前轮舵机、旧 Hybrid A*/Pure Pursuit 导航、赛道运行时/视觉、坐标导航/可信地图/网格救援 |
| 旧链专用单测 | 18 | 对应旧任务、转向、视觉、导航、v1 配置和屏幕启动器测试文件 |
| v1 配置/启动器 | 10 | v1 loader/models/factory、旧运行时地标配置、两份旧车辆 profile、串口屏启动器和两个服务模板 |
| ROS2 阿克曼输出/部署 | 18 | 旧 base/mission bridge、Twist→舵角转换及其测试、Nav2 启动/参数/行为树、旧轮廓和部署脚本 |
| 旧探针/历史使用文档 | 14 | 旧雷达点导航、孤立网格测试、转向/赛道视觉/前进探针、PWM 部署工具和旧指南 |

`components/__init__.py` 已去掉被删除模块的 lazy exports。
`config/__init__.py` 仅提供 schema-v2 exports，`NavigationConfig` 指向现行 SI 模型。
v2 loader 遇到 v1 文件明确要求迁移，不再提示调用已删除的 loader。
共享默认值仅保留 C10B 固件轨距和最小转弯半径。

通信协议 `navigation_protocol.py` 使用独立 `NavigationCommandGoal` DTO，
保持厘米/逆时针度的无线格式、HMAC、去重和停止命令；解除对旧导航实现的依赖。
FleetBus 的协议、节点、队列、轨迹及只读状态适配器保留，其测试不再构造旧应用。

## 保留范围及原因

- `rear_motor.py`、`c10b_diff_backend.py`：差速运行时仍使用的底层串口协议、
  发送线程、watchdog 和安全停车。两者的实现未因本次清理修改。
- `ackermann_firmware_compat`：已安装 C10B 固件的真实能力适配，不是旧任务策略。
  不能只删除名字就改用未经验证的差速协议。
- differential_*、basic_motion_controller、pose_fusion、D500、T265、硬件锁、
  电池、GPIO/PWM HAL、声光报警和继电器：现行/共享资产。
- 新任务板组件、开局集成及 39 项测试：全部保留。
- ROS2 D500、坐标转换、场地几何和地图生成：通用工具仍可复用。
  hardware.launch.py 只启动 D500，use_hardware 默认 false，不再打开底盘。
- MIGRATION_BASELINE.md：明确标为历史审计，保留曾经的架构和验证依据。
- 未关联转向的旧报警探针、无线和电机底层工具：未按目录名机械删除。

本次没有将 ROS2 Nav2 迁移成差速控制。其旧运动输出链已删除，现行小车运动入口
统一为 main_robocup.py。未来接入 Nav2 需独立实现并验证差速桥。

## 验证

- 清理前基线：790 项测试，8 项旧链跳过。
- 清理后：532 项默认单测全部通过，无跳过。
  减少的 260 项来自旧链测试文件及一项旧应用快照测试；增加了两项导出/传感器启动边界测试。
- compileall、git diff --check 通过。
- ROS2 保留工具的纯 Python 测试：car_ros_bridge 4 项、地图生成 1 项，全部通过。
- dry-run 5 步及生成日志的 replay 10 步均在无硬件条件下完成，无 error/safe_stop。
- 无真实板端服务、设备树、电机或相机操作；ROS2 实际节点启动未验证。

## 已部署环境的迁移

仓库部署脚本现在只安装共享报警和电池服务，不再安装旧 PWM/串口屏启动器。
删除仓库文件不会自动停止板端已安装服务。已有部署在切换代码前，需要操作员
停用旧 `mission-screen-launcher.service`、`car-nav2.service` 和
`rock5a-pwm0-permissions.service`，防止旧进程或旧安装副本继续启动：

```bash
sudo systemctl disable --now mission-screen-launcher.service car-nav2.service rock5a-pwm0-permissions.service
sudo systemctl daemon-reload
```

按实际存在的 unit 选择命令；不要停用声光报警、电池监控或 T265 初始化服务。
本次未执行上述板端命令。ROS2 工作区里此前安装的旧包需在操作员确认后重新构建；
不能依靠旧 install/ 的残留入口代表当前源码功能。
