# 差速小车硬件移植

主入口 `code/main_robocup.py` 和 runtime 使用 SI 单位的差速控制接口。
配置由 `code/config/v2_loader.py` 解析，`v2_factory.py` 组合设备、定位和运动组件。
旧前轮舵机、Ackermann 导航及 v1 配置工厂已删除。

移植时录入实际轮距、轮径、车身轮廓、传感器外参、电机方向、串口路径和协议模式，
并按 [HARDWARE_MEASUREMENTS.md](HARDWARE_MEASUREMENTS.md)、
[HARDWARE_COMMISSIONING.md](HARDWARE_COMMISSIONING.md) 验证后再设置 measured 标记。
坐标原点位于两驱动轮轴线中点，yaw 逆时针为正。

C10B 的 `rear_motor.py` 帧编码、发送线程和 watchdog 保持不变。
保留 `ackermann_firmware_compat` 直到实际验证差速 v/omega 固件；换车时不能
根据底盘名称猜测协议，也不能改速度限制来掩盖轮距或电机方向问题。

D500 协议、T265 适配、GPIO 声光报警、继电器和硬件锁保留各自的组件边界。
新硬件差异放在 adapter/backend，配置读取集中在工厂。
HAL 的 GPIO/PWM 原语和测试仍保留，可供共享硬件使用，但没有前轮转向业务。
当前 ROCK 5A 接线见 [platforms/rock5a.md](platforms/rock5a.md)。