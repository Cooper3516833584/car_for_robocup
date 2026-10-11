# CH3 投放车道边界修复（2026-10-11）

## 当前动作

+90° YOLO 搜索中央 10% → 停车取三个不同时间戳的新框中心 → 必要时在车道内
前进 2 cm 测一次像素增益，固定符号做不超过 2 cm 的融合直进/直退 → 前进 7 cm
→ 左转 90° 一次，保存实际转后 pivot → 舵机 0° → 一次侧向融合直线接近
→ CH3 释放一次 → 从最新融合位置沿 side_yaw 直倒回 pivot 内侧截面 → 右转
90° 一次 → 舵机 +90° → 继续原比赛车道。

侧向接近到退回内侧期间，任务层只允许平行于 side_yaw 的 `track_global_line`。
没有靠边旋转、转向横移、误差方向自动翻转或历史最佳 XY 回跑。全局 PP、
TurnController、融合主链、人工 STOP、总超时、设备错误和驱动急停未修改。
普通 PP 曲率修正不是任务层原地旋转。转向真实平移不再被当作零：接近起点取
左转结束时的实际融合位姿，倒退终点保持当前侧向通道，避免对角追历史 pivot。

## 三个现场数

| 数值 | 当前设置与来源 |
|---|---|
| 安全侧向行程 `DROP_SAFE_M` | **0.47 m**，用户在本次会话提供的 pivot 到安全投放点融合距离 |
| +90° 沿车道参考 `SEARCH_REFERENCE_CX` | **320 px**（640 宽归一化），初始值，仍需现场校准 |
| 机械前进补偿 `DROP_ADVANCE_M` | **0.07 m**，保留原设计，仍需确认是否修正 |

0.47 m 作为本车当前端点使用，不代表软件已验证全场车轮范围。需重复测量最大
正向停车超程、融合漂移及所有车轮外缘，确认该值包含足够内缩余量；pivot 转向
时也必须在车道内。3～5 次安全位置的单独左转测量用于查明此前 8 cm 位移是
机械滑移还是定位外参；本次没有把该单次位移硬编码为补偿。

## 0° 视觉默认状态与采图

旧 `assets/ch3_yellow_servo0_reference.jpg` 只作对比，默认不启用视觉距离修正。
没有新的实帧时仍完成安全融合定距投放。识别函数缩放到 640×480，沿用 HSV，
排除下方挡板，对足够长的上半弧按列取顶部并拟合二次模型；输出 feature X/Y
与可见 span。最近三帧的中位数只有在位置和 span 一致时才有效。靠边控制只用
可靠 Y 且 span 与参考一致时提前停车，X 只留日志。没有 Y 方向自动猜测和横移。
这次只对 yellow + slot2/CH3 开放视觉停车，其它颜色和仓位走融合端点。

停车采集，不打开底盘、定位 runtime 或继电器，也不需要电机确认参数：

```bash
python3 tools/run_ch3_drop_test.py --vision-preview \
  --camera 0 --log-dir logs/ch3-preview-centre
```

将车固定在 CH3 准确覆盖黄色圆心的位置，再在前后各 2 cm 重复采集。每次保存
三张原始分辨率图、HSV mask、拟合叠加图及数值 JSON。验证特征 Y 随前后位移
平滑、方向一致、拟合对象重复后，在 **PC 仓库**保存新的
`assets/ch3_yellow_servo0_field_reference.jpg`，再提交、推送、车端快进更新。
若尚不稳定，保持该文件不存在，继续融合定距。

独立 `main_robocup.py --competition-stage drop-align` 现在仅停车 0° 观测，不驱动
车轮，不投放。完整往返仍使用独立 CH3 测试入口：

```bash
python3 tools/run_ch3_drop_test.py --confirm-motor-test \
  --release-mode simulate --payload-slot 2 --search-distance-m 3 \
  --drop-safe-m 0.47 --log-dir logs/ch3-lane-safe-001
```

默认 release-mode 为 simulate。部署不启动这个测试。连续多次确认车轮不越线、
CH3 覆盖圆心、融合与实物一致、倒退回内侧后才转向，再由现场操作人选择真实继电器。
这不依赖任务板或 HC 场地坐标。主比赛入口继续原路线、读板和 HC 顺序。

## 证据与软件回归

正常日志目录的 `search_frames/` 保存现有 YOLO worker 的原图、框图、时间戳；
`drop_frames/` 保存侧向观测的原图、HSV mask、全弧拟合图和数值。事件记录
yaw、fused_x/y、road/side_projection、observed/reference X/Y、span、chosen_action。
视觉无效不会生成错误移动；相机故障回退，定位损坏和设备故障保留原处理。

针对性测试覆盖固定正反像素增益、重复时间戳、局部遮挡、可见宽度变化、挡板
变化、现场记录的宽度及 X 大跳变、3～8 cm 转向平移、早停后直倒、CH3 一次
释放、原路线继续、STOP 和定位错误。无硬件测试不能替代现场车轮边界验收。

本地 Python 3.13 回归共 945 项，938 项通过、7 项按环境跳过；compileall 与
git diff --check 通过。部署仅更新代码，不启动电机、相机或真实继电器测试。
