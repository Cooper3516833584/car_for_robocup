# 任务板配额下发给无人机

小车只负责读板和下发自由投放配额；红、蓝、绿地块的 4 件自由物资全部由无人机执行。
任务板印刷三行，例如红 2、蓝 1、绿 1，数量在比赛前随机设置。正式比赛发送实际 OCR 共识结果，不使用示例值。
小车自身黄色目标的投放路线与本次自由配额通信相互独立，本次不修改它的运动和继电器逻辑。

完整比赛 `code/competition_task.py` 在读板后调用 `send_task_to_drone_once()`：名称保留供现有调用者兼容，现在代表一次逻辑任务交接，含确认和重传。
读板失败或任务/启动确认失败时底盘停车并结束当前任务，不以 `(1,2,1)` 兜底继续整场。
独立 `task-board` 调试仍可显示旧兜底值；这些值不会进入正式比赛发送路径。

## 同一无线链路的两步交接

复用 FleetBus V1 帧、CRC 和 BB33 封装。源小车 `0x20`，目标无人机 `0x10`。
小车 HC-15 仍为 `/dev/ttyS4`、115200 8N1。机端沿用 `FC_Client → FC_Server → 飞控 UART2`，不打开 CH340 继电器端口。

| 命令 | body（小端） | 成功确认 |
| --- | --- | --- |
| `DRONE_SET_DROP_TASK=0x26` | `version:u8=1, task_id:u32, red:u8, blue:u8, green:u8` | `COMPLETED` 表示合法配额已保存 |
| `DRONE_START_DROP_TASK=0x27` | `version:u8=1, task_id:u32` | `ACCEPTED` 表示对应任务的启动请求已接受 |

每色 0～4，合计必须为 4。黄色必投数量保留无人机现有固定 1 件，不属于任务板的自由配额。
任务编号为非零随机 32 位值，两条命令处于同一小车 session；机端拒绝未保存配额、错编号、错 session 和改变已启动任务的请求。
只接收小车直接寻址的命令，地面站不能直接给此机端任务发配额/开始。ACK 回到请求来源；默认旧 AirFleetNode 的地面站控制方式保持兼容。

一次确认等待 6 秒，最多 3 次重传。重传复用原 session/seq/完整帧，机端缓存响应；即使任务或启动 ACK 丢失，也不会重置投放账本或重复启动。
小车只认无人机地址及对应 session/seq/command_id 的 ACK；数据保存必须等 `COMPLETED`，只收到 `RECEIVED`/`ACCEPTED` 不发送开始。
ACK 只表示软件交接，不能作为起飞、落地或投放完成的确认。

机端 RX 工作线程只校验并保存任务。`rescue_drop_2026.py --car-control --confirm-flight` 的原任务线程拿到匹配开始后，把真实配额交给 `DropLedger` 再执行现有飞行任务。原视觉、定位、避障、继电器及真实飞行确认要求继续生效。
`TARGETED_STOP` 设置原任务 stop_event，ACK 为 `ACCEPTED`；着陆由原任务清理逻辑处理。双方重启到待命以及地面站开始/终止到小车的常驻接收入口仍需单独接入，不能把本次配额交接当作完整比赛自启动。

## 独立通信检查

仅下发明确给出的配额，不请求无人机启动，也不运行小车定位/运动：

```bash
python3 code/main_robocup.py --competition-stage hc-send \
  --task-red-count 2 --task-blue-count 1 --task-green-count 1
```

机端可用 `python3 python_sdk/rescue_drop_2026.py --car-control` 接收并记录配额，仍保持原定位监视模式；不加 `--confirm-flight` 会拒绝启动请求。
真正比赛机端由人工确认环境后使用 `--car-control --confirm-flight` 和已确认的继电器端口；小车完整比赛会在读板成功后完成两步交接。
同一小车 UART 只应有一个所有者；将来增加常驻地面站接收时，共享 `FleetTaskSender` 的 transport 并分发 RX，不能另开同一路 UART。

跨仓库无硬件联测：

```bash
python3 tools/test_drop_task_link.py --drone-sdk ../drone/python_sdk
```

此脚本使用两端实际协议和节点实现，模拟飞控 BB33 解包/封包，覆盖全部 15 种配额、任务/启动 ACK 丢失、CRC 损坏和分段接收，不导入或连接真实飞控、相机、继电器。

## 本次修改与验证（2026-10-11）

修改文件：`code/competition_task.py`、`code/main_robocup.py`、`code/components/fleet_models.py`、`code/components/drop_task_protocol.py`、`code/components/fleet_task_sender.py`、`code/test/test_competition_task.py`、`code/test/test_competition_direct_stages.py`、`code/test/test_fleet_task_sender.py`、`tools/test_drop_task_link.py`、`docs/COMPETITION_TASK.md`、本文。

已执行 `python3 -m compileall -q code tools`、`python3 -m unittest discover -s code/test -p 'test_*.py'`（965 项，7 项跳过）、跨仓库联测（45 组通过）和 `git diff --check`。
按用户要求仅推送 GitHub，本次不拉取设备、不启动任务；未验证真实无线距离、起飞、投放、落地和双方重启待命。
