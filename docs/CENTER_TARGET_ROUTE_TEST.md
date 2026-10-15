# 相机与融合路线测试

`tools/run_center_target_route.py` 保留 280/420/250 cm 的相机/融合导航/蜂鸣测试，
不提供旧物资继电器路径。`--check-vision` 只检查相机和模型；
真实底盘路线必须传 `--confirm-motor-test --log-dir ...`。

```sh
python3 tools/run_center_target_route.py --check-vision
python3 tools/run_center_target_route.py --confirm-motor-test --log-dir logs/camera-patrol
```

`tools/run_ch3_drop_test.py` 的历史文件名保留，内部只用现有 SG90 组件。
它从当前融合位置和航向生成搜索线，保持相机 +90°，水平对齐后执行同一套
融合接近/单槽释放/原轴返回流程。槽位只有 0/1，默认 0；正常和异常退出均全开。
`--vision-preview` 保留原地相机 0° 三帧证据，不创建底盘 runtime、不操作物资舵机。

```sh
python3 tools/run_ch3_drop_test.py --vision-preview --log-dir logs/stationary-preview
python3 tools/run_ch3_drop_test.py --confirm-motor-test --payload-slot 0 --log-dir logs/sg90-search
python3 tools/run_ch3_drop_test.py --confirm-motor-test --simulate-release --log-dir logs/simulated-release
```

最后一条仍真实行驶，但显式跳过物资舵机。完整单机任务入口为
`code/car_full_flow_test.py`，不是本工具；比赛坐标不使用此工具的搜索线。
`tools/run_yellow_payload_demo.py` 转发单槽黄色测试，默认 slot 0。
单独静止操作使用 `tools/payload_servo_test.py`，详见 [PAYLOAD_SG90.md](PAYLOAD_SG90.md)。
以上真机命令均为现场操作参考，本次离线单测不执行。
