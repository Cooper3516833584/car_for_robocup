# 比赛监听的 micromamba 缓存隔离

micromamba 2.9.0 会读取用户缓存目录下的 `mamba/proc/*.json`。
板端已发生两次空 JSON 导致监听无法启动；移走空文件只能恢复当次启动。

`competition-boot.service` 通过 `tools/mamba_run_isolated.py` 启动原监听。
每次启动在 `/run/robocup-car/mamba-cache/` 下创建新的 `launch-*` 目录，
只给 micromamba 设置这个 `XDG_CACHE_HOME`。历史共享缓存和上次失败的
独立缓存均不参与本次启动，因此不需要等待、扫描或删除其他进程的文件。
`RuntimeDirectoryPreserve=no` 让 systemd 在服务停止时清理运行目录；
`/run` 也不会把旧文件保留到下一次开机。

启动 Python 子进程前恢复原有 `XDG_CACHE_HOME`（原先未设置则取消设置），
保留视觉模型、设置及其他库的缓存位置。micromamba 环境激活、原 Python、
正式监听入口、RUN/STOP、比赛计时和导航保持原样。启动器使用 `exec`，
原有 `KillMode=control-group` 继续覆盖 micromamba 和比赛子进程。

验证：

```sh
python -m unittest discover -s code/test -p test_mamba_run_isolated.py -v
MAMBA_TEST_EXE=/home/radxa/robocup_ros/bin/micromamba \
MAMBA_TEST_PREFIX=/home/radxa/robocup_ros/env \
MAMBA_ROOT_PREFIX=/home/radxa/robocup_ros/root \
/home/radxa/robocup_ros/vision_env/bin/python -m unittest discover \
  -s code/test -p test_mamba_run_isolated.py -v
```

后一条在临时目录注入空文件和非法 JSON：原启动应失败，隔离启动应成功，
并检查子进程原缓存环境、CONDA_PREFIX、PYTHONPATH 和 ROS 设置。
测试不启动监听，不访问电机、舵机、相机或串口。

部署遵循 AGENTS.md：PC 提交并推送，板端干净快进后安装 service 模板。
先通过 `car_full_flow_test.py --status` 确认 `running=false` 再重启监听；
比赛正在运行时推迟重启。监听启动仍会执行原有五路物资 OPEN 初始化。
