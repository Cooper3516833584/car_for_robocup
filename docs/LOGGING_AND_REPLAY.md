# RoboCup logging and replay

The RoboCup runtime can write a bounded asynchronous JSONL event stream:

```powershell
py -3 code\main_robocup.py --config configs\robocup_diffdrive.example.toml --mode dry-run --log-dir logs\dry-run
```

`logs/` is ignored by Git. Each record uses a host monotonic timestamp relative
to runtime start. Sensor pose, fused pose, navigation, requested and limited
drive commands, and safety transitions share this timeline. The worker logger
uses non-blocking bounded queues; high-priority safety events use a separate
priority queue. Logging exceptions do not enter the motor-control path.

Replay canonical `t265_pose` and `d500_pose` events through the current fusion
configuration without sensor hardware or algorithm sleeps:

```powershell
py -3 code\main_robocup.py --mode replay --replay-file logs\dry-run\events.jsonl
py -3 tools\replay_pose_log.py logs\dry-run\events.jsonl --output logs\dry-run\fused.jsonl
py -3 tools\summarize_run.py logs\dry-run\events.jsonl
```

Replay files may also contain unrelated diagnostic events; the fusion replay
reader selects only valid canonical T265 and D500 pose records. `--config`
lets the same log be replayed against a changed fusion configuration.
