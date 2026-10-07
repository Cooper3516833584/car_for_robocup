# 小车目标识别权重

`best_car.pt` 来自用户现有 `target_yolo/best_car.pt`，未重新训练。
类别：red、blue、green、yellow。测试任务使用全部类别。

文件大小：5,450,515 字节。
SHA-256：`97c58bd858089a8ec7065015efe6fa7fa8556e14b1a677cc750993d6247cefda`。

随 Git 部署到板端，避免任务依赖另一台机器上的绝对路径。加载时只使用
此本地权重，缺失时终止测试。
