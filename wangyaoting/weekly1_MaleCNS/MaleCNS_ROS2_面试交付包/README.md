# weekly1

1. 环境是：ros2 humble gazebo+rviz\
2. 利用 MaleCNS 中真实生物神经连接关系，构建一个具有“感知—神经信息处理—运动决策”闭环能力的人工神经控制系统，并将其应用于 ROS2 移动机器人，使机器人具备自主避障和导航能力。\
3. 本项目派生数据来自 [HHMI Janelia MaleCNS 官方下载页](https://male-cns.janelia.org/download/)列出的 v1.0 数据：\
- `body-annotations-male-cns-v1.0-minconf-0.5.feather`：神经元 body ID、类别、左右侧等注释；\
- `connectome-weights-male-cns-v1.0-minconf-0.5.feather`：真实有向连接及突触数；\
- `body-neurotransmitters-male-cns-v1.0.feather`：神经递质预测。\
官方页面注明 MaleCNS 数据按 CC-BY 许可发布。此包保留派生子图中的真实 ID、边和突触数；\
针对机器人写的感知编码、神经模型、读出训练、ROS2 控制和仿真代码是工程实现，不属于 MaleCNS 官方模型。`setup.py` 将项目代码标为 MIT，不改变原始数据的 CC-BY 要求。\
4. 我把 MaleCNS v1.0 的真实神经元和突触连接提取成一个小型网络，保留了 304 个原始神经元 ID、2529 条有向边和突触计数。Gazebo 机器人提供激光扫描与里程，我把左、中、右障碍距离及目标方向编码成六路输入。活动沿真实连接图传播；从下行和腹神经索神经元读取 104 维活动，用训练的线性读出生成速度和转向，向 ROS2 `/cmd_vel` 发布。Gazebo 差速轮运动产生新的 `/scan`、`/odom`，控制器每 0.1 秒重新计算，所以是闭环。在一个静态障碍场景中，机器人从左侧起点绕开两处障碍进入目标范围，记录路径约 5.55 米。连接关系来自真实 MaleCNS，但感知映射、活动方程、读出层和近障保护是工程设计；这是局部目标点导航原型，不是完整果蝇脑模拟或 Nav2 全局导航。\
5. ## 代码目录

| 路径 | 作用 |\

| `malecns_robot/connectome.py` | 从真实边建立神经连接矩阵、活动传播、线性读出 |\
| `malecns_robot/controller.py` | ROS2 订阅、10 Hz 控制闭环、停止保护、日志 |\
| `tools/build_circuit.py` | MaleCNS 原始表到派生子图 |\
| `tools/train_readout.py` | 合成专家样本、岭回归、连接清零消融 |\
| `malecns_robot/models/robot.sdf` | 差速轮和激光雷达的 Gazebo 模型 |\
| `malecns_robot/worlds/arena.world` | 两个箱体障碍与边界 |\
| `malecns_robot/launch/demo.launch.py` | 一次启动 Gazebo、机器人、控制器、RViz |\
| `tools/evaluate_run.py` | 从 CSV 重算路程、到达和几何间隙 |\