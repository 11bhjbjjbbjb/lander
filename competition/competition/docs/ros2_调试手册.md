月球探索 ROS2 调试手册

---

> 说明：本手册是在《ros1_调试手册.md》的基础上整理的 **ROS2 版本调试文档**。  
> ROS1 与 ROS2 的比赛规则、功能模块基本一致，**唯一新增**的是 ROS2 版本中的「大模型场景理解任务」，其它调试步骤保持相同思路，仅将指令更新为 ROS2 用法并同步了工程路径。

## 目录

1. 实现对齐夹取平台功能  
1.1 夹取平台颜色阈值的调节（LAB TOOL）  
1.2 对齐夹取平台校准（底盘视觉对齐）  
1.3 对准夹取平台功能测试  

2. 实现夹取功能  
2.1 夹取功能校准  
2.1.1 校准平面  
2.1.2 校准夹取平台  
2.2 夹取功能测试  
2.3 夹取功能调试  

3. 实现对齐坡面功能  
3.1 小车识别地面校准  
3.2 小车识别坡面校准  
3.3 对齐坡面功能测试  
3.4 对齐坡面功能调试  

4. 建立地图  

5. 启动月球探索 ROS2 程序  

6. 调试部分  
6.1 调节机械臂偏差  
6.2 校准 IMU、线速度、角速度  
6.3 替换离线语音资源  
6.4 夹取调整  
6.5 导航位置调整（非必要）  
6.6 导航参数调节（不建议修改）  
6.7 固定参数修改（非必要）  
6.8 训练 YOLO 模型（非必要）  
6.9 进阶优化（形状识别）  

7. 大模型场景理解任务（ROS2 新增）  
7.1 场景理解服务启动  
7.2 三张任务卡片识别流程  
7.3 大模型任务常见问题排查  

8. 文件说明（ROS2 版本）  

---

ROSLander 月球探索 ROS2 实现引导的作用：

1. **基础部分**  
   - 介绍构成“月球探索（ROS2）”程序的功能模块  
   - 完成项目中单项功能模块的实现（对齐夹取、夹取、对齐坡面、建图）  
   - 在单项功能调通的基础上，完成完整月球探索项目

2. **调试部分**  
   - 对月球探索项目实现中出现的功能问题进行定位与调试  
   - 优化月球探索项目实现效果  

3. **进阶优化**  
   - 通过修改参数或源码，使项目效果进一步优化  

4. **文件说明**  
   - 说明 ROS2 竞赛包中各个文件（源码、配置文件等）的位置及作用

> 使用本手册前，请先完成《ROSLander 快速使用手册》中 ROS2 相关环境配置与建图体验，否则可能无法顺利完成竞赛项目。

---

## 1 实现对齐夹取平台功能

### 1.1 夹取平台颜色阈值的调节（LAB TOOL）

整体思路与 ROS1 一致，仍通过 LAB TOOL 调节颜色阈值，只是相机启动命令改为 ROS2 形式。

#### 1.1.1 LAB TOOL 的启动及关闭

注意事项：

1. 终端输入指令需严格区分大小写，建议多使用 `Tab` 补全。  
2. 按顺序执行步骤，否则 LAB TOOL 可能无法正常打开。

步骤：

1. 启动机器人，并通过 NoMachine 远程连接。  
2. 在桌面点击终端图标，打开命令行终端。  
3. 关闭手机 APP 自启服务（与 ROS1 相同）：  

   ```bash
   sudo systemctl stop start_app_node.service
   ```

4. 启动深度摄像头（ROS2）：

   ```bash
   ros2 launch peripherals depth_camera.launch.py
   ```

5. 打开新的终端，启动 LAB TOOL 工具：

   ```bash
   python3 ~/software/lab_tool/main.py
   ```

6. 按照软件界面进行颜色阈值调节；关闭时点击关闭按钮并选择 “Yes”。  
7. 为不影响后续程序调用，请在启动摄像头的终端使用 `Ctrl+C` 结束相机节点。  
8. 重新启动 APP 自启服务：

   ```bash
   sudo systemctl restart start_app_node.service
   ```

   启动完成后机械臂会回到初始位置。

#### 1.1.2 LAB TOOL 的界面说明

界面分为「画面显示区」与「识别调节区」，功能与 ROS1 一致，此处不再赘述，可直接参考《ros1_调试手册.md》中的说明。

特别提醒：

- 若右侧原始画面黑屏或无画面，请检查摄像头线缆并重新插拔或重启相机节点。

#### 1.1.3 调节颜色阈值示例

调节方法与 ROS1 相同，仍通过 LAB 颜色空间调节 `L/A/B` 三个通道的 `min/max` 范围，直到左侧处理画面中 **目标区域为白色，其它区域为黑色**。  
示例步骤、阈值范围对照表可直接参考 ROS1 文档，本处只提醒两点：

- 红色靠近 `+a`，增大 A 分量的 `min`；  
- 颜色偏浅增大 L 分量，偏深减小 L 分量；暖/冷色通过 B 分量调节。

#### 1.1.4 调节相机识别夹取平台的颜色阈值

与 ROS1 相同：  

- 使用橙色海绵块作为夹取平台目标；  
- 在 LAB TOOL 中调节阈值，确保能完整识别到海绵块区域；  
- 将该颜色命名为 `box`，供后续识别与夹取使用。

### 1.2 对齐夹取平台校准（底盘视觉对齐）

ROS2 中对齐夹取平台的逻辑与 ROS1 一致，接口仍为 `/position_correction` 下的服务，只是调用方式改为 `ros2 service call`。

启动前准备：

- 机器人前方 2 米范围内不能有与海绵块相同颜色的物体；  
- 将海绵块正放在机器人正前方约 20 cm 处；  
- 海绵块长边与机器人平行，且中心与机器人中心对齐。

步骤：

1. 打开终端，关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

2. 启动对齐与校准相关节点（包含相机、控制器、自动对齐节点等）：

   ```bash
   ros2 launch competition automatic_pick.launch.py
   ```

3. 在新的终端中，先启动图像（可选，若 launch 已带 `debug_display:=true` 可省略）：

   ```bash
   ros2 service call /position_correction/start std_srvs/srv/Trigger "{}"
   ```

4. 调用校准服务：`data: true` 表示同时开启停靠点保存模式（将目标 box 置于视野内，配合 `debug:=true` 稳定约 10 帧后自动保存）；`data: false` 仅摆机械臂不进入保存流程。

   ```bash
   ros2 service call /position_correction/calibration std_srvs/srv/SetBool "{data: true}"
   ```

5. 若上一步使用 `data: true`，且节点以 `debug:=true` 启动，将目标(box)置于画面内保持稳定，出现终端提示「[校准] 保存完毕」后即自动退出保存模式，按需 `Ctrl+C` 结束即可。

### 1.3 对准夹取平台功能测试

测试流程与 1.2 基本一致，只是服务调用顺序略有不同。

1. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

2. 启动对齐相关程序：

   ```bash
   ros2 launch competition automatic_pick.launch.py
   ```

3. 在新终端中，启动位置识别：

   ```bash
   ros2 service call /position_correction/start std_srvs/srv/Trigger "{}"
   ```

4. 在另一终端中，调用对齐与夹取前对正服务（与 ROS1 中的 `pick_1` 对应，接口名称保持一致）：

   ```bash
   ros2 service call /position_correction/pick_1 std_srvs/srv/Trigger "{}"
   ```

若机器人能自动调整姿态并对准夹取平台，则对齐功能正常；若没有动作或对齐偏差较大，可参考“6. 调试部分”中的相关小节进行排查。

---

## 2 实现夹取功能

夹取功能依赖形状识别与深度信息，与 ROS1 一致，需要先做平面与夹取平台的标定，再进行正式夹取测试。

### 2.1 夹取功能校准

#### 2.1.1 校准平面

1. 确认机器人前方地面无障碍物。  
2. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

3. 启动形状识别与机械臂相关节点（ROS2）：

   ```bash
   ros2 launch competition shape_recognition.launch.py
   ```

4. 在新终端中，调用平面校准服务：

   ```bash
   ros2 service call /shape_recognition/calibration_flat std_srvs/srv/Trigger "{}"
   ```

5. 再打开一个终端，启动形状识别并保存平面数据：

   ```bash
   ros2 service call /shape_recognition/start std_srvs/srv/Trigger "{}"
   ```

校准过程中机械臂会移动到预定姿态，请注意周围是否有干涉物体。

#### 2.1.2 校准夹取平台

1. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

2. 启动形状识别相关程序：

   ```bash
   ros2 launch competition shape_recognition.launch.py
   ```

3. 将夹取平台放在指定位置，与机器人平行并留约 1 cm 间隙。  
4. 在新终端调用距离校准服务：

   ```bash
   ros2 service call /shape_recognition/calibration_dist std_srvs/srv/Trigger "{}"
   ```

5. 再打开终端，启动形状识别并保存夹取平台参数：

   ```bash
   ros2 service call /shape_recognition/start std_srvs/srv/Trigger "{}"
   ```

### 2.2 夹取功能测试

与 ROS1 一致：先启动形状识别并**先调用 `/start` 再调用 `/pick`**。`/pick` 会先将机械臂移到向下识别的姿态，再开始检测目标并执行夹取动作。

1. 将夹取平台放入机器人视野，并按 2.1.2 的姿态摆放；在平台上放置待夹取物块（长方体/正方体/圆柱体）。  
2. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

3. 启动形状识别与机械臂程序（会同时启动摄像头、kinematics、舵机、形状识别节点）：

   ```bash
   ros2 launch competition shape_recognition.launch.py
   ```

4. （可选）设置待夹取物体形状，默认 `box`（长方体）；若夹正方体设为 `cube`，圆柱体设为 `cylinder`。**需在调用 `/pick` 之前设置**，`/pick` 会读取该参数：

   ```bash
   ros2 param set /shape_recognition target_shape "box"
   ```

5. 在新终端**先**启动形状识别（订阅相机并开始同步图像）：

   ```bash
   ros2 service call /shape_recognition/start std_srvs/srv/Trigger "{}"
   ```

6. 再打开终端，调用夹取服务（机械臂会先下摆到夹取姿态，再根据识别结果执行夹取）：

   ```bash
   ros2 service call /shape_recognition/pick std_srvs/srv/Trigger "{}"
   ```

若能正常完成夹取，说明夹取功能调试成功；否则可参考下一小节进行调试。

### 2.3 夹取功能调试

常见问题与 ROS1 一致：

- **问题 1：机械臂不进入夹取动作（调用 `/pick` 后无反应或只下摆不夹取）**  
  - 必须**先调用 `/start` 再调用 `/pick`**，否则没有图像输入，不会触发夹取逻辑。  
  - 确认 `ros2 launch competition shape_recognition.launch.py` 已启动 **kinematics** 与 **舵机** 节点（launch 默认会带起）；若 endpoint 一直为空，会打印“等待机械臂当前位姿，暂不执行移动”。  
  - 确认 `target_shape` 与台上物体一致：长方体用 `box`，正方体用 `cube`，圆柱体用 `cylinder`；可在调用 `/pick` 前执行 `ros2 param set /shape_recognition target_shape "box"`。

- **问题 2：机器人没有进行夹取，机械臂朝下静止不动**  
  - 通常是相机与夹取平台的夹角不合适或平面标定失败。  
  - 建议先按照“6.1 调节机械臂偏差”将机械臂调整到标准姿态，然后 **重新执行 2.1 的两步校准**。

- **问题 3：机器人可以夹取，但没有夹住物块**  
  - 说明识别正常，但机械臂末端目标位置有偏差。  
  - 需要按照“6.4 夹取调整”中说明，通过修改 `config.yaml` 中的补偿参数进行微调。

---

## 3 实现对齐坡面功能

ROS2 中坡面对齐功能的节点与话题名称保持与 ROS1 一致，只是使用 ROS2 的启动与服务调用方式。

### 3.1 小车识别地面校准

1. 确保机器人视野中都是平地，无任何物体干扰。  
2. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

3. 启动坡面识别相关程序：

   ```bash
   ros2 launch competition ramp.launch.py
   ```

4. 在新终端调用地面校准服务：

   ```bash
   ros2 service call /ramp/calibration_flat std_srvs/srv/Trigger "{}"
   ```

5. 再打开终端，启动坡面识别并保存数据：

   ```bash
   ros2 service call /ramp/start std_srvs/srv/Trigger "{}"
   ```

出现标志性提示后按 `Ctrl+C` 结束即可。

### 3.2 小车识别坡面校准

1. 在实际赛场中操作，保证机器人视野中心对准坡面。  
2. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

3. 启动坡面对齐程序：

   ```bash
   ros2 launch competition ramp.launch.py
   ```

4. 在新终端调用坡面校准服务：

   ```bash
   ros2 service call /ramp/calibration_ramp std_srvs/srv/Trigger "{}"
   ```

5. 再打开终端，启动识别并保存数据：

   ```bash
   ros2 service call /ramp/start std_srvs/srv/Trigger "{}"
   ```

若画面中绿线与黄色矩形框之间角度差较大，请适当旋转小车重新校准，直至两者基本平行。

### 3.3 对齐坡面功能测试

1. 确保机器人视野中存在坡面。  
2. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

3. 启动坡面对齐程序：

   ```bash
   ros2 launch competition ramp.launch.py
   ```

4. 在新终端启动坡面识别：

   ```bash
   ros2 service call /ramp/start std_srvs/srv/Trigger "{}"
   ```

5. 在另一终端调用对齐坡面服务：

   ```bash
   ros2 service call /ramp/up std_srvs/srv/Trigger "{}"
   ```

若机器人会调整自身位置并对齐坡面，则功能正常；否则参考下一节“3.4 对齐坡面功能调试”。

### 3.4 对齐坡面功能调试

- 若测试时机器人没有移动，通常是坡面校准失败或者没有正确调用 `calibration_ramp` 与 `start`。  
- 请重新执行 **3.1 与 3.2 的地面与坡面校准步骤**，然后再次进行 3.3 测试。

---

## 4 建立地图

ROS2 版本建图流程与 ROS1 基本一致，仍建议按照《ROSLander 快速使用手册》中“建图体验”部分完成：

1. 将机器人摆放在比赛场地的起点位置；  
2. 使用配套建图程序完成场地建图；  
3. 建图完成后回到出发点，保存地图。  

完成后需要记住地图路径，ROS2 版本默认地图路径示例为：

- `~/ros2_ws/src/slam/maps/map_01`

该路径会作为 `position_correction_pick.launch.py` 中 `map` 启动参数的默认值。

---

## 5 启动月球探索 ROS2 程序

在完成前述各项标定与地图构建后，可启动完整的月球探索 ROS2 程序。

1. 将机器人摆放在建立地图时的起点位置，并确保地图已经保存。  
2. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

3. 启动月球探索 ROS2 主程序：

   ```bash
   ros2 launch competition position_correction_pick.launch.py
   ```

   - 如需指定自定义地图，可添加参数（示例）：  

     ```bash
     ros2 launch competition position_correction_pick.launch.py map:=/home/ubuntu/ros2_ws/src/slam/maps/my_map
     ```

4. 启动完成后，使用语音唤醒与控制（与 ROS1 一致）：  
   - 说：“小迈小迈” → 唤醒机器人；  
   - 唤醒成功后，说：“开始执行任务” → 机器人开始执行整套月球探索任务。  

若机器人没有开始任务，请参考“6. 调试部分”中的相关条目排查。

---

## 6 调试部分

### 6.1 调节机械臂偏差

若在夹取时机械臂无法到达指定位置，说明机械臂零位或关节偏差较大。  
请参考课程中“机械臂偏差调节（选看）”部分，完成关节零位与偏差的重新标定；ROS2 下机械臂驱动方式与 ROS1 基本一致，只是话题与服务前缀切换为 ROS2 风格。

### 6.2 校准 IMU、线速度、角速度

若导航过程中，机器人在地图上的位置与实际位置偏差较大，需要进行 IMU 与运动学参数校准。  
可参考课程中“IMU 校准”、“里程计标定”等章节，ROS2 框架下的标定思路与 ROS1 相同。

### 6.3 替换离线语音资源

若启动后无法正常唤醒机器人或播报“开始执行任务”，需要检查离线语音资源与 APPID 设置：

1. 按课程“配置麦克风”章节申请 APPID 并替换离线资源；  
2. 修改 ROS2 竞赛包中对应的 APPID 配置（与 ROS1 相同，只是工程路径为 `ros2_ws`）。

语音底层启动依赖 `xf_mic_asr_offline` 包，可单独测试麦克风初始化：

```bash
ros2 launch competition mic_init.launch.py
```

### 6.4 夹取调整

若识别正常但夹取位置存在系统性偏差，可通过修改补偿参数进行调节。

- **参数文件位置（ROS2）**：

  ```text
  /home/ubuntu/ros2_ws/src/competition/config/config.yaml
  ```

- **关键参数说明**：
  - `offset`: 三个数值分别对应 \(x, y, z\) 方向的补偿（单位：米）；  
  - `pick_location_time`: 夹取前向前移动的时间（秒）；  
  - `pick_stop_pixel_coordinate` / `place_stop_pixel_coordinate`: 图像坐标下的停止位置像素等。  

坐标含义与 ROS1 一致：

- \(x\)：控制夹取前后距离；  
- \(y\)：控制夹取左右偏差；  
- \(z\)：控制夹取高度。  

根据夹取时的实际偏差逐步调节即可。

### 6.5 导航位置调整（非必要）

若导航到某个任务点时，实际位置与预期不符，可对目标点坐标进行微调。

1. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

2. 打开导航语音控制脚本（ROS2 路径）：

   ```bash
   gedit /home/ubuntu/ros2_ws/src/competition/competition/navigation_transport/voice_control_navigation.py
   ```

3. 在脚本中找到 `self.control(...)` 的调用位置，参数含义与 ROS1 一致：
   - `x`：目标点横向坐标（米）  
   - `y`：目标点纵向坐标（米）  
   - `yaw`：朝向角度（度）  
   - `mode`：模式/任务点标识

4. 修改完脚本后，重新启动主程序进行验证：

   ```bash
   ros2 launch competition position_correction_pick.launch.py
   ```

5. 若希望通过实际导航点来重新标定，可在导航过程中使用：

   ```bash
   ros2 topic echo /goal_pose
   ```

   或使用导航界面选择目标点，根据输出的目标坐标填回 `self.control` 中。

### 6.6 导航参数调节（不建议修改）

ROS2 中导航使用的是 Nav2 框架，主要参数集中在以下文件：

- `ros2_ws/src/competition/config/costmap_common_params.yaml`  
- `ros2_ws/src/competition/config/global_costmap_params.yaml`  
- `ros2_ws/src/competition/config/local_costmap_params.yaml`  
- `ros2_ws/src/competition/config/teb_local_planner_params.yaml` 等  
- 以及 `ros2_ws/src/navigation/config/nav2_params.yaml`、`nav2_controller_teb.yaml` 等。

通常默认参数已经调好，**非必要不建议修改**。如果确实需要优化：

- costmap 相关参数可参考 ROS Wiki / Nav2 官方调参文档；  
- TEB 局部规划器参数可根据避障与路径平滑性需求微调。  

若出现倒车时容易撞到围挡的问题，可重点检查 TEB 相关的速度与加速度约束配置。

### 6.7 固定参数修改（非必要）

#### 6.7.1 夹取向前时间修改

若机器人夹取时向前移动距离过大，可能会顶到平台，需要适当缩短向前移动时间。

- **参数文件位置**：

  ```text
  /home/ubuntu/ros2_ws/src/competition/config/config.yaml
  ```

- **相关参数**：`pick_location_time`

调试步骤示例：

1. 关闭 APP 自启服务：

   ```bash
   sudo systemctl stop start_app_node.service
   ```

2. 启动主程序：

   ```bash
   ros2 launch competition position_correction_pick.launch.py
   ```

3. 在新终端调用对齐并前进的服务：

   ```bash
   ros2 service call /voice_control_navigation/aligning std_srvs/srv/Trigger "{}"
   ```

4. 根据小车停止位置与平台距离，调整 `pick_location_time`：  
   - 撞到台子 → 减小时间；  
   - 离台子过远 → 增加时间。

#### 6.7.2 上坡向前时间修改

若机器人在上坡过程中向前移动过多，可能撞到围挡，可通过修改 `up_ramp_time` 调整。

- **参数文件位置**：

  ```text
  /home/ubuntu/ros2_ws/src/competition/config/config.yaml
  ```

- **相关参数**：`up_ramp_time`

调试步骤类似 6.7.1：

1. 关闭 APP 自启服务；  
2. 启动主程序：

   ```bash
   ros2 launch competition position_correction_pick.launch.py
   ```

3. 调用上坡部分测试服务：

   ```bash
   ros2 service call /voice_control_navigation/back std_srvs/srv/Trigger "{}"
   ```

4. 根据是否碰到围挡对 `up_ramp_time` 做增减，直至上坡停止位置合理。

### 6.8 训练 YOLO 模型（非必要）

ROS2 中仍使用 YOLOv5 进行卡片/物体识别，当前默认版本仍为 `yolov5 6.2`。  
若需要更高识别效果，可按照机器学习课程重新训练模型，并替换工程中的模型文件。

- **模型文件放置路径（ROS2 安装后 share 路径）**：
  - 运行时通过 `get_package_share_directory('competition')` 获取 share 目录，  
  - 模型文件位于：

    ```text
    <share>/competition/navigation_transport/yolo5/
    ```

  - 默认文件名示例（可根据实际调整）：`shape_models.engine`、`shape_models_libmyplugins.so` 或 `best.pt` 等。

- **调试 YOLO 的专用启动文件（仅 YOLO + 麦克风）**：

  ```bash
  ros2 launch competition competition.launch.py
  ```

  该 Launch 会启动 `mic_init` 与 `yolov5_node`，便于单独测试识别效果。

如需更换模型文件名或类别列表，请修改：

- `ros2_ws/src/competition/launch/competition.launch.py` 中 `yolov5_node` 的参数（`engine` / `lib` / `classes` 等）；  
- 以及主程序 `position_correction_pick.launch.py` 中关于 YOLO 的配置，保证两处配置一致。

### 6.9 进阶优化（形状识别）

进阶优化部分与 ROS1 基本一致，主要是对形状识别算法逻辑进行微调：

- **核心代码位置（ROS2）**：

  ```text
  /home/ubuntu/ros2_ws/src/competition/competition/navigation_transport/shape_recognition/shape_recognition_down.py
  ```

整体思路：

- 根据识别到的宽、高、边数量以及处理后的深度统计量（标准差等），对目标进行更稳定的分类；  
- 可通过打印中间变量，观察在不同物块和光照条件下的量纲变化，再调整阈值。

如需调试，可参考配套视频教程，在代码中临时打印宽度/高度/边数量/标准差等信息，并逐步优化判断阈值。

---

## 7 大模型场景理解任务（ROS2 新增）

ROS2 版本相对 ROS1 **新增的大模型任务**，主要负责识别三张任务卡片中的场景元素，并进行语音播报。  
该部分由 `vllm_scene_understand_service` 节点与相关服务/话题组成。

### 7.1 场景理解服务启动

若只想单独调试大模型场景理解模块，可使用专用的 Launch 文件：

```bash
ros2 launch competition vllm_scene_understand_service.launch.py
```

该 Launch 会：

- 可选地启动摄像头（`peripherals/depth_camera.launch.py`）；  
- 启动 `vllm_scene_understand_service` 节点；  
- 根据参数决定是否开启图像预览窗口。

主要启动参数（可通过 `ros2 launch` 传入或 `ros2 param` 修改）：

- `image_topic`：默认 `/astra_camera/rgb/image_raw`  
- `window_name`：预览窗口名称，默认 `VLLM Scene View`  
- `default_prompt`：默认场景理解提示词（已预设为月球探索场景）  
- `enable_preview`：是否启用预览窗口（`true` / `false`）  
- `enable_camera`：是否在 Launch 内自动启动摄像头节点

节点启动后会订阅摄像头话题，并在需要时调用大模型接口进行图像理解与 TTS 播报。

### 7.2 三张任务卡片识别流程

在主程序中，大模型任务用于识别三个任务点处的卡片内容。  
服务接口基于 `std_srvs/srv/Trigger`，可通过以下命令在终端单独调用进行测试：

1. **识别三张卡片（不立即播报）**：

   ```bash
   ros2 service call /vllm_scene_understand_service/recognize_card_1 std_srvs/srv/Trigger "{}"
   ros2 service call /vllm_scene_understand_service/recognize_card_2 std_srvs/srv/Trigger "{}"
   ros2 service call /vllm_scene_understand_service/recognize_card_3 std_srvs/srv/Trigger "{}"
   ```

   - 每次调用会抓取当前相机画面，并通过大模型给出识别结果；  
   - 结果会被缓存下来，暂不播报。

2. **统一播报三张卡片识别结果**：

   ```bash
   ros2 service call /vllm_scene_understand_service/trigger_report std_srvs/srv/Trigger "{}"
   ```

   - 若三张卡片中仍有未识别的任务点，服务会提示哪些卡片尚未识别；  
   - 全部识别完成后，会依次播报三个任务点对应的场景元素。

3. **使用默认提示词进行一次场景理解并播报（保留接口）**：

   ```bash
   ros2 service call /vllm_scene_understand_service/trigger_scene_understand std_srvs/srv/Trigger "{}"
   ```

4. **通过话题发送自定义提示词并触发场景理解**：

   ```bash
   ros2 topic pub --once /vllm_scene_understand_service/scene_understand_prompt std_msgs/msg/String "data: '你是一个月球探索爱好者，帮我看看这张卡片上主要是什么元素。'"
   ```

   节点收到自定义 Prompt 后，会使用该提示词进行一次新的场景理解并根据配置决定是否播报。

5. **状态反馈**：

   - 节点会在 `/vllm_scene_understand_service/status` 上发布当前状态（如 `start` / `stop`），可使用：

     ```bash
     ros2 topic echo /vllm_scene_understand_service/status
     ```

     来观察任务进行情况。

### 7.3 大模型任务常见问题排查

1. **未获取到摄像头画面**  
   - 终端出现 “未获取到摄像头画面，请确认摄像头节点是否已启动”；  
   - 检查 `peripherals` 包是否正常，确认 `depth_camera.launch.py` 已启动且 `image_topic` 与节点参数一致。

2. **大模型调用失败**  
   - 日志中提示 “VLLM 调用失败”；  
   - 请检查 `competition/large_models/config.py` 中的 `api_key`、`base_url`、模型名称等配置是否正确；  
   - 确认网络连通性、显卡/推理环境是否准备完毕。

3. **识别结果为空或不合理**  
   - 可能是光照、卡片距离或角度问题，建议调高光照、正对摄像头；  
   - 可更换 `default_prompt` 或通过话题发送更明确的自定义提示词；  
   - 若总是返回“未识别到相关元素”，请确认卡片设计是否符合题目要求。

---

## 8 文件说明（ROS2 版本）

以下为 ROS2 竞赛包中与比赛相关的主要文件与目录说明（与 ROS1 基本对应，路径根据 ROS2 工程结构调整）：

- **源代码（Python 节点等）**  
  - 位置：

    ```text
    /home/ubuntu/ros2_ws/src/competition/competition/
    ```

  - 主要子目录：
    - `navigation_transport/calibration_position/automatic_pick.py`：对齐夹取平台源码  
    - `navigation_transport/ramp/ramp.py`：对齐坡面源码  
    - `navigation_transport/shape_recognition/shape_recognition_down.py`：形状识别与夹取逻辑  
    - `navigation_transport/yolo5/yolov5_node.py`、`yolov5_trt_6_2.py`：YOLO 识别节点与推理逻辑  
    - `navigation_transport/voice_control_navigation.py`：主语音控制与任务调度程序  
    - `large_models/vllm_scene_understand_service.py`：大模型场景理解服务节点（ROS2 新增）

- **配置文件**  

  ```text
  /home/ubuntu/ros2_ws/src/competition/config/
  ```

  - `config.yaml`：对齐、夹取、坡面等功能的标定与补偿参数  
  - `costmap_common_params.yaml`、`global_costmap_params.yaml`、`local_costmap_params.yaml`：导航 costmap 相关参数  
  - `teb_local_planner_params.yaml`、`dwa_local_planner_params.yaml` 等：局部规划器相关参数  
  - 以及 Nav2 配置（在 `navigation/config` 目录下），用于全局导航调参。

- **Launch 文件**（ROS2）  

  ```text
  /home/ubuntu/ros2_ws/src/competition/launch/
  ```

  - `automatic_pick.launch.py`：对齐夹取平台（位置校正）相关节点启动  
  - `shape_recognition.launch.py`：形状识别与夹取相关节点启动  
  - `ramp.launch.py`：坡面对齐相关节点启动  
  - `position_correction_pick.launch.py`：月球探索 ROS2 主程序（包含导航、YOLO、对齐、夹取、坡面、大模型等）  
  - `competition.launch.py`：仅启动 YOLO + 麦克风，用于识别模型单独测试  
  - `mic_init.launch.py`：麦克风初始化  
  - `vllm_scene_understand_service.launch.py`：大模型场景理解服务调试启动文件

- **导航包（Nav2 相关）**  

  ```text
  /home/ubuntu/ros2_ws/src/navigation/
  ```

  - `launch/navigation.launch.py`：导航整体启动（被 `position_correction_pick.launch.py` 引用）  
  - `launch/include/*.launch.py`：局部包含文件，如 `localization.launch.py`、`navigation_base.launch.py` 等  
  - `config/nav2_params.yaml`：Nav2 核心参数  
  - `config/nav2_controller_*.yaml`：控制器（如 TEB、DWB）参数

通过以上说明，可以将 ROS1 调试文档中的步骤一一映射到 ROS2 工程中：  

- **命令层面**：`roslaunch` → `ros2 launch`，`rosservice` → `ros2 service`，`rosparam` → `ros2 param`，`rostopic` → `ros2 topic`；  
- **路径层面**：`ros_ws/src/competition/...` → `ros2_ws/src/competition/...`；  
- **功能层面**：对齐、夹取、对齐坡面、建图、导航、语音等逻辑保持不变，仅新增了基于大模型的场景理解任务。

