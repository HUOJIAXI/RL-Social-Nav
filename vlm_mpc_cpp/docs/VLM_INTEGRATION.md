# VLM集成节点使用说明

## 概述

`vlm_integration_node` 是一个ROS2节点，用于订阅3D bounding boxes并基于事件触发调用VLM（Vision Language Model）进行场景分析。

## 功能特性

1. **订阅3D Bounding Boxes**: 从 `/darknet_ros_3d/bounding_boxes` topic订阅检测结果
2. **事件触发机制**: 
   - 检测到新的bounding box时触发
   - 机器人移动超过阈值时触发（场景变化）
   - 可选的周期性触发
3. **Prompt组装**: 将3D bounding boxes信息转换为结构化的VLM prompt
4. **VLM API调用**: 支持调用外部VLM API（需要配置）

## 使用方法

### 1. 启动节点

```bash
# 使用默认配置
ros2 launch social_mpc_nav vlm_integration.launch.py

# 使用自定义配置文件
ros2 launch social_mpc_nav vlm_integration.launch.py \
    config_file:=/path/to/your/config.yaml
```

### 2. 配置参数

编辑 `config/vlm_integration.yaml` 文件来配置节点行为：

```yaml
vlm_integration_node:
  ros__parameters:
    # Topic配置
    bounding_boxes_topic: "/darknet_ros_3d/bounding_boxes"
    odom_topic: "/task_generator_node/tiago_official/odom"
    vlm_prompt_topic: "/vlm/prompt"
    vlm_response_topic: "/vlm/response"

    # 事件触发配置
    trigger_on_new_detection: true      # 检测到新物体时触发
    trigger_periodic: false             # 周期性触发
    periodic_interval_sec: 5.0          # 周期（秒）
    trigger_on_scene_change: true       # 场景变化时触发
    scene_change_threshold: 0.3         # 场景变化阈值（米）

    # 检测过滤
    min_confidence: 0.5                 # 最小置信度
    max_detection_distance: 10.0        # 最大检测距离（米）

    # VLM API配置
    enable_vlm_call: false              # 是否启用VLM API调用
    vlm_api_url: "http://localhost:8000/v1/chat/completions"
    vlm_model: "gpt-4-vision-preview"
    vlm_api_key: ""                     # API密钥（建议使用环境变量）
```

### 3. 查看输出

节点会发布以下topics：

- `/vlm/prompt`: 包含组装好的VLM prompt（字符串格式）
- `/vlm/response`: VLM API的响应（当启用API调用时）

查看prompt内容：
```bash
ros2 topic echo /vlm/prompt
```

## 事件触发机制

### 1. 新检测触发 (`trigger_on_new_detection`)
- 当检测到新的物体类别时触发
- 当检测到的物体数量发生变化时触发

### 2. 场景变化触发 (`trigger_on_scene_change`)
- 当机器人移动距离超过 `scene_change_threshold` 时触发
- 用于捕获机器人进入新区域时的场景变化

### 3. 周期性触发 (`trigger_periodic`)
- 按照 `periodic_interval_sec` 设定的间隔定期触发
- 即使没有检测到新物体也会触发

## Prompt格式

节点会将3D bounding boxes信息转换为如下格式的prompt：

```
You are analyzing a 3D scene from a mobile robot's perspective. 
The robot is currently at position (x, y) in the map frame. 
The robot has detected N object(s) in its environment:

Object 1:
  - Class: person
  - Confidence: 0.95
  - Center position: (2.5, 1.2, 0.8) meters
  - Size: 0.5 x 0.3 x 1.7 meters
  - Bounding box: x[min, max], y[min, max], z[min, max]
  - Distance from robot: 2.8 meters
  - Relative angle: 25.5 degrees

...

Please provide:
1. A brief description of the scene
2. Potential navigation constraints or social rules
3. Recommendations for safe and socially-aware navigation
4. Any groups or interactions between detected objects
```

## VLM API集成

### 当前状态

目前节点已经实现了prompt组装和事件触发机制，但VLM API的HTTP调用部分需要进一步实现。

### 实现选项

1. **使用libcurl**: 在 `callVLMAPI()` 函数中实现HTTP POST请求
2. **使用外部服务**: 创建一个独立的ROS2服务节点来处理HTTP调用
3. **使用Python脚本**: 通过system call调用Python脚本进行API调用

### 示例：使用外部Python服务

可以创建一个Python节点订阅 `/vlm/prompt` topic，然后调用VLM API并将结果发布到 `/vlm/response`。

## 与MPC控制器的集成

VLM集成节点可以与现有的MPC控制器配合使用：

1. VLM节点分析场景并提供语义理解
2. VLM响应可以用于：
   - 调整MPC的社交约束权重
   - 识别需要特殊处理的区域（如人群聚集区）
   - 提供导航建议

## 调试

查看节点日志：
```bash
ros2 run social_mpc_nav vlm_integration_node --ros-args --log-level debug
```

查看检测到的bounding boxes：
```bash
ros2 topic echo /darknet_ros_3d/bounding_boxes
```

## 下一步工作

1. 实现完整的VLM API HTTP客户端
2. 解析VLM响应并提取导航约束
3. 将VLM分析结果集成到MPC控制器
4. 添加图像输入支持（如果VLM支持视觉输入）
