# VLM触发条件配置说明

## 概述

VLM集成节点现在支持可配置的智能触发机制，只在环境发生**明显变化**时才触发VLM调用，避免过于频繁的调用。

## 核心机制

### 1. 节流机制 (Throttle)

**参数**: `min_trigger_interval_sec`

- **作用**: 设置两次VLM调用之间的最小时间间隔
- **默认值**: 10.0秒
- **推荐值**: 
  - 快速VLM: 5-10秒
  - 标准VLM: 10-20秒
  - 慢速VLM: 20-30秒

**示例**:
```yaml
min_trigger_interval_sec: 15.0  # 至少间隔15秒才触发
```

### 2. 基础触发条件

#### 新检测触发 (`trigger_on_new_detection`)
- **作用**: 检测到新的bounding box时触发
- **默认**: `true`
- **注意**: 这个条件会结合下面的高级条件来判断是否"显著变化"

#### 场景变化触发 (`trigger_on_scene_change`)
- **作用**: 机器人移动超过阈值时触发
- **参数**: `scene_change_threshold` (默认0.3米)
- **默认**: `true`

#### 周期性触发 (`trigger_periodic`)
- **作用**: 定期触发（即使没有变化）
- **默认**: `false` (通常不推荐，除非需要定期检查)

### 3. 高级触发条件（显著变化检测）

这些条件帮助识别**真正有意义的环境变化**：

#### 3.1 新物体类型 (`trigger_on_new_object_type`)
- **作用**: 当新的物体类别出现或消失时触发
- **示例场景**:
  - 检测到第一个"person" → 触发
  - "car"出现 → 触发
  - "person"消失 → 触发
- **默认**: `true`
- **推荐**: 保持开启，这是最明显的场景变化

#### 3.2 物体数量显著变化 (`trigger_on_object_count_change`)
- **作用**: 当物体数量变化超过阈值时触发
- **参数**: `object_count_change_threshold` (默认2)
- **示例**:
  - 从2个人变成4个人 → 触发（变化=2）
  - 从3个人变成4个人 → 不触发（变化=1 < 阈值2）
- **默认**: `false`
- **适用场景**: 人群密度变化重要的场景

#### 3.3 物体位置显著变化 (`trigger_on_object_position_change`)
- **作用**: 当物体移动超过阈值距离时触发
- **参数**: `object_position_change_threshold` (默认1.0米)
- **示例**:
  - 人从(2, 3)移动到(3.5, 3) → 触发（移动1.5米 > 1.0米）
  - 人从(2, 3)移动到(2.3, 3) → 不触发（移动0.3米 < 1.0米）
- **默认**: `true`
- **推荐**: 保持开启，检测动态场景变化

#### 3.4 物体类型组合变化 (`trigger_on_object_type_combination_change`)
- **作用**: 当物体类型组合发生变化时触发（即使数量相同）
- **示例**:
  - 从[person, person]变成[person, car] → 触发
  - 从[person, person]变成[person, person, person] → 不触发（类型组合相同）
- **默认**: `true`
- **适用场景**: 场景语义变化重要的场景

## 配置示例

### 示例1: 保守配置（减少触发频率）
```yaml
min_trigger_interval_sec: 20.0  # 至少间隔20秒
trigger_on_new_object_type: true
trigger_on_object_count_change: false
trigger_on_object_position_change: true
object_position_change_threshold: 2.0  # 物体移动2米以上才触发
trigger_on_object_type_combination_change: false
```

### 示例2: 敏感配置（快速响应变化）
```yaml
min_trigger_interval_sec: 5.0  # 至少间隔5秒
trigger_on_new_object_type: true
trigger_on_object_count_change: true
object_count_change_threshold: 1  # 数量变化1个就触发
trigger_on_object_position_change: true
object_position_change_threshold: 0.5  # 物体移动0.5米就触发
trigger_on_object_type_combination_change: true
```

### 示例3: 平衡配置（推荐）
```yaml
min_trigger_interval_sec: 10.0  # 至少间隔10秒
trigger_on_new_object_type: true  # 新物体类型
trigger_on_object_count_change: false  # 不关心数量变化
trigger_on_object_position_change: true  # 关心位置变化
object_position_change_threshold: 1.0  # 移动1米以上
trigger_on_object_type_combination_change: true  # 关心类型组合变化
```

## 触发逻辑

节点按以下顺序检查触发条件：

1. **节流检查**: 如果距离上次触发时间 < `min_trigger_interval_sec`，直接跳过
2. **新检测检查**: 检查是否有显著变化（按优先级）:
   - 新物体类型出现/消失
   - 物体数量显著变化（如果启用）
   - 物体类型组合变化（如果启用）
   - 物体位置显著变化（如果启用）
3. **场景变化检查**: 检查机器人是否移动超过阈值
4. **触发**: 如果满足条件且通过节流检查，则触发VLM调用

## 日志输出

节点会输出详细的触发信息：

```
[INFO] Significant scene change detected: new object type: person
[INFO] Event triggered: new_detection=1, scene_changed=0 | Filtered 3 bounding boxes | Time since last: 12.34s
[INFO] triggerVLMQuery() called with 3 bounding boxes
[INFO] ✅ Published VLM prompt (1234 characters) to /vlm/prompt
```

如果被节流，会看到：
```
[DEBUG] Trigger throttled: 5.23 seconds since last trigger (min: 10.00)
```

## 调试建议

1. **触发太频繁**: 
   - 增加 `min_trigger_interval_sec`
   - 关闭 `trigger_on_object_count_change`
   - 增加 `object_position_change_threshold`

2. **触发太少**:
   - 减少 `min_trigger_interval_sec`
   - 启用 `trigger_on_object_count_change`
   - 减少 `object_position_change_threshold`

3. **查看详细日志**:
   ```bash
   ros2 run social_mpc_nav vlm_integration_node --ros-args --log-level debug
   ```

## 最佳实践

1. **根据VLM处理速度调整节流间隔**: 如果VLM需要5秒处理，设置至少10秒间隔
2. **优先使用新物体类型检测**: 这是最可靠的显著变化指标
3. **谨慎使用数量变化检测**: 可能在小幅波动时误触发
4. **根据应用场景调整位置阈值**: 
   - 室内导航: 0.5-1.0米
   - 室外导航: 1.0-2.0米
   - 高速场景: 2.0-5.0米
