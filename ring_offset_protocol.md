# 串口新增帧：圆环横向偏移 (type = 0x03)

## 帧格式

```
AA FF 03 02 [offset_H] [offset_L] [checksum]
```

| 字段 | 长度 | 说明 |
|------|------|------|
| `AA FF` | 2B | 帧头（与现有协议一致） |
| `03` | 1B | 帧类型（新增） |
| `02` | 1B | 数据长度 |
| `[offset_H][offset_L]` | 2B | 有符号 int16，**大端序**，单位 **cm**，右正左负 |
| `[checksum]` | 1B | 前6字节之和 & 0xFF |

## 示例

圆环在飞机右侧 30cm：
```
AA FF 03 02 00 1E 44
```

圆环在飞机左侧 15cm：
```
AA FF 03 02 FF F1 E4
```

对齐（偏移 0cm）：
```
AA FF 03 02 00 00 06
```

## 行为说明

- 雷达检测到圆环时，**10Hz 持续发送**此帧
- 圆环丢失后**自动停发**，飞控保持当前位置即可
- 飞控只需根据 offset 值修正**横向位置**，高度和前进方向不变

## 飞控接收示例（伪代码）

```c
if (frame_type == 0x03 && data_len == 2) {
    int16_t offset_cm = (int16_t)((data[0] << 8) | data[1]);  // 大端
    float offset_m = offset_cm / 100.0f;
    // offset_m > 0: 圆环在右，向右修正
    // offset_m < 0: 圆环在左，向左修正
    set_lateral_target(offset_m);
}
```

## 坐标约定

```
        前(+X)
          ↑
左(-Y) ←  机体  → 右(+Y)
```

offset 正值 = 圆环在飞机右侧 → 飞机向右飞修正  
offset 负值 = 圆环在飞机左侧 → 飞机向左飞修正
