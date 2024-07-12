# interface

## 自定义消息

### 下位机发送给上位机

#### 传感器：

| 名称         | 数据类型 | 说明 |
| ------------ | -------- | ---- |
| fire         |          |      |
| smoke        |          |      |
| coal_gas     |          |      |
| co           |          |      |
| formaldehyde |          |      |
| voltage      |          |      |
| light        |          |      |
| temperature  |          |      |
| humidity     |          |      |
| pm1          |          |      |
| pm2          |          |      |
| pm10         |          |      |

#### 图片：

| 名称   | 数据类型 | 说明 |
| ------ | -------- | ---- |
| height | int32    |      |
| width  | int32    |      |
| step   | int32    |      |
| data   | string   |      |

#### 地图：

| 名称       | 数据类型 | 说明 |
| ---------- | -------- | ---- |
| resolution | float32  |      |
| width      | uint32   |      |
| height     | uint32   |      |
| x          | float64  |      |
| y          | float64  |      |
| data       | int8[]   |      |

#### 位置：

| 名称 | 数据类型 | 说明 |
| ---- | -------- | ---- |
| x    | float64  |      |
| y    | float64  |      |



### 上位机发送给下位机

#### 传感器报警阈值：

| 名称         | 数据类型 | 说明 |
| ------------ | -------- | ---- |
| fire         |          |      |
| smoke        |          |      |
| coal_gas     |          |      |
| co           |          |      |
| formaldehyde |          |      |
| voltage      |          |      |
| light        |          |      |
| temperature  |          |      |
| humidity     |          |      |
| pm1          |          |      |
| pm2          |          |      |
| pm10         |          |      |

#### 动作指令

| 名称      | 数据类型 | 说明 |
| --------- | -------- | ---- |
| motion_id |          |      |
| x         |          |      |
| y         |          |      |
| angle     |          |      |
| algorithm |          |      |

