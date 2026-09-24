# 3DG / 任务机协议 v1

任务机作为 WebSocket 服务端（默认 `ws://127.0.0.1:8765`），3DG 客户端；每个二进制 WebSocket 消息恰好一个 `mission.Envelope`，无额外长度头。不是 gRPC。

| 路径 | 消息 |
|---|---|
| `/control` | Hello / Heartbeat / VehicleState / CompanionStatus / PlannerTrajectory / Command / CommandResult / Config / LogEntry / VideoStreamInfo |
| `/cloud` | CloudFrame / VoxelGrid |
| `/files` | FileChunk |

1. 控制连接建立后 3DG 发 Hello。服务端返回 Hello，协议版本 1，非空 session_id、从 1 递增 sequence。之后客户端才打开 cloud/files；服务端三个通道使用同一个服务端 session_id。
2. 两端各自使用自己的 session_id；在同一发送端、同一通道内 sequence 严格递增。客户端重连会重新握手。服务端重启要更换 session_id；bulk 通道不额外发送 Hello。重复或倒序消息丢弃。
3. 两端每秒发送心跳；客户端控制通道 6 秒收不到任何有效消息就断开并每 3 秒尝试重连。bulk 单独重连。命令不自动重发，超过 30 秒结果未知。
4. 请求含 UUID request_id；服务端回相同 request_id 的 ACCEPTED，再回 SUCCEEDED 或 FAILED，detail 是用户可读说明。任务机必须自行实现幂等去重。状态面板只信状态消息，不把 ACCEPTED 当作运行成功。

## 坐标和时间

单位米、秒、弧度，右手系，Z 向上；所有地图、飞机、速度、航线和 EGO 曲线必须先在任务机转换到相同 `frame_id` / `map_id`。默认 `odom` / `local-map`。姿态四元数 xyzw，表示机体 FLU 到约定世界坐标的旋转；不能直接发送 NED/FRD。地面站不隐式计算 TF 或经纬度转换。

timestamp_ns 默认 Unix 纳秒，time_domain=`unix`；其他时钟需显式标识。第一版实时过期判断基于本机收到数据后的单调时间，不做跨设备时钟同步或延迟补偿。发送端必须抛弃积压的旧遥测，不能用旧位置持续刷新状态。

## 点云

`point_data` 每点 16 字节：小端 float32 x/y/z（12 字节）+ uint8 r/g/b/reserved（4 字节）；无填充、无首版压缩。长度必须等于 point_count*16，最多 300000 点/帧，单消息最多 8 MiB。非有限点被过滤。

显示通道是可丢帧实时通道，客户端最多 10 Hz 派发并保留最新待显示帧。`snapshot=true` 表示替换当前累积地图（非分片快照）；false 表示把本帧并入体素地图。增量帧可能被丢弃，因此客户端累积不保证完整。可靠完整地图应通过 SAVE_MAP + GET_FILE 获取机载 PCD，而非依赖实时通道拼接。建议机载降采样后传输。

## EGO 与人工航线

### EGO 栅格图层

客户端 Hello capability `ego-voxel-grid-v1` 表示支持 `Envelope.grid`（字段 22）。这是 v1 的可选扩展；只向声明支持的客户端发送，原有字段编号不变。

`VoxelGrid` 包含 `frame_id`、`map_id`、`resolution_m`、占据体素中心 `centers` 和 `inflated`。每帧是当前可视局部范围的完整快照，替换旧栅格，不做历史累积；空中心列表清除栅格。单帧最多 50000 个体素，分辨率范围 0.02–5 m，坐标必须有限且与当前地图匹配。代理需限定发送范围；不能把被省略或超范围区域解释为空闲。

任务机适配使用 `/grid_map/occupancy_inflate`（`inflated=true`，建议默认）或 `/grid_map/occupancy`（`inflated=false`），读取实际 EGO `grid_map/resolution`；不能使用 3DG 实时点云显示体素大小代替。建议 2 Hz 刷新，包括无变化时重发有效快照。任务机订阅/转发尚待实现。

3DG 左侧新增“EGO 栅格地图”独立开关，使用琥珀色体素方块显示，方块留细缝以辨认网格，不参与碰撞计算。实时点云与栅格各保留最新一帧，不互相覆盖。断连、切换会话或超过 3 秒未更新时清除栅格，避免把旧障碍显示为实时数据。此图层不写入参考 PCD，也不改变 EGO 规划算法。

PlannerTrajectory 是机载采样好的曲线，不传 ROS B-spline 消息原始字节。有效曲线至少 2 点、最多 10000 点，time_from_start_s 非负且严格递增。新 trajectory_id 替换旧曲线。FAILED/CANCELLED 清除旧曲线。若规划不变，任务机仍需周期重发当前有效曲线（3 秒超时隐藏）。

MissionPlan 是人工航点任务，最多 1000 点，含位置、航向、速度和停留时间。UPLOAD_MISSION 仅提交任务，不隐含起飞或执行。任务机应检查地图版本、飞行约束、障碍物和权限，明确应答。

## 配置与视频

SET_CONFIG 携带 Config；GUI 保存配置只写本机，弹窗“发送当前本地配置到任务机”才发机载命令（弹窗尚未保存的编辑值不会发送）。GET_CONFIG 和 Config 消息定义预留，当前 UI 以本地配置为准。

VIDEO_START/STOP 控制机载流生产。VideoStreamInfo 携带 rtsp/http(s) URL；3DG 收到后不自动播放，需用户打开悬浮视频窗口。媒体编码与播放器解码能力有关，建议机载 H.264 + RTSP 后实际联调。收到停止状态或控制连接失联时本机播放停止。

## 文件下载

GET_FILE 的 file_id 由任务机提供（可写在 SAVE_MAP 完成说明中）。任务机在 `/files` 回 FileChunk，Envelope.request_id 对应 GET_FILE，每块最多 1 MiB、文件最多 512 MiB。offset 从 0 连续；每块 total_size 一致，最后一块 last=true，sha256 为整个文件 SHA-256 的 32 字节原始值（不是十六进制文本）。首版要求有序传输，不支持断点续传。文件名由地面站用户选择，远端 name 不作为本地路径。

文件 30 秒无数据、失联、长度/偏移或摘要错误时取消临时文件；完整校验后才原子落盘。

## 任务机接入范围

后续任务机代理负责 ROS 订阅、TF、Odin1 / EGO 进程管理、视频服务、记录及 PCD 文件生成。本仓库仅提供协议和地面站，不包含上述代理，不执行远程任意 shell 命令。没有内置身份认证，部署前必须在可信网络或认证网关后使用。
