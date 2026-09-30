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

timestamp_ns 默认 Unix 纳秒，time_domain=`unix`；其他时钟需显式标识。实时过期判断仍基于本机收到数据后的单调时间。发送端必须抛弃积压的旧遥测，不能用旧位置持续刷新状态。

无人机 HUD 的解锁状态和飞行模式来自飞控 `/mavros/state`，不同于任务机的实物/仿真运行模式。`VehicleState.fcu_state_valid` 仅在收到 2.5 秒内且 `connected=true` 时置位；此时 `armed` 和 `flight_mode` 才可显示。飞控未连接、状态过期、旧代理未提供字段或 3DG 连接中断时显示“未知”。即使 Odin 里程计尚未就绪，代理仍发送不带有效位姿的飞控状态；位姿过期后不能用新飞控状态延长旧位姿的有效期。

代理 Hello 声明 `time-sync-v1` 后，3DG 在连接时及每 60 秒发送 `GET_TIME`。代理最终 `CommandResult` 的 Envelope 时间戳作为任务机采样时间；3DG 用本机请求时刻与单调往返时间估算时差，往返超过 2 秒时丢弃该样本。绝对时差达到 10 秒才发送 `SET_TIME`，`unix_time_ns` 为发令时 3DG 电脑的 Unix 纳秒时间。任务机仅在单独启用校时且具有系统时间权限时执行，结果经回执反馈，成功后立即重新测量。重复校时至少间隔 60 秒；协议不提供远程任意命令。

## 任务机运行模式

任务机 Hello 声明 `task-mode-v1` 后，3DG 的“任务机 → 运行模式”显示 `CompanionStatus.task_mode`，并允许发送 `SET_TASK_MODE`，其 `task_mode` 只能是 REAL 或 SIMULATION。代理将选择写入状态目录的 `task_mode.json`；无文件的旧安装默认实物模式。运行会话与记录的选择不一致，或两类会话同时运行时报告 MODE_CONFLICT。

跨模式切换要求相关会话均停止，飞控明确报告已解锁时拒绝切换。模式选择不会启动或停止任何进程。SIMULATION 下代理拒绝 3DG 的实物 Odin/EGO 启动；仿真主机和端侧 `simproxy` 仍按现有仿真流程启动。旧代理未声明能力时，3DG 禁用模式切换。

## Odin 启动模式与原生地图

任务机 Hello 声明 `odin-start-modes-v1` 后，3DG 才开放新启动弹窗。`GET_ODIN_MAPS` 返回任务机允许目录中通过 `MAPV0001` 文件头检查的原生 `.bin` 地图及其不透明 ID；PCD 下载副本不在列表中。`START_ODIN` 的 `odin_start` 指定模式 0（里程计）、1（SLAM 建图）或 2（重定位）；模式 2 必须带刚从列表取得的地图 ID。代理在执行前再次解析 ID 和文件状态，只写 Odin YAML 中的模式及重定位地图路径，不接受客户端传来的路径。未携带 `odin_start` 的旧客户端仍按现有配置启动模式 0/1。

重定位时，收到里程计和点云只表示链路已启动；只有新收到 Odin 发布的 `odom` 到 `map` TF 才报告重定位成功。失败或超时时保留明确状态，不把请求已受理当作定位成功。当前 3DG 实时绘制仍使用配置的 `odom` / `map_id`，地图选择本身不自动把原生地图点云加载到桌面视图。

## 点云

`point_data` 每点 16 字节：小端 float32 x/y/z（12 字节）+ uint8 r/g/b/reserved（4 字节）；无填充、无首版压缩。长度必须等于 point_count*16，最多 300000 点/帧，单消息最多 8 MiB。非有限点被过滤。

显示通道是可丢帧实时通道，客户端最多 10 Hz 派发并保留最新待显示帧。`snapshot=true` 表示替换当前累积地图（非分片快照）；false 表示把本帧并入体素地图。增量帧可能被丢弃，因此客户端累积不保证完整。可靠完整地图应通过 SAVE_MAP + GET_FILE 获取机载 PCD，而非依赖实时通道拼接。建议机载降采样后传输。

## EGO 与人工航线

### EGO 栅格图层

客户端 Hello capability `ego-voxel-grid-v1` 表示支持 `Envelope.grid`（字段 22）。这是 v1 的可选扩展；只向声明支持的客户端发送，原有字段编号不变。

`VoxelGrid` 包含 `frame_id`、`map_id`、`resolution_m`、占据体素中心 `centers` 和 `inflated`。完整快照替换旧栅格；空中心列表清除栅格。单帧最多 50000 个体素，分辨率范围 0.02–5 m，坐标必须有限且与当前地图匹配。代理需限定发送范围；不能把被省略或超范围区域解释为空闲。

客户端 Hello 同时声明 `ego-voxel-grid-delta-v1` 后，代理在首次连接、地图属性变化、增量不划算及最长 60 秒时发送完整快照，其他时候发送 `delta=true` 的 `added` / `removed`。完整快照带 `version`，增量带 `base_version` 与递增 `version`。客户端必须按序应用，版本不匹配时清空旧栅格并重连 `/cloud` 获取完整快照；增量不得作为可丢弃的最新帧合并。旧客户端仍接收完整快照，最多 0.2 Hz。

任务机适配使用 `/grid_map/occupancy_inflate`（`inflated=true`），读取实际 EGO `grid_map/resolution`；不能使用 3DG 实时点云显示体素大小代替。默认最多 1 Hz 转发最新更新；源栅格仍新鲜但内容不变时发送空增量刷新有效期。

3DG 左侧有“EGO 栅格地图”独立开关，默认使用琥珀色、不透明的体素方块显示，方块留细缝以辨认网格。在“设置 → 3DG 配置”可调整本地栅格颜色和不透明度（0–100%），保存后立即应用并在下次启动时恢复。显示参数不参与碰撞计算，也不发送到任务机。实时点云可合并待显示帧；栅格更新必须按序应用。断连、切换会话或超过 20 秒未收到新更新时清除栅格，避免把旧障碍显示为实时数据。此图层不写入参考 PCD，也不改变 EGO 规划算法。

PlannerTrajectory 是机载采样好的曲线，不传 ROS B-spline 消息原始字节。有效曲线至少 2 点、最多 10000 点，time_from_start_s 非负且严格递增。新 trajectory_id 替换旧曲线。FAILED/CANCELLED 清除旧曲线。若规划不变，任务机仍需周期重发当前有效曲线（3 秒超时隐藏）。

MissionPlan 是人工航点任务，最多 1000 点，含位置、航向、全局速度、类型、停留时间、云台俯仰和变焦。`Waypoint.hold_s` 映射 YAML `dwell_time`；缺省类型为 `Pass_through`，缺省变焦为 1，显式变焦 0 仍需保留。UPLOAD_MISSION 仅提交任务，不隐含起飞或执行。任务机检查坐标系、地图、版本、航点类型与速度并明确应答。

GET_YAML_DOCUMENT 使用 `yaml_document_id=route` 时返回任务机选中的 EGO YAML 航点：存在 `active_mission.json` 时读取已上传的独立快照，否则读取基础 `astra_ego.yaml`。3DG 将其载入同一编辑列表；用户修改后通过 UPLOAD_MISSION 生成新快照并在成功后读回，手动刷新则以任务机当前版本替换本地编辑内容。route revision 包含所选文件路径与内容；上传和启动携带读取时的 revision，版本变化则拒绝写入。代理仅在未解锁、航点控制器未运行、版本一致且参数有效时保存上传快照；Odin 定位可继续运行。`PATCH_YAML_DOCUMENT` 仍保留给旧客户端，但统一编辑界面不再使用该入口。

## 配置与视频

SET_CONFIG 携带 Config；GUI 保存配置只写本机，弹窗“发送当前本地配置到任务机”才发机载命令（弹窗尚未保存的编辑值不会发送）。GET_CONFIG 和 Config 消息定义预留，当前 UI 以本地配置为准。

VIDEO_START/STOP 控制机载流生产。VideoStreamInfo 携带 rtsp/http(s) URL；3DG 收到 `running=true` 和有效地址后自动播放，画面默认位于三维视图左下角，可拖动，并从右上角等比例缩放。媒体编码与播放器解码能力有关，建议机载 H.264 + RTSP 后实际联调。收到停止状态或控制连接失联时本机播放停止并隐藏画面。

## 文件下载

GET_FILE 的 file_id 由任务机提供（可写在 SAVE_MAP 完成说明中）。任务机在 `/files` 回 FileChunk，Envelope.request_id 对应 GET_FILE，每块最多 1 MiB、文件最多 512 MiB。offset 从 0 连续；每块 total_size 一致，最后一块 last=true，sha256 为整个文件 SHA-256 的 32 字节原始值（不是十六进制文本）。首版要求有序传输，不支持断点续传。文件名由地面站用户选择，远端 name 不作为本地路径。

文件 30 秒无数据、失联、长度/偏移或摘要错误时取消临时文件；完整校验后才原子落盘。

## 任务机接入范围

后续任务机代理负责 ROS 订阅、TF、Odin1 / EGO 进程管理、视频服务、记录及 PCD 文件生成。本仓库仅提供协议和地面站，不包含上述代理，不执行远程任意 shell 命令。没有内置身份认证，部署前必须在可信网络或认证网关后使用。
