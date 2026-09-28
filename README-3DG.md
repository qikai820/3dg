# 3DG Mission 第一版

基于 CloudCompare v2.13.2 的 C++ / Qt 桌面实现，不是网页原型。原版程序仍可直接运行；设置 `THREEDG=1` 才切换到地面站界面。

## 启动

在本目录执行：

```bash
./scripts/run-3dg.sh --demo  # 本地模拟点云、飞机与 EGO 曲线，无真实设备操作
./scripts/run-3dg.sh         # 离线地面站，手动连接任务机
./scripts/build-3dg.sh      # 重新编译
```

构建脚本使用当前目录的 `build-local/`，避免沿用搬迁前的 CMake 缓存；旧 `build/` 保留。编译完成后可运行 `./scripts/test-3dg.sh`。如需其他构建目录，给构建、启动和测试脚本统一设置绝对路径 `THREEDG_BUILD_DIR`。默认只编译主程序、核心 I/O 插件和 3DG 测试；并行编译数默认为 2，可通过 `BUILD_JOBS` 调整。本机翻译工具位于 `~/software/Qt/5.15.2/gcc_64`；其他位置可设置 `QT5_LINGUIST_TOOLS_DIR` 指向 `Qt5LinguistToolsConfig.cmake` 所在目录。

模拟开关也在“视图”菜单。演示数据经过同一 Protobuf 解码与校验入口，但不使用外部任务机。真实网络命令往返由独立测试验证。

无人机位置使用内置的 PX4 X500 实体网格（机架、起落架、电机、四副桨叶），按米制原始尺寸显示，随遥测的位置与四元数更新完整姿态。“视图 → 定位无人机”可放大到飞机附近；“适配视图”返回全局。模型一次加载，运行时无需联网或额外模型插件。无效位姿、断连和超过 3 秒未更新会隐藏模型。机体坐标沿用 +X 机头、+Y 左、+Z 上，任务机须提供对应的机体到地图旋转。模型来源、精简过程和 BSD 许可见 `qCC/mission3dg/assets/x500/`；桨叶为静态显示，协议尚未提供转速。

姿态收包后立即更新模型变换并请求重绘，不再等待 100 ms 点云刷新。点云、地图、实际轨迹与 EGO 曲线只在各自数据变化时重建；演示位姿为 50 Hz。配套任务机代理也需更新到独立 50 Hz 位姿转发版本，否则旧代理仍每秒仅发送一次姿态。实际可见帧率受遥测输入、点云处理、网络及显卡性能限制。

## 已实现

- 底层三维视图，浅色半透明浮动卡片：左侧窄图标栏控制场景图层，地图与连接详情按需展开；航点任务位于底部横向缩略条，点击“查看”打开编辑弹窗。顶部工具栏可通过左侧箭头向左收起并再次展开。右侧保留 HUD、任务机状态和操作卡片；1024×710 窗口可用。
- “EGO 栅格地图”位于 EGO 实时规划与实际轨迹之间，九宫格图标可独立开关琥珀色体素显示。采用机载栅格分辨率、完整局部快照替换，空帧/失联/过期清除；不累计到参考地图。演示模式提供模拟栅格，真实 `/grid_map/occupancy_inflate` 转发需后续任务机代理接入。
- 顶部“设置 → 3DG 配置”管理本地连接、地图、点云和显示参数；“任务机 → 任务机 YAML”从机载代理读取已登记的 YAML，按文件和层级自动分组，支持搜索、修改预览、版本检查与保存。“任务机”菜单还包含“日志记录”和“任务启动 → Odin / EGO”。航点数组继续由航点编辑器管理；旧版点云过滤组只读。坐标系/地图/体素变化会切换到对应的累积地图并清除旧航线。
- 顶部最新日志可展开六行、滚动查看历史，并从弹窗开始或停止机载记录。内存最多 2000 条，同时落盘。读旧日志时不强制跳回底部。
- 彩色实时点云与体素累积地图，PCD ASCII / binary / binary_compressed 读取，二进制 XYZRGB PCD 保存。保存坐标信息至同名 `.meta.json`；无元数据的文件按当前配置解释。
- 普通连接默认开启顶部“实时累积”，第一帧立即显示累积地图；左侧“累积 / 参考地图”控制其显示。取消“实时累积”会保留已有地图并暂停累积。加载本地 PCD / PLY 后自动暂停累积，避免覆盖参考地图。地图详情分别显示实时点数和累积 / 参考地图点数。
- 累积地图每 30 秒在有变化时自动保存为 XYZRGB PCD，正常关闭前也会保存；下次启动自动加载并继续累积。文件位于 Qt `AppLocalDataLocation/3dg/maps/`，同名 `.meta.json` 记录坐标系、地图 ID、体素大小和演示/真实模式；运行日志会显示具体路径。新会话保留同一地图的累积点，“清空累积地图”会删除对应的自动保存文件。演示点云与真实点云分别保存。
- PLY 使用原 CloudCompare 加载器。PCD 文件及解压数据各以 256 MiB 为上限，原始点数上限 1000 万；超过 200 万点时均匀抽样，最多显示 200 万点，原文件不变。实时单帧上限 30 万点，累积上限 40 万体素。达到上限不新增体素，已有体素可更新。
- 三维航点添加、拾取 XY + 指定 Z、双击编辑 XYZ/航向/速度/停留、删除/上移、任务 JSON 保存/读取/上传接口。选中表格中的航点会在三维视图显示红 X、绿 Y、蓝 Z 三轴箭头；拖动某一箭头只调整该坐标。黄色为任务航线，白色为实飞轨迹，青色为 EGO 曲线。拾取需要点云有点的位置，不是任意空间射线求交。
- 实时姿态仪、位置、速度、电量；无效/过期遥测不当作有效值显示。EGO 每条新曲线替换上一条，失败/取消/超过 3 秒未更新时清除或隐藏。
- 视频画面默认放在三维视图左下角，无边框和浮窗按钮；拖动画面可移动，拖动右上角沿对角线等比例缩放（16:9，左下角固定）。保存了本地视频地址或收到运行中的机载视频地址时自动播放；无地址时初始隐藏，点击“视图 → 悬浮视频窗口”会显示带提示的空窗口。播放失败或视频结束后窗口保持可见，具体错误写入日志。Qt Multimedia / GStreamer 播放本地视频或 RTSP/HTTP(S)。视频内容不塞进 Protobuf，仅用协议传地址与状态。机载开启/停止和本地播放/停止是不同动作。
- 三个独立 WebSocket 通道；握手、心跳、重连、序号去重、命令 ID 和超时、限流、地图/坐标系一致性检查、文件分块下载和 SHA-256 校验。

## 任务机端边界

任务机代理源码位于 AstraDrone_v1.0 的 `services/mission3dg_agent`。桌面端使用 `yaml-config-v1` 扩展命令读取及提交 YAML 参数，旧版固定连接/显示配置仍可使用。代理默认只读；真实写入需要显式启用，并且运行中的任务进程会阻止修改。保存后要重启对应进程，界面不把磁盘值当作已生效的运行值。

航线是人工编排及可视化，不是碰撞检测或自动避障规划器。安全间距目前仅作为配置接口参数，不在地面站执行安全校验。“结束任务”不是紧急停机。真实飞行前还需要机载坐标转换、控制权限、飞控失联保护、碰撞/动力学约束及实机联调。

首版不集成 ROS，EGO 曲线由机载端采样后发送。PCD 在本机保存与“保存机载地图”是两个独立功能。任务机文件须凭文件 ID 下载，不提供远端路径执行能力。

## 协议与目录

- `qCC/mission3dg/protocol/mission.proto`：稳定的 v1 数据定义。
- `qCC/mission3dg/protocol/README.md`：任务机适配约定。
- `MissionController.cpp`：悬浮界面、绘制、HUD。
- `ControllerActions.cpp`：交互、收包、地图/航线/视频/下载、模拟器。
- `ProtocolClient.cpp`：通信与消息校验；`CloudIO.cpp`：PCD 与点云格式。

## 构建依赖

沿用原工程的 Qt5、CMake、Ninja、CloudCompare 子模块；新增 Qt5 WebSockets、Qt5 Multimedia/Widgets、Protobuf 编译器与开发库。当前机器新增依赖解包到 `.deps/root`，未通过 sudo 更改系统。启动脚本设置对应库和视频插件路径。

其他 Ubuntu 24.04 环境可安装 `libqt5websockets5-dev qtmultimedia5-dev libqt5multimedia5-plugins libprotobuf-dev protobuf-compiler` 后用 CMake 构建 `-DOPTION_3DG=ON`。不构建地面站则指定 `-DOPTION_3DG=OFF`。

### Windows x64 便携包

Windows 构建完成后，使用 `scripts/package-3dg-windows.py --build-dir build-windows --qt-dir /path/to/Qt/5.15.2/mingw81_64 --lrelease /path/to/host/lrelease` 收集 Qt、MinGW 运行库、核心 I/O 插件和着色器。脚本会检查 PE DLL 导入，生成 `artifacts/windows/CloudCompare-3DG-2.13.2-windows-x64.zip` 及 SHA-256 文件。解压后运行 `Start-3DG.cmd`，离线演示运行 `Start-3DG-Demo.cmd`。此交叉编译包仍需在目标 Windows 机器验证界面、视频和任务机连接。

MinGW 交叉编译前，对上游 CCCoreLib 子模块应用 `patches/CCCoreLib-Windows-MinGW-ScalarField.patch`：在工程根目录运行 `git -C libs/qCC_db/extern/CCCoreLib apply "$PWD/patches/CCCoreLib-Windows-MinGW-ScalarField.patch"`。该补丁修复 MinGW 下导入类的内联析构函数编译错误；主仓库仍使用上游子模块提交。

## 验证

```bash
export LD_LIBRARY_PATH="$PWD/.deps/root/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
./scripts/test-3dg.sh
# 需要可用图形显示；完成后退出并保存截图、PCD、航线与 JSON 结果
THREEDG_SMOKE_DIR="$PWD/artifacts/smoke" ./scripts/run-3dg.sh --demo
# 可选：用真实大地图同时测试读取和界面加载
THREEDG_TEST_LARGE_PCD=/path/to/map.pcd ./scripts/test-3dg.sh
THREEDG_SMOKE_LOAD_PCD=/path/to/map.pcd THREEDG_SMOKE_DIR="$PWD/artifacts/large-pcd" ./scripts/run-3dg.sh --demo
```

测试覆盖 PCD 颜色/坐标往返、ASCII/二进制/LZF、异常头、版本/长度/四元数/非有限值、握手/重复序号/错会话及真实本地 WebSocket 命令应答。图形自检输出位于 `artifacts/smoke/`。视频实际 RTSP 摄像头、真实机载网络和飞行安全不在已验证范围内。

运行日志位于 Qt `AppLocalDataLocation/3dg/logs/`，参数用 `QSettings("3DG", "Mission")` 保存。不自动启动网络、不自动下发任务。`ws://` 没有加密；仅在可信隔离网络使用，跨公网须另加认证与 TLS 网关，不要直接暴露端口。
