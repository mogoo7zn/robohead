# 实机部署（树莓派 + STM32）

目标平台：**Raspberry Pi 5（8GB）+ Ubuntu Server 24.04 LTS ARM64 + ROS 2 Jazzy**，
底层 **STM32F407**（UART 串口通信），两个 USB UVC 相机（定位相机 + 方块相机）。

详细的系统安装步骤（烧录系统、装 ROS 2、SSH 配置等）见
[design/deployment.md](design/deployment.md)，本文只讲**从零到跑起来**的最短路径。

## 什么是 ros2\_ws？

`ros2_ws` = ROS 2 **workspace**（colcon 工作空间），是 ROS 2 的标准工程结构，
只在树莓派上使用（Mac 开发完全不碰它）。里面是 3 个 ROS 2 包，**全部有具体代码**：

| 包                     | 内容                                                                                           |
| --------------------- | -------------------------------------------------------------------------------------------- |
| `robogame_interfaces` | 自定义 msg：`Pose2D` / `BlockDetection` / `MissionStatus`（CMake 编译）                              |
| `robogame_adapters`   | 实机相机后端：`CameraMarkerDetector`（AprilTag）、`CameraBlockDetector`（HSV）、图像转换（numpy，不用 cv\_bridge） |
| `robogame_bringup`    | 实机入口 `mission_node` + `mission.launch.py`                                                    |

`mission_node` 做的事：用 `RealClock` + `SerialTransport` + 两个相机后端装配**同一个**
`MissionStack`，订阅 `/camera_localization/image_raw` 与 `/camera_block/image_raw`，
以 50Hz 循环驱动 `stack.tick()`，约 1Hz 发布 `MissionStatus` 面板快照。

## 部署步骤

```bash
# 0. 树莓派装好 Ubuntu 24.04 + ROS 2 Jazzy（见 design/deployment.md §44–§54）

# 1. 取代码并安装核心包
git clone <repo> ~/robogame && cd ~/robogame
pip install -e .          # 安装 core / mock 两个包

# 2. 编译 ROS 2 工作空间
#    本机无 ROS 时用 Docker：make ros-build
#    Pi 上直接：
cd ros2_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install && source install/setup.bash && cd ..

# 3. 配置硬件（config/hardware.yaml，出厂为 memory/TODO）
#    mcu.transport: serial
#    mcu.serial.device: /dev/serial0
#    mcu.serial.baudrate: 115200

# 4. 实测场地数据（config/real_field.yaml）
#    所有 marker 坐标、塔位、start_pose 必须现场测量填写，
#    残留 TODO 会被 check-config 拦下。

# 5. 预检 + 启动
make run                        # = bash scripts/start_robot.sh（默认 real_field.yaml）
#    或指定场地文件：
bash scripts/start_robot.sh real_field.yaml
#    或直接 ros2 launch：
ros2 launch robogame_bringup mission.launch.py \
    config_dir:=. field_file:=real_field.yaml control_period:=0.02
```

## 开机自启（比赛不依赖 SSH）

```bash
sudo cp deploy/robogame.service /etc/systemd/system/
sudo touch /opt/robogame/ENABLE && sudo systemctl enable --now robogame
# 查看：journalctl -u robogame -f
```

注意：service 文件假定仓库位于 `/home/pi/robogame`、以 `pi` 用户运行；
路径不同需先改 `deploy/robogame.service` 里的 `WorkingDirectory` / `ExecStart`。
服务由 `/opt/robogame/ENABLE` 标志文件守门——存在才拉起，防止调试时抢跑。

启动流程满足完全自主要求：上电 → systemd 拉起 → SELF\_CHECK →
等待物理启动按钮（START\_EVENT，走 STM32 GPIO，不依赖网络）→ 自主比赛。

## 赛前检查清单

- [ ] `make check-config` 全绿（无 TODO 残留、路线图连通）
- [ ] `python scripts/gen_protocol_header.py --check` 通过（协议未漂移）
- [ ] real\_field.yaml 所有数值为本场实测值
- [ ] 串口 `/dev/serial0` 可用，MCU 心跳正常（MissionStatus 里 `watchdog_healthy=true`）
- [ ] 两个相机话题有图像输出
- [ ] 急停开关（硬件级，直接切断执行器电源）可用
- [ ] 满电；Pi 供电在电机满载时不掉压

## 常见问题

| 现象                                 | 排查                                                  |
| ---------------------------------- | --------------------------------------------------- |
| mission\_node 起不来，ImportError core | 没装核心包：`pip install -e .`（在仓库根目录）                    |
| MCU 心跳超时 → SAFE\_STOP              | 检查 `/dev/serial0`、波特率、STM32 固件是否包含最新协议头             |
| 相机无检测                              | 确认话题名与 launch 参数一致；`v4l2-ctl --list-devices` 检查设备   |
| 定位置信度持续走低                          | marker 没识别到：检查 tag 打印质量、`localization_camera` 内参与外参 |
| 巡线丢失（LINE\_STATE=lost）             | 线传感器标定 / 光照；必要时下发新的 `CMD_LINE_FOLLOW_CONFIG`        |
| check-config 报 marker 缺失           | 路线图引用了 field.yaml 中不存在的 marker，补齐或改路线               |

