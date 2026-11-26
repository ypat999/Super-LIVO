# solution_d_fastlivo2 — FAST-LIVO2 对照臂

把 `third_party/FAST-LIVO2-ROS2` 适配到本机（RK3588 + Livox MID360 + 海康 MV-CB013），
作为与方案 A（`solution_a_lightweight`，Super-LIVO + 影子轨迹视觉修正）对照的官方
FAST-LIVO2 紧耦合实现。目录结构仿照 A：只放「本机标定 / 启动方式 / 构建脚本」，
算法源码仍留在 `third_party/FAST-LIVO2-ROS2`，不复制。

## 目录

```
solution_d_fastlivo2/
├── config/
│   ├── fastlivo2_mid360_hik.yaml   # 主参数：话题、外参、雷达、VIO、落盘
│   └── camera_hik_pinhole.yaml     # vikit camera_loader 读取的相机内参
├── launch/
│   └── run_d.launch.py             # 参数服务器 + fastlivo_mapping（去掉 republish）
├── scripts/
│   ├── build_d.sh                  # Sophus + U-AMC vikit + colcon 构建（幂等）
│   ├── calib_time_offset.py        # 现场标定「雷达钟 − 相机钟」
│   └── run_d_live.sh               # 一键实时运行（仿 tools/visual_a_live.sh）
├── .deps/                          # Sophus / vikit 本地依赖（不装系统）
└── ws/                             # colcon 工作区（src 为符号链接，install 为产物）
```

## 相对上游的关键改动

1. `third_party/FAST-LIVO2-ROS2/src/main.cpp`：原文件用空 `Node::SharedPtr` 构造
   `ImageTransport`，必崩。现创建 `laserMapping_aux` 节点作为入参（`LIVMapper`
   内部自建 `laserMapping` 节点并接管全部 IO）。
2. 相机链路：本机相机发布 raw `bgr8`，启动文件不再挂 `image_transport/republish`。
3. 相机模型：`PinholeCamera` 只做针孔投影，因此按 `camera.dist_coeffs` 先
   `initUndistortRectifyMap` 去畸变，再投影，与 FAST-Calib 的 K/D 一致。
4. 时钟：相机 `header.stamp` 是宿主墙钟（≈1.7955e9），雷达 `header.stamp` 是
   Mid360 设备钟（≈1.5902e9）。`time_offset.img_time_offset` 需填约 `-2.0538e8`，
   由 `scripts/calib_time_offset.py` 每次启动现场标定（中位数，只含传感器延迟），
   并开启 `rolling_shutter.td_estimate_enable` 跟踪慢漂。
5. 外参：`Rcl/Pcl` 直接复用 A 的 FAST-Calib + 4 孔心重解结果（RMSE 3.2 mm），
   保证两条链路看到的是同一套标定。
6. `third_party/rpg_vikit_rational_polynomial/vikit_ros/include/vikit/params_helper.h`：
   `getRemoteParam` 里 `wait_for_service` 原为 100 ms，本机实测进程刚启动时
   service discovery 需约 2.5 s，抢跑会返回空字符串并抛
   `Camera model not correctly specified.`。已改为 10 s。vikit 是上游克隆
   （build_d.sh 自动 clone），该修改以 `patches/vikit-wait-for-service.patch`
   维护，`build_d.sh` 克隆后自动应用（幂等）。
7. 运行时依赖：`vikit_common` 是 plain CMake 装在 `.deps/vikit`，不在
   `ws/install` 的 setup.bash 里。`run_d.launch.py` 已通过 `additional_env`
   注入 `LD_LIBRARY_PATH`，`run_d_live.sh` 也额外 export，直接 `ros2 launch` 也能跑。

## 构建

```bash
cd /home/ztl/git/sl_ws/Super-LIVO/solution_d_fastlivo2
./scripts/build_d.sh          # 首次会下载 Sophus deb、克隆 U-AMC vikit（需 GitHub）
JOBS=2 ./scripts/build_d.sh   # 限制并行
REBUILD=1 ./scripts/build_d.sh
```

产物：`ws/install/fast_livo/lib/fast_livo/fastlivo_mapping`。

## 运行

```bash
./scripts/run_d_live.sh                       # 输出 ~/slam_data/output/live_d_MMDD_HHMMSS
./scripts/run_d_live.sh /path/to/out
IMG_TIME_OFFSET=-205383152 ./scripts/run_d_live.sh   # 跳过现场标定
USE_RVIZ=1 ./scripts/run_d_live.sh
```

Ctrl-C 停止（FAST-LIVO2 在退出时 `savePCD()` 才写图，`kill -9` 会丢图）。

产物（脚本把 `third_party/FAST-LIVO2-ROS2/Log` 软链到本次输出目录）：

- `<out>/log/pcd/all_raw_points.pcd`：彩色整图
- `<out>/log/pcd/all_downsampled_points.pcd`：`filter_size_pcd=0.02` 降采样
- `<out>/log/result/scene_d.txt`：`t x y z qx qy qz qw`
- `<out>/app.log`

## 与方案 A 对比

公平起见建议同一段数据跑两遍。两种方式：

1. **录 bag 回放（推荐，可复现）**

   ```bash
   # 先录一段（传感器已在跑）
   ros2 bag record -o /home/ztl/slam_data/bag_demo /livox/lidar /livox/imu /left_camera/image
   # A：实时链路回放（A 的 offset 在回放时必须手填实测值）
   # D：先 IMG_TIME_OFFSET=<实测值> 再启动，bag 侧用 0.5× 回放
   ros2 bag play /home/ztl/slam_data/bag_demo --rate 0.5
   ```

2. **实时轮流跑**：先 `tools/visual_a_live.sh`，再 `scripts/run_d_live.sh`，
   两段都走同一路线；比较彩色点云的错色/纹理、轨迹 `evo`、CPU 占用。

对比要点：

- **几何**：`evo_traj` 比轨迹（D 的 `scene_d.txt` 与 A 的 `evo` 输出，A 需开
  `evo.pose_output_en: true`）。
- **着色**：同视角看 `all_raw_points.pcd` 与 A 的 `super_livo_visual_a_colored.pcd`，
  重点看运动方向上的错色（A 已知存在 +48 ms 系统性相位偏置）。
- **实时性**：两脚本的 `app.log` 均有逐帧耗时；R 上可 `top -H` 看 `fastlivo_mapping`
  的 CPU 与内存（FAST-LIVO2 的体素地图 + 视觉 patch 比 A 重得多）。

## 已知限制

- FAST-LIVO2 输出路径在编译期由 `ROOT_DIR` 写死，脚本用软链方式隔离到输出目录。
- `Log/` 若被改成真实目录（需手工创建），脚本不再建软链，产物会落到
  `third_party/FAST-LIVO2-ROS2/Log`。
- 本机相机读出时间未标定，`rolling_shutter.enable` 保持 `false`。
