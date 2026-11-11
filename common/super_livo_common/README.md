# super_livo_common

方案 A/B 共用的纯算法头文件库（无编译单元、无 ROS 依赖）。目的只有一个：
**把"同名不同实现"的口径收敛到一处**，避免改了一边忘了另一边导致 A/B 实验结论不可比。

## 内容

| 头文件 | 提供 | 谁在用 |
| --- | --- | --- |
| `super_livo_common/degeneracy_signature.hpp` | `DegeneracySignature`（`/lio/degeneracy` 载荷解析、`valid()`、`severity()` 对数映射）+ `DegeneracyScale`（`cond_lo`/`cond_hi`/`timeout_sec` 默认值） | A `visual_a_node`、B `visual_b_node` |
| `super_livo_common/anisotropic_prior.hpp` | `directionalPositionInformation(base, sev, dir, σ_healthy, σ_degen)`：退化方向松开、垂直方向收紧，再按 sev 插值 | A `visual_update`、B `buildLidarPrior` |
| `super_livo_common/measurement_noise.hpp` | `reprojectionVariance(σ_pix, f, σ_anchor, 1/z)` = `σ_pix² + (f·σ_anchor/z)²`；`associationVariance(pixel_radius)` | A `ReprojectionConstraint`、B `LidarVisualConstraint` |

## 刻意不放进来的东西

- **估计器**：A 是外挂 `visual_update` 回灌宿主 ESIKF，B 是自包含 `unified_estimator`，
  两套生命周期不同，合并会互相污染（见根 README 的定位说明）。
- **前端 / 约束构造**：A（FAST+PyrLK + 逐帧 LiDAR 深度）与 B（Patch + 特征轨迹 + MSCKF
  + 持久锚点）是不同算法，仅"噪声口径"共享，不共享实现。
- **ROS 接线**：节点里的参数声明、话题、日志格式各自维护，不抽公共基类。

## 编译

纯头文件，`ament_cmake` 只导出 include 目录：

```bash
colcon build --packages-select super_livo_common
```

消费者（A/B 的 `CMakeLists.txt`）：

```cmake
find_package(super_livo_common REQUIRED)
target_include_directories(<target> PRIVATE ... ${super_livo_common_INCLUDE_DIRS})
```
