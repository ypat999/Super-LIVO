# exp —— 实验脚本与配置归档

存放视觉约束（B2/B3/χ² 门控）、退化处理等实验用的一次性脚本、跑批与评估工具。
**这些是从 `/tmp` 抢救出来的快照**：脚本内部仍硬编码 `/tmp/xxx` 作为日志/结果输出路径，
复用时按需改 OUTPUT 目录，不要直接跑（会写到 /tmp 并在系统重启后丢失）。

实验结论与过程见 [../.trae/documents/B2B3-光流旋转约束实验记录.md](../.trae/documents/B2B3-光流旋转约束实验记录.md)。

## 目录

```
exp/
├── scripts/
│   ├── b2/      # B2：光流相对旋转注入（σ 扫描、时间对齐诊断、配置生成）
│   ├── b3/      # B3：18 维通道 + 陀螺积分残差（含雅可比有限差分校核）
│   ├── p1/      # 方案①：B2 加 χ² 门控 + 安全化扫描、NTU 不退化验证
│   └── common/  # 通用：bag 回放、NTU 评估、轨迹比对、配置备份
└── config/      # 实验期配置快照（含已废弃的 latch/force 版本，仅作历史对照）
```

## 关键脚本

### common
| 脚本 | 用途 |
|---|---|
| `run_room.sh` | room bag 回放（Mid-360，`livox_360_visa_room*.yaml`），输出 traj + 日志 |
| `run_ntu_degen.sh` | NTU eee_03 跑批协议：0.5 倍速、`cloud_crop.py` 裁剪、`ros_batch.lock` 互斥 |
| `cloud_crop.py` | 以里程计位置为球心裁点云（NTU 走廊外点过滤，复刻官方配置） |
| `eval_loop.py` / `eval_all.sh` | 批量跑配置组合并汇总终点误差 |
| `cmp_traj.py` | 两条 TUM 轨迹逐 stamp 对比（位姿差 / 旋转差 / yaw 差） |
| `visa_backup.sh` | 备份 `config/` 下的 visa 系列 yaml |

### b2
| 脚本 | 用途 |
|---|---|
| `b2_setup.sh` / `b2_cfg.sh` | 生成 room B2 派生配置（内参/外参/time_offset 注入） |
| `b2_phase1.sh` / `b2_phase2.sh` | 两阶段跑批：先 off vs b2，再 σ 扫描 |
| `b2_one.sh` / `b2_pair.sh` / `b2_pair2.sh` | 单配置 / 成对跑（含并发互斥） |
| `b2_stats.sh` / `b2chk.sh` / `b2.sh` / `b2_d2.sh` | 日志摘要：`[VisRot]` 注入量、接受率、是否存在 |
| `b2_diag.py` | 视觉帧 vs 激光帧时间对齐诊断（slack 窗口、配对成功率） |
| `b2_timecheck.py` | 时间戳检查（图像/点云/IMU 频差、断档） |
| `b2_recref.py` | 复现参考节点缓存逻辑校验 |

### b3
| 脚本 | 用途 |
|---|---|
| `b3_mkcfg.sh` / `b3_mkntu.sh` | 生成 room / NTU 的 B3 配置（不动原 yaml 本体） |
| `b3_batch1.sh` / `b3_batch2.sh` / `b3_grid.sh` | 基线 dt × σ × 门控 的网格跑批 |
| `b3_one.sh` / `b3_ntu.sh` / `b3_smoke.sh` | 单跑 / NTU 跑 / 冒烟 |
| `b3_eval.sh` / `b3_evalall.sh` / `b3_ntueval*.sh` | ATE 评估（官方 T_B_PRISM + combteeth 0.1 s + SE(3) Umeyama） |
| `b3_stat*.sh` / `b3_roomtab.sh` / `b3_ntusum.sh` / `b3_show*.sh` | 日志/结果汇总表 |
| **`b3_bitcmp2.py`** | `enable=false` 与基线的**逐 stamp 逐字节比对**（回归安全的证据） |
| **`b3_verify_h.py`** | 解析雅可比 vs 有限差分互校（曾抓到旧式 `Δt·J_r·J_r` 错误，rel 0.33→1e-9） |
| `b3_fdsign.py` / `b3_ident.py` | 有限差分符号/恒等性检查（残差符号约定确认） |

### p1（方案① χ² 门控）
| 脚本 | 用途 |
|---|---|
| `p1_batch.sh` / `p1_chain2-4.sh` / `p1_contall.sh` | 安全化扫描：σ{1e-4,3e-4,1e-3} × thr{9,100,1000} |
| `p1_run.sh` / `p1_smoke.sh` / `p1_a.sh` / `p1_ntu.sh` | 单跑 / 冒烟 / A 组 / NTU 验证 |
| **`p1_cont.py`** | 逐帧连续性：单帧最大位移、>20 cm 帧数（发现 0.396 m 跳变） |
| **`p1_ndiff.py`** | 同二进制两跑的噪声带估计（±0.05 m 不显著判据） |
| `p1_safe.py` | 注入安全统计汇总（max dp / max Δyaw / 接受率） |
| `p1_tab.sh` / `p1_row.sh` / `p1_wait*.sh` | 结果表生成与串行等待 |

## 复现一次对照的推荐顺序

1. `scripts/common/run_room.sh` 跑 off 基线（记下输出目录）；
2. `scripts/b3/b3_mkcfg.sh` 生成目标配置（或直接改 `config/livox_360_visa_room_b2.yaml`）；
3. `scripts/common/run_room.sh` 跑新配置（同 bag、同参数，仅切换 yaml）；
4. `scripts/common/cmp_traj.py` 逐 stamp 对比；`scripts/p1/p1_cont.py` 看单帧跳变；
5. 改过代码则必须跑 `scripts/b3/b3_bitcmp2.py` 确认 off 路径逐字节不变。

> 纪律：只认同 bag 逐点差或多跑分布；±0.05 m 内差异不算改善；不接受用危险单帧跳变换来的终点收益。
