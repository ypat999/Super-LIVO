#!/bin/bash
set -e
CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config
cp $CFG/livox_360_visa_room_b2.yaml $CFG/livox_360_visa_room_b3.yaml
cat >> $CFG/livox_360_visa_room_b3.yaml <<'EOF'

# =============================================================================
# B3：视觉长基线 -> 陀螺零偏 bg 的直接观测（在 B2 之上叠加）。
# 残差 e = Log(R_meas_rel^T * Exp(bg*dt - Phi_base))：姿态完全消掉，
# 雅可比只落在 bg 分量（ESKF 状态索引 9..11），需 18 维观测通道 UpdateObserveFull。
# 只在 lidar UpdateObserve 之后做第二次独立更新；b3.enable=false 时前端与注入都不动，
# 与 B2 行为一致；lio.visual.enable=false 时 B2/B3 全部不存在。
# =============================================================================
    lio.visual.b3.enable: true
    lio.visual.b3.baseline_dt: 1.0
    lio.visual.b3.sigma_rot: 0.05
    lio.visual.b3.chi2_thresh: 9.0
    lio.visual.b3.pyramid_levels: 4
    lio.visual.b3.fd_check: true
EOF
echo "room_b3.yaml written"
tail -12 $CFG/livox_360_visa_room_b3.yaml
