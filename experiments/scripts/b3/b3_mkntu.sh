#!/bin/bash
set -e
CFG=/home/ywj/git/Super-LIVO/third_party/Super-LIO/src/super_lio/config
cp $CFG/NTU.yaml $CFG/NTU_visa_b3.yaml
cat >> $CFG/NTU_visa_b3.yaml <<'EOF'

# =============================================================================
# 视觉旋转测量（B2）+ 长基线陀螺零偏观测（B3）—— NTU eee_03
# 内参/畸变/Rcl/Pcl 逐字取自
# solution_b_tight_coupled/super_livo_visual_b/config/visual_b_ntu.yaml。
# 本文件为 NTU.yaml 的派生副本（NTU.yaml 本体未改动）：LiDAR/IMU 段与 NTU.yaml 一致。
# B2 对照跑：-p lio.visual.b3.enable:=false；全关：-p lio.visual.enable:=false
# =============================================================================
    lio.visual.enable: true
    lio.visual.image_topic: "/left/image_raw"
    lio.visual.img_time_offset: 0.1
    lio.visual.frame_slack: 0.0875
    lio.visual.cam_fx: 425.0258563372763
    lio.visual.cam_fy: 426.7976260903337
    lio.visual.cam_cx: 386.0151866550880
    lio.visual.cam_cy: 241.9130336743440
    lio.visual.cam_width: 752
    lio.visual.cam_height: 480
    lio.visual.cam_dist: [-0.288105327549552, 0.074578284234601, 0.0007784489598138802, -0.0002277853975035461]
    lio.visual.Rcl: [0.000000, -1.000000, 0.000000,
                     0.010783, 0.000000, 0.999942,
                     0.999942, 0.000000, -0.010783]
    lio.visual.Pcl: [-0.109094, -0.035730, -0.066500]
    lio.visual.klt.max_features: 300
    lio.visual.klt.min_distance: 7.0
    lio.visual.klt.fb_error: 1.0
    lio.visual.klt.pyramid_levels: 3
    lio.visual.klt.use_fast: true
    lio.visual.klt.fast_threshold: 10
    lio.visual.klt.lk_window: 21
    lio.visual.essential.threshold: 1.5
    lio.visual.essential.prob: 0.999
    lio.visual.gate.min_inliers: 20
    lio.visual.sigma_rot: 0.02
    lio.visual.b3.enable: true
    lio.visual.b3.baseline_dt: 1.0
    lio.visual.b3.sigma_rot: 0.05
    lio.visual.b3.chi2_thresh: 9.0
    lio.visual.b3.pyramid_levels: 4
    lio.visual.b3.fd_check: true
EOF
echo "NTU_visa_b3.yaml written"; tail -14 $CFG/NTU_visa_b3.yaml
