# Super-LIVO
Integrate Super-LIO and SVO under FAST-LIVO2 framework

## 子模块（third_party）

依赖代码以 git submodule 方式固定在 `third_party/` 下，克隆时需递归拉取：

```bash
git clone --recursive https://github.com/ypat999/Super-LIVO.git
# 已克隆过的主仓库：
git submodule update --init --recursive
```

| 路径 | 上游仓库 | 分支 | 固定提交 |
| --- | --- | --- | --- |
| `third_party/FAST-LIVO2-ROS2` | https://github.com/U-AMC/FAST-LIVO2-ROS2.git | main | `a09599a` |
| `third_party/Super-LIO` | https://github.com/ypat999/Super-LIO.git | ros2 | `3d2d29c` |
| `third_party/basalt` | https://github.com/VladyslavUsenko/basalt.git | master | `0f3b2b5` |
| `third_party/rpg_svo_pro_open` | https://github.com/uzh-rpg/rpg_svo_pro_open.git | master | `ca371f3` |

说明：子模块当前均为上游原始代码，未做本地修改；若需在其上开发，请在对应子模块内新建分支，避免与上游分支冲突。
