#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

namespace superlivo_common {

// ============================================================================
// Super-LIO `/lio/degeneracy` 签名与严重度映射
// ----------------------------------------------------------------------------
// 方案 A/B 都必须消费同一套 Super-LIO 退化检测，且严重度口径必须一致；此前两份
// 节点各自复制了同样的解析 + 对数映射代码，改一处很容易漏另一处，故抽到此处。
// ============================================================================

/// 严重度映射标尺（两套方案共用默认值）。
struct DegeneracyScale {
  double cond_lo{35.0};     ///< cond_t 低于它视为健康（severity = 0）
  double cond_hi{10000.0};  ///< cond_t 高于它视为完全退化（severity = 1）
  double timeout_sec{1.0};  ///< 超过这么久没收到新签名则视为不可用
};

/// 退化签名载荷：[cond6, axes, active, cond_t, evt_min, ux, uy, uz]。
struct DegeneracySignature {
  double cond6{0.0};
  int axes{0};
  bool active{false};
  double cond_t{0.0};
  double evt_min{0.0};
  Eigen::Vector3d dir{Eigen::Vector3d::Zero()};

  /// 检测器确实产出了签名。cond=0 说明 Super-LIO 侧没跑检测（detect_en 关闭）
  /// 或非 trans_only 模式；不能当"健康"用，否则退化门控会静默失效。
  bool valid() const { return cond6 > 0.0 && cond_t > 0.0; }

  /// 解析消息载荷；长度不足 8 时返回 false，且不修改现有值。
  bool fromArray(const std::vector<double>& data) {
    if (data.size() < 8) {
      return false;
    }
    cond6 = data[0];
    axes = static_cast<int>(data[1]);
    active = data[2] > 0.5;
    cond_t = data[3];
    evt_min = data[4];
    const Eigen::Vector3d u(data[5], data[6], data[7]);
    // 无方向（非 trans_only 模式）时必须置零，不能沿用上一帧的方向。
    dir = (u.norm() > 1e-6) ? u.normalized() : Eigen::Vector3d::Zero();
    return true;
  }

  /// 严重度：axes/active 是 Super-LIO 自己的硬判据，触发即 1；
  /// 未触发时按 cond_t 在对数刻度上连续映射（lo → 0，hi → 1）。
  /// 不能把"未触发"当作健康短路，否则严重度只会在硬阈值处 0 → 1 跳变，
  /// 阈值敏感性实验与渐进接管都做不了。
  double severity(double cond_lo, double cond_hi) const {
    if (axes > 0 || active) {
      return 1.0;
    }
    const double lo = std::max(1e-6, cond_lo);
    const double hi = std::max(lo * 1.001, cond_hi);
    const double c = std::max(lo, cond_t);
    return std::min(1.0, std::max(0.0, std::log(c / lo) / std::log(hi / lo)));
  }
};

}  // namespace superlivo_common
