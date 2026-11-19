// ============================================================================
// 共用标定数据加载器（header-only，A/B 两个方案都接入）。
// ----------------------------------------------------------------------------
// 数据来源与优先级（"各取所长"约定）：
//   1. camera_intrinsic.yaml（OpenCV FileStorage 格式）：提供内参 + 畸变；
//   2. FAST-Calib-ROS2 output/calib_result.txt（FAST-LIVO2 文本格式）：
//      提供外参 Rcl/Pcl；其 cam_* 内参仅作为 1 缺失时的后备；
//   3. 两者都读不到（路径为空 / 文件不存在）→ 沿用数据集 yaml 的参数
//      （非标数据集兜底）。
// 读取失败不报错中断，只通过 err 输出原因并返回 false，由调用方打印日志。
// ============================================================================
#ifndef SUPER_LIVO_COMMON_CALIBRATION_SOURCE_HPP
#define SUPER_LIVO_COMMON_CALIBRATION_SOURCE_HPP

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

namespace superlivo_common {

/// 一份标定文件能提供的全部信息（缺哪部分对应 has_* 为 false）。
struct CalibrationData {
  bool has_intrinsics = false;
  int width = 0;
  int height = 0;
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  std::vector<double> distortion;  // [k1, k2, p1, p2]

  bool has_extrinsics = false;
  std::vector<double> Rcl;  // 9，行优先
  std::vector<double> Pcl;  // 3
};

namespace detail {

/// 从任意文本片段中按顺序提取浮点数（容忍逗号 / 方括号 / 换行）。
inline std::vector<double> grabNumbers(const std::string& text) {
  std::vector<double> nums;
  const char* p = text.c_str();
  while (*p != '\0') {
    if (*p == '-' || *p == '+' || *p == '.' || std::isdigit(static_cast<unsigned char>(*p))) {
      char* end = nullptr;
      const double v = std::strtod(p, &end);
      if (end != p) {
        nums.push_back(v);
        p = end;
        continue;
      }
    }
    ++p;
  }
  return nums;
}

/// 在整份文本里找 "key:"，返回冒号之后到行尾（标量）或到配对 ']'（列表）的内容。
inline bool extractAfterKey(const std::string& content, const std::string& key,
                            std::string& value_out) {
  const std::string needle = key + ":";
  const std::size_t pos = content.find(needle);
  if (pos == std::string::npos) {
    return false;
  }
  std::size_t start = pos + needle.size();
  const std::size_t lb = content.find('[', start);
  const std::size_t eol = content.find('\n', start);
  if (lb != std::string::npos && (eol == std::string::npos || lb < eol)) {
    const std::size_t rb = content.find(']', lb);
    if (rb == std::string::npos) {
      return false;
    }
    value_out = content.substr(lb, rb - lb + 1);
    return true;
  }
  value_out = content.substr(start, eol == std::string::npos ? std::string::npos
                                                             : eol - start);
  return true;
}

}  // namespace detail

/// 读取 camera_intrinsic.yaml（OpenCV FileStorage 格式）。
/// 优先平铺键 fx/fy/cx/cy/k1/k2/p1/p2；缺失时回退 camera_matrix/dist_coeffs 矩阵。
inline bool loadCameraIntrinsic(const std::string& path, CalibrationData& out,
                                std::string& err) {
  if (path.empty()) {
    err = "路径为空";
    return false;
  }
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    err = "无法打开文件 " + path;
    return false;
  }

  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  std::vector<double> dist;

  const bool flat_ok = !fs["fx"].empty() && !fs["fy"].empty() && !fs["cx"].empty() &&
                       !fs["cy"].empty() && !fs["k1"].empty() && !fs["k2"].empty() &&
                       !fs["p1"].empty() && !fs["p2"].empty();
  if (flat_ok) {
    fs["fx"] >> fx;
    fs["fy"] >> fy;
    fs["cx"] >> cx;
    fs["cy"] >> cy;
    double k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0;
    fs["k1"] >> k1;
    fs["k2"] >> k2;
    fs["p1"] >> p1;
    fs["p2"] >> p2;
    dist = {k1, k2, p1, p2};
  } else if (!fs["camera_matrix"].empty() && !fs["dist_coeffs"].empty()) {
    cv::Mat K, D;
    fs["camera_matrix"] >> K;
    fs["dist_coeffs"] >> D;
    if (K.rows < 3 || K.cols < 3 || K.type() != CV_64F) {
      err = "camera_matrix 格式不正确（需 3x3 CV_64F）: " + path;
      return false;
    }
    fx = K.at<double>(0, 0);
    fy = K.at<double>(1, 1);
    cx = K.at<double>(0, 2);
    cy = K.at<double>(1, 2);
    if (D.total() < 4) {
      err = "dist_coeffs 少于 4 项: " + path;
      return false;
    }
    const double* d = reinterpret_cast<const double*>(D.data);
    dist = {d[0], d[1], d[2], d[3]};
  } else {
    err = "既无平铺 fx/fy/cx/cy/k1/k2/p1/p2，也无 camera_matrix/dist_coeffs: " + path;
    return false;
  }

  out.has_intrinsics = true;
  out.fx = fx;
  out.fy = fy;
  out.cx = cx;
  out.cy = cy;
  out.distortion = dist;
  if (!fs["image_width"].empty() && !fs["image_height"].empty()) {
    fs["image_width"] >> out.width;
    fs["image_height"] >> out.height;
  }
  return true;
}

/// 读取 FAST-Calib-ROS2 的 output/calib_result.txt（FAST-LIVO2 文本格式）。
/// Rcl/Pcl 为外参；cam_width/cam_fx/cam_d* 等为内参后备；scale 忽略。
inline bool loadFastCalib(const std::string& path, CalibrationData& out, std::string& err) {
  if (path.empty()) {
    err = "路径为空";
    return false;
  }
  std::ifstream in(path);
  if (!in.is_open()) {
    err = "无法打开文件 " + path;
    return false;
  }
  std::stringstream ss;
  ss << in.rdbuf();
  const std::string content = ss.str();

  std::string raw;
  if (detail::extractAfterKey(content, "Rcl", raw)) {
    auto nums = detail::grabNumbers(raw);
    if (nums.size() == 9) {
      out.has_extrinsics = true;
      out.Rcl = nums;
    }
  }
  if (detail::extractAfterKey(content, "Pcl", raw)) {
    auto nums = detail::grabNumbers(raw);
    if (nums.size() == 3) {
      out.has_extrinsics = true;
      out.Pcl = nums;
    }
  }
  if (!out.has_extrinsics) {
    err = "未解析到完整的 Rcl(9)/Pcl(3): " + path;
    return false;
  }

  // 内参后备（camera_intrinsic.yaml 缺失时使用）。
  double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
  bool intr_ok = true;
  for (const auto& kv : std::vector<std::pair<std::string, double*>>{
           {"cam_fx", &fx}, {"cam_fy", &fy}, {"cam_cx", &cx}, {"cam_cy", &cy}}) {
    if (!detail::extractAfterKey(content, kv.first, raw)) {
      intr_ok = false;
      break;
    }
    const auto nums = detail::grabNumbers(raw);
    if (nums.size() != 1) {
      intr_ok = false;
      break;
    }
    *kv.second = nums[0];
  }
  std::vector<double> dist;
  if (intr_ok) {
    for (int i = 0; i < 4 && intr_ok; ++i) {
      if (!detail::extractAfterKey(content, "cam_d" + std::to_string(i), raw)) {
        intr_ok = false;
        break;
      }
      const auto nums = detail::grabNumbers(raw);
      if (nums.size() != 1) {
        intr_ok = false;
        break;
      }
      dist.push_back(nums[0]);
    }
  }
  if (intr_ok) {
    out.has_intrinsics = true;
    out.fx = fx;
    out.fy = fy;
    out.cx = cx;
    out.cy = cy;
    out.distortion = dist;
    if (detail::extractAfterKey(content, "cam_width", raw)) {
      const auto nums = detail::grabNumbers(raw);
      if (nums.size() == 1) {
        out.width = static_cast<int>(nums[0]);
      }
    }
    if (detail::extractAfterKey(content, "cam_height", raw)) {
      const auto nums = detail::grabNumbers(raw);
      if (nums.size() == 1) {
        out.height = static_cast<int>(nums[0]);
      }
    }
  }
  return true;
}

/// 合并两份标定：preferred 有的字段覆盖 fallback 的，缺失字段回退 fallback。
/// 用法：mergeCalibration(intrinsicYaml, fastCalib) —— 内参以 intrinsic.yaml 优先、
/// FAST-Calib 后备；外参只有 FAST-Calib 提供，自然落到 fallback 分支。
inline CalibrationData mergeCalibration(const CalibrationData& preferred,
                                        const CalibrationData& fallback) {
  CalibrationData merged;
  if (preferred.has_intrinsics) {
    merged.has_intrinsics = true;
    merged.width = preferred.width != 0 ? preferred.width : fallback.width;
    merged.height = preferred.height != 0 ? preferred.height : fallback.height;
    merged.fx = preferred.fx;
    merged.fy = preferred.fy;
    merged.cx = preferred.cx;
    merged.cy = preferred.cy;
    merged.distortion = preferred.distortion;
  } else {
    merged.has_intrinsics = fallback.has_intrinsics;
    merged.width = fallback.width;
    merged.height = fallback.height;
    merged.fx = fallback.fx;
    merged.fy = fallback.fy;
    merged.cx = fallback.cx;
    merged.cy = fallback.cy;
    merged.distortion = fallback.distortion;
  }
  if (preferred.has_extrinsics) {
    merged.has_extrinsics = true;
    merged.Rcl = preferred.Rcl;
    merged.Pcl = preferred.Pcl;
  } else {
    merged.has_extrinsics = fallback.has_extrinsics;
    merged.Rcl = fallback.Rcl;
    merged.Pcl = fallback.Pcl;
  }
  return merged;
}

}  // namespace superlivo_common

#endif  // SUPER_LIVO_COMMON_CALIBRATION_SOURCE_HPP
