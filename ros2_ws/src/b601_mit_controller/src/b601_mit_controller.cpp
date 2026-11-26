#include "b601_mit_controller/b601_mit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <controller_interface/helpers.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace b601_mit_controller
{
namespace
{
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<double, 9>;   // 行主序

constexpr double kGravity = b601_config::mit::gravity_m_s2;

Vec3 mul(const Mat3 & R, const Vec3 & v)
{
  return {
    R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
    R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
    R[6] * v[0] + R[7] * v[1] + R[8] * v[2]};
}

Mat3 mul(const Mat3 & A, const Mat3 & B)
{
  Mat3 C{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      C[3 * i + j] = A[3 * i + 0] * B[0 + j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
    }
  }
  return C;
}

Vec3 add(const Vec3 & a, const Vec3 & b) {return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};}
Vec3 sub(const Vec3 & a, const Vec3 & b) {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}
Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dot(const Vec3 & a, const Vec3 & b) {return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];}

Mat3 from_array(const double * v)
{
  Mat3 R{};
  for (int i = 0; i < 9; ++i) {
    R[i] = v[i];
  }
  return R;
}

Vec3 from_array3(const double * v) {return {v[0], v[1], v[2]};}

/// Rodrigues：绕单位轴旋转 angle
Mat3 rot_axis(const Vec3 & axis, double angle)
{
  const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
  const double x = axis[0], y = axis[1], z = axis[2];
  return {
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c};
}

/// 把参数读成"每个关节一个值"：接受标量（全体同值）或 N 元数组。
bool param_to_doubles(const rclcpp::Parameter & p, std::vector<double> & out)
{
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    out.assign(1, p.as_double());
    return true;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    out = p.as_double_array();
    return true;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    out.assign(1, static_cast<double>(p.as_int()));
    return true;
  }
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
    out.clear();
    for (const auto v : p.as_integer_array()) {
      out.push_back(static_cast<double>(v));
    }
    return true;
  }
  return false;
}

/// 线性推进到目标（不越过）。刚度斜坡、目标斜坡都用它 —— 相比一阶滤波，
/// 斜率是常数、到位时间可算，验收时能直接对表。
double move_toward(double cur, double target, double max_step)
{
  if (max_step <= 0.0) {
    return target;
  }
  const double d = target - cur;
  if (std::abs(d) <= max_step) {
    return target;
  }
  return cur + std::copysign(max_step, d);
}

}  // namespace

// ------------------------------------------------------------------ 接口声明
controller_interface::InterfaceConfiguration
B601MitController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
    cfg.names.push_back(name + "/kp");
    cfg.names.push_back(name + "/kd");
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_EFFORT);
    if (name == "joint1") {
      cfg.names.push_back(name + "/low_stiffness");
    }
  }
  return cfg;
}

controller_interface::InterfaceConfiguration B601MitController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // 本实例的关节：position + velocity
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  // 整链位形：本实例没控的链关节也要读 position（重力前馈要用整条链）
  for (const auto & name : chain_names_) {
    if (std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end()) {
      continue;   // 已经声明过，别重复
    }
    cfg.names.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return cfg;
}

// ---------------------------------------------------------------------- 初始化
std::vector<double> B601MitController::to_per_joint(
  const std::string & name, const rclcpp::Parameter & p, std::size_t n) const
{
  std::vector<double> raw;
  if (!param_to_doubles(p, raw)) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "%s 的类型不支持（需要 double / double[]），当前 %s",
      name.c_str(), p.get_type_name().c_str());
    return {};
  }
  std::vector<double> out;
  if (raw.size() == n) {
    out = raw;
  } else if (raw.size() == 1) {
    out.assign(n, raw[0]);
  } else {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "%s 需要 1 个或 %zu 个值（= joints 的个数），实际 %zu", name.c_str(), n, raw.size());
    return {};
  }
  for (auto & v : out) {
    if (!std::isfinite(v)) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s 含非有限值", name.c_str());
      return {};
    }
  }
  return out;
}

bool B601MitController::validate_chain_names()
{
  if (chain_names_.size() != chain::kNumJoints) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "chain_joints 必须是 %zu 个（整条运动链），实际 %zu —— 重力前馈无法计算",
      chain::kNumJoints, chain_names_.size());
    return false;
  }
  for (std::size_t k = 0; k < chain::kNumJoints; ++k) {
    const std::string expect = chain::kJoints[k].name;
    if (chain_names_[k] != expect) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "chain_joints[%zu] 应为 %s（顺序必须与 b601_chain.hpp 一致），实际 %s",
        k, expect.c_str(), chain_names_[k].c_str());
      return false;
    }
  }
  return true;
}

bool B601MitController::read_gain_params()
{
  auto node = get_node();
  const std::size_t n = joint_names_.size();
  auto ensure = [&node](const std::string & name, auto default_value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter(name, default_value);
      }
    };

  ensure("kp_move", b601_config::mit::kp_move);
  ensure("kd_move", b601_config::mit::kd_move);
  ensure("friction", std::vector<double>(n, 0.0));
  kp_move_ = to_per_joint("kp_move", node->get_parameter("kp_move"), n);
  kd_move_ = to_per_joint("kd_move", node->get_parameter("kd_move"), n);

  // 旧自定义文件仍可通过 hold 参数提供等待增益。统一配置显式提供 free 增益。
  ensure("kp_hold", kp_move_);
  ensure("kd_hold", kd_move_);
  ensure("kp_free", to_per_joint("kp_hold", node->get_parameter("kp_hold"), n));
  ensure("kd_free", to_per_joint("kd_hold", node->get_parameter("kd_hold"), n));
  ensure("free_enabled", std::vector<bool>(n, true));
  ensure("settle_band_min", b601_config::mit::settle_band_min);
  ensure("settle_band_max", b601_config::mit::settle_band_max);

  kp_free_ = to_per_joint("kp_free", node->get_parameter("kp_free"), n);
  kd_free_ = to_per_joint("kd_free", node->get_parameter("kd_free"), n);
  settle_band_min_ = to_per_joint("settle_band_min", node->get_parameter("settle_band_min"), n);
  settle_band_max_ = to_per_joint("settle_band_max", node->get_parameter("settle_band_max"), n);
  {
    std::vector<bool> raw;
    const auto & p = node->get_parameter("free_enabled");
    if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL_ARRAY) {
      raw = p.as_bool_array();
    } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
      raw.assign(1, p.as_bool());
    } else {
      RCLCPP_ERROR(node->get_logger(), "free_enabled 类型不支持（需要 bool / bool[]）");
      return false;
    }
    free_enabled_.assign(n, true);
    if (raw.size() == n) {
      for (std::size_t i = 0; i < n; ++i) free_enabled_[i] = raw[i];
    } else if (raw.size() != 1) {
      RCLCPP_ERROR(node->get_logger(), "free_enabled 需要 1 或 %zu 个值，实际 %zu", n, raw.size());
      return false;
    } else {
      for (std::size_t i = 0; i < n; ++i) free_enabled_[i] = raw[0];
    }
  }
  friction_ = to_per_joint("friction", node->get_parameter("friction"), n);
  if (kp_move_.size() != n ||
    kd_move_.size() != n || friction_.size() != n ||
    kp_free_.size() != n || kd_free_.size() != n ||
    settle_band_min_.size() != n || settle_band_max_.size() != n)
  {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    kp_free_[i] = std::max(0.0, kp_free_[i]);
    kd_free_[i] = std::max(0.0, kd_free_[i]);
    settle_band_min_[i] = std::max(0.0, settle_band_min_[i]);
    settle_band_max_[i] = std::max(settle_band_min_[i], settle_band_max_[i]);
  }
  for (std::size_t i = 0; i < n; ++i) {
    kp_move_[i] = std::max(0.0, kp_move_[i]);
    kd_move_[i] = std::max(0.0, kd_move_[i]);
  }
  return true;
}

controller_interface::CallbackReturn B601MitController::on_init()
{
  // 注意：controller_manager 可能多次调用 on_init（例如先加载后重新配置），
  // 所以每个参数都要"没声明过才声明"，否则会抛 'parameter has already been declared'。
  try {
    auto node = get_node();
    auto ensure = [&node](const std::string & name, auto default_value) {
        if (!node->has_parameter(name)) {
          node->declare_parameter(name, default_value);
        }
      };

    // ---- 关节集合：本实例控哪些 + 整链有哪些（后者只读 position）----
    ensure("joints", std::vector<std::string>{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    joint_names_ = node->get_parameter("joints").as_string_array();
    ensure("chain_joints", std::vector<std::string>{
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    chain_names_ = node->get_parameter("chain_joints").as_string_array();
    if (joint_names_.empty()) {
      RCLCPP_ERROR(node->get_logger(), "joints 不能为空");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (!validate_chain_names()) {
      return controller_interface::CallbackReturn::ERROR;
    }

    ensure("gravity_params", std::vector<double>(4 * chain::kNumGravityLinks, 0.0));
    const auto gp = node->get_parameter("gravity_params").as_double_array();
    if (gp.size() != 4 * chain::kNumGravityLinks) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "gravity_params 长度应为 %zu，实际 %zu",
        4 * chain::kNumGravityLinks, gp.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    std::copy(gp.begin(), gp.end(), gravity_params_.begin());

    if (!read_gain_params()) {
      return controller_interface::CallbackReturn::ERROR;
    }

    ensure("gravity_scale", gravity_scale_.load());
    gravity_scale_ = std::min(b601_config::mit::gravity_scale_max, std::max(0.0, node->get_parameter("gravity_scale").as_double()));
    const auto n = joint_names_.size();
    ensure("gravity_joint_scale", b601_config::mit::gravity_joint_scale_default);
    ensure("gravity_bias", b601_config::mit::gravity_bias_default);
    gravity_joint_scale_ = to_per_joint("gravity_joint_scale", node->get_parameter("gravity_joint_scale"), n);
    ensure("gravity_joint_scale_free", gravity_joint_scale_);
    gravity_joint_scale_free_ = to_per_joint(
      "gravity_joint_scale_free", node->get_parameter("gravity_joint_scale_free"), n);
    gravity_bias_ = to_per_joint("gravity_bias", node->get_parameter("gravity_bias"), n);
    const auto valid_values = [n](const std::vector<double> & values, bool signed_values) {
        return values.size() == n &&
               std::all_of(values.begin(), values.end(), [signed_values](double value) {
                 return std::isfinite(value) && (signed_values || value >= 0.0);
               });
      };
    if (!valid_values(gravity_joint_scale_, false) ||
      !valid_values(gravity_joint_scale_free_, false) || !valid_values(gravity_bias_, true))
    {
      RCLCPP_ERROR(node->get_logger(), "逐关节重力补偿参数或变化率无效");
      return controller_interface::CallbackReturn::ERROR;
    }
    ensure("free_follow_feedback", free_follow_feedback_);
    free_follow_feedback_ = node->get_parameter("free_follow_feedback").as_bool();
    ensure("low_after_first_target", low_after_first_target_);
    low_after_first_target_ = node->get_parameter("low_after_first_target").as_bool();
    ensure("friction_scale", friction_scale_.load());
    friction_scale_ = std::min(b601_config::mit::friction_scale_max, std::max(0.0, node->get_parameter("friction_scale").as_double()));
    ensure("friction_vel_eps", friction_vel_eps_);
    friction_vel_eps_ = std::max(b601_config::mit::friction_velocity_epsilon_min, node->get_parameter("friction_vel_eps").as_double());
    ensure("motor_impedance", true);
    if (!node->get_parameter("motor_impedance").as_bool()) {
      RCLCPP_ERROR(node->get_logger(), "本控制器需要 dm_command_mode:=mit_impedance");
      return controller_interface::CallbackReturn::ERROR;
    }
    ensure("task_torque_timeout_sec", task_torque_timeout_sec_);
    task_torque_timeout_sec_ = node->get_parameter("task_torque_timeout_sec").as_double();
    ensure("target_velocity_timeout_sec", target_velocity_timeout_sec_);
    target_velocity_timeout_sec_ = node->get_parameter("target_velocity_timeout_sec").as_double();
    if (!std::isfinite(task_torque_timeout_sec_) || task_torque_timeout_sec_ < 0.0 ||
      !std::isfinite(target_velocity_timeout_sec_) || target_velocity_timeout_sec_ <= 0.0)
    {
      RCLCPP_ERROR(node->get_logger(), "任务力矩或速度目标超时参数无效");
      return controller_interface::CallbackReturn::ERROR;
    }
    ensure("use_gravity", use_gravity_.load());
    use_gravity_ = node->get_parameter("use_gravity").as_bool();
    // ---- 真机鲁棒性三件套（默认值即为保守值，可用 ros2 param set 现场调）----
    // ---- 指令层（Phase 1）----
    ensure("settle_band_pct", settle_band_pct_);
    settle_band_pct_ = std::clamp(node->get_parameter("settle_band_pct").as_double(), 0.0, 1.0);
    ensure("settle_sec", settle_sec_);
    settle_sec_ = std::max(b601_config::mit::settle_sec_min, node->get_parameter("settle_sec").as_double());
    ensure("target_slew", std::vector<double>{});
    target_slew_ = to_per_joint("target_slew", node->get_parameter("target_slew"), joint_names_.size());
    if (target_slew_.size() != joint_names_.size() ||
      !std::all_of(target_slew_.begin(), target_slew_.end(), [](double value) {
        return std::isfinite(value) && value > 0.0;
      }))
    {
      RCLCPP_ERROR(node->get_logger(), "target_slew 需要有效的正逐关节斜率");
      return controller_interface::CallbackReturn::ERROR;
    }
    ensure("kp_up_sec", kp_up_sec_);
    kp_up_sec_ = std::max(b601_config::mit::ramp_time_sec_min, node->get_parameter("kp_up_sec").as_double());
    ensure("kp_down_sec", kp_down_sec_);
    kp_down_sec_ = std::max(b601_config::mit::ramp_time_sec_min, node->get_parameter("kp_down_sec").as_double());
    ensure("q_ref_lpf_hz", q_ref_lpf_hz_);
    const auto q_ref_lpf_hz = node->get_parameter("q_ref_lpf_hz").as_double();
    q_ref_lpf_hz_ = q_ref_lpf_hz == 0.0 ? 0.0 :
      std::max(b601_config::mit::position_filter_hz_min, q_ref_lpf_hz);
    ensure("track_timeout_sec", track_timeout_sec_);
    track_timeout_sec_ = std::max(0.0, node->get_parameter("track_timeout_sec").as_double());
    ensure("velocity_lpf_hz", velocity_lpf_hz_.load());
    velocity_lpf_hz_ = std::max(b601_config::mit::velocity_filter_hz_min, node->get_parameter("velocity_lpf_hz").as_double());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "参数读取失败：%s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "MIT 控制器：%zu 个关节（%s），整链重力前馈用 %zu 个关节的位形",
    joint_names_.size(), joint_names_.front().c_str(), chain_names_.size());
  {
    std::string gains;
    char buf[64];
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      std::snprintf(
        buf, sizeof(buf), "%s%s 跟踪=%.3f/%.3f 保持=%.3f/%.3f", i ? " |" : "",
        joint_names_[i].c_str(), kp_move_[i], kd_move_[i], kp_free_[i], kd_free_[i]);
      gains += buf;
    }
    RCLCPP_INFO(get_node()->get_logger(), "逐关节增益：%s", gains.c_str());
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "指令层：slew=%.2f 误差带=%.0f%%×位移且夹在[%.4f,%.4f] 到位判定=%.2fs "
    "刚上升/下降=%.2f/%.2fs 等待档某关节禁用=%s track_timeout=%.1fs",
    target_slew_.front(), settle_band_pct_ * 100.0,
    settle_band_min_.front(), settle_band_max_.front(), settle_sec_,
    kp_up_sec_, kp_down_sec_,
    std::any_of(free_enabled_.begin(), free_enabled_.end(), [](bool b) {return !b;}) ? "是" : "否",
    track_timeout_sec_);
  RCLCPP_INFO(
    get_node()->get_logger(),
    "电机侧 MIT：重力 %.2f，摩擦 %.2f，任务力矩超时 %.2f s",
    gravity_scale_.load(), friction_scale_.load(), task_torque_timeout_sec_);
  return controller_interface::CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult B601MitController::parameter_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::optional<bool> use_gravity;
  const std::size_t n = joint_names_.size();
  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "use_gravity") {
      use_gravity = p.as_bool();
    } else if (name == "gravity_scale") {
      gravity_scale_ = std::clamp(p.as_double(), 0.0, b601_config::mit::gravity_scale_max);
    } else if (name == "friction_scale") {
      friction_scale_ = std::clamp(p.as_double(), 0.0, b601_config::mit::friction_scale_max);
    } else if (name == "kp_move" || name == "kd_move" || name == "friction")
    {
      auto v = to_per_joint(name, p, n);
      if (v.size() != n) {
        result.successful = false;
        result.reason = name + " 长度不对（需要 1 或 " + std::to_string(n) + " 个值）";
        return result;
      }
      for (auto & x : v) {
        x = std::max(0.0, x);
      }
      if (name == "kp_move") {
        kp_move_ = v;
      } else if (name == "kd_move") {
        kd_move_ = v;
      } else {
        friction_ = v;
      }
    } else if (name == "velocity_lpf_hz") {
      velocity_lpf_hz_ = std::max(b601_config::mit::velocity_filter_hz_min, p.as_double());
    } else {
      continue;
    }
    RCLCPP_INFO(get_node()->get_logger(), "运行时更新 %s", name.c_str());
  }
  if (use_gravity.has_value()) {
    use_gravity_ = *use_gravity;
  }
  return result;
}

bool B601MitController::bind_interfaces()
{
  const std::size_t n = joint_names_.size();
  auto joint_index = [this](const std::string & name) -> int {
      for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i] == name) {
          return static_cast<int>(i);
        }
      }
      return -1;
    };

  pos_idx_.assign(n, -1);
  vel_idx_.assign(n, -1);
  cmd_idx_.assign(n, -1);
  pos_cmd_idx_.assign(n, -1);
  vel_cmd_idx_.assign(n, -1);
  kp_idx_.assign(n, -1);
  kd_idx_.assign(n, -1);
  low_stiffness_idx_ = -1;
  chain_pos_idx_.fill(-1);

  for (std::size_t k = 0; k < state_interfaces_.size(); ++k) {
    const auto & si = state_interfaces_[k];
    // Humble 的 get_name() 返回 "<关节>/<接口>"，关节名要用 get_prefix_name()
    const std::string prefix = si.get_prefix_name();
    const std::string iface = si.get_interface_name();
    if (iface == hardware_interface::HW_IF_POSITION) {
      const int j = joint_index(prefix);
      if (j >= 0) {
        pos_idx_[j] = static_cast<int>(k);
      }
      // 整链位形：无论本实例是否控这个关节，都要能读到
      for (std::size_t c = 0; c < chain_names_.size(); ++c) {
        if (chain_names_[c] == prefix) {
          chain_pos_idx_[c] = static_cast<int>(k);
        }
      }
    } else if (iface == hardware_interface::HW_IF_VELOCITY) {
      const int j = joint_index(prefix);
      if (j >= 0) {
        vel_idx_[j] = static_cast<int>(k);
      }
    }
  }
  for (std::size_t k = 0; k < command_interfaces_.size(); ++k) {
    const int j = joint_index(command_interfaces_[k].get_prefix_name());
    if (j < 0) {
      continue;
    }
    const auto & iface = command_interfaces_[k].get_interface_name();
    if (iface == "low_stiffness" && command_interfaces_[k].get_prefix_name() == "joint1") {
      low_stiffness_idx_ = static_cast<int>(k);
    } else if (iface == hardware_interface::HW_IF_EFFORT && cmd_idx_[j] < 0) {
      cmd_idx_[j] = static_cast<int>(k);
    } else if (iface == hardware_interface::HW_IF_POSITION && pos_cmd_idx_[j] < 0) {
      pos_cmd_idx_[j] = static_cast<int>(k);
    } else if (iface == hardware_interface::HW_IF_VELOCITY && vel_cmd_idx_[j] < 0) {
      vel_cmd_idx_[j] = static_cast<int>(k);
    } else if (iface == "kp" && kp_idx_[j] < 0) {
      kp_idx_[j] = static_cast<int>(k);
    } else if (iface == "kd" && kd_idx_[j] < 0) {
      kd_idx_[j] = static_cast<int>(k);
    }
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (pos_cmd_idx_[i] < 0 || vel_cmd_idx_[i] < 0 || kp_idx_[i] < 0 || kd_idx_[i] < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "%s 缺少 MIT 命令接口，需要 dm_command_mode:=mit_impedance",
        joint_names_[i].c_str());
      return false;
    }
  }

  // 本实例关节 → 整链行号（夹爪等不在链里的关节记 -1：它的重力项按 0 处理）
  gravity_row_.assign(n, -1);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t c = 0; c < chain::kNumJoints; ++c) {
      if (joint_names_[i] == std::string(chain::kJoints[c].name)) {
        gravity_row_[i] = static_cast<int>(c);
        break;
      }
    }
  }

  for (std::size_t i = 0; i < n; ++i) {
    if (pos_idx_[i] < 0 || vel_idx_[i] < 0 || cmd_idx_[i] < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "接口绑定失败：%s 缺 position/velocity 状态接口或命令接口（pos=%d vel=%d cmd=%d）",
        joint_names_[i].c_str(), pos_idx_[i], vel_idx_[i], cmd_idx_[i]);
      return false;
    }
  }
  for (std::size_t c = 0; c < chain_names_.size(); ++c) {
    if (chain_pos_idx_[c] < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "整链位形读不到：chain_joints[%zu]=%s 没有 position 状态接口",
        c, chain_names_[c].c_str());
      return false;
    }
  }
  RCLCPP_INFO(
    get_node()->get_logger(),
    "接口已按名绑定：%zu 状态 / %zu 命令接口（本实例 %zu 关节，整链 %zu 关节）",
    state_interfaces_.size(), command_interfaces_.size(), n, chain_names_.size());
  return true;
}

controller_interface::CallbackReturn B601MitController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();
  target_sub_ = node->create_subscription<b601_target_interfaces::msg::MitTarget>(
    "~/joint_target", rclcpp::SystemDefaultsQoS(),
    std::bind(&B601MitController::target_callback, this, std::placeholders::_1));
  gravity_buffer_.writeFromNonRT(std::vector<GravitySample>(joint_names_.size()));
  gravity_sub_ = node->create_subscription<GravityCompensation>(
    "~/gravity_compensation", rclcpp::QoS(1),
    [this](const GravityCompensation::SharedPtr message) {
      EffortOverrides efforts;
      if (read_overrides(message->efforts, efforts)) {
        write_gravity(efforts, message->replace);
      }
    });
  ff_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/ff", b601_config::mit::topic_depth);
  compensation_sources_pub_ = node->create_publisher<b601_target_interfaces::msg::CompensationSources>(
    "~/compensation_sources", rclcpp::SensorDataQoS());
  debug_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/debug", b601_config::mit::topic_depth);

  param_cb_handle_ = node->add_on_set_parameters_callback(
    std::bind(&B601MitController::parameter_callback, this, std::placeholders::_1));

  state_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("~/state", b601_config::mit::topic_depth);
  soft_stiffness_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "~/soft_stiffness", b601_config::mit::topic_depth);

  const std::size_t n = joint_names_.size();
  TargetCommand init;
  init.q.assign(n, 0.0);
  init.seq = 0;
  target_buffer_.writeFromNonRT(init);
  have_target_ = false;
  have_state_ = false;
  done_ = true;                 // 激活后先按"已到位"：不许有动作，等人发目标
  t_in_band_ = 0.0;
  t_track_ = 0.0;
  seq_seen_ = 0;
  RCLCPP_INFO(node->get_logger(), "已配置：等待目标话题 %s/joint_target", node->get_name());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn B601MitController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bind_interfaces()) {
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = joint_names_.size();
  q_cmd_.assign(n, 0.0);
  q_up_.assign(n, 0.0);
  q_ref_.assign(n, 0.0);
  settle_band_.assign(n, 0.0);
  q_.assign(n, 0.0);
  qd_.assign(n, 0.0);
  qd_f_.assign(n, 0.0);
  q_des_.assign(n, 0.0);
  tau_cmd_.assign(n, 0.0);
  ff_components_.assign(n * 3, 0.0);
  compensation_sources_.joints = joint_names_;
  compensation_sources_.gravity_enabled = use_gravity_.load();
  compensation_sources_.gravity_external.assign(n, false);
  compensation_sources_.friction_external.assign(n, false);
  kp_cur_.assign(n, 0.0);
  kd_cur_.assign(n, 0.0);
  tau_g_.fill(0.0);

  // 锚定低刚度档先用跟踪增益稳定当前位形，再按正常到位判据降档。
  done_ = std::none_of(free_enabled_.begin(), free_enabled_.end(), [](bool enabled) {return enabled;}) ||
    (free_follow_feedback_ && !low_after_first_target_);
  for (std::size_t i = 0; i < n; ++i) {
    q_[i] = state_interfaces_[pos_idx_[i]].get_value();
    qd_[i] = state_interfaces_[vel_idx_[i]].get_value();
    qd_f_[i] = qd_[i];        // 低通状态从实测速度起步，避免首周期突跳
    q_des_[i] = q_[i];
    q_cmd_[i] = q_[i];
    q_up_[i] = q_[i];
    q_ref_[i] = q_[i];
    settle_band_[i] = settle_band_max_[i];
    tau_cmd_[i] = 0.0;
    // 激活时刚度必须是"当前状态该有的那个"：
    //   free_enabled=true（等待模式）→ kp_free（软，可手推，符合设计）
    //   free_enabled=false（纯 PD 对标 pos_vel）→ kp_move
    // ⚠ 以前无条件从 kp_free 起步，纯 PD 配置下会有 0.8 s 的"软手爬坡"，
    //   仿真里 joint2 在这 0.8 s 内掉到 6.26 rad/s 直接撞安全急停。
    kp_cur_[i] = done_ && free_enabled_[i] ? kp_free_[i] : kp_move_[i];
    kd_cur_[i] = done_ && free_enabled_[i] ? kd_free_[i] : kd_move_[i];
    command_interfaces_[cmd_idx_[i]].set_value(0.0);
    command_interfaces_[pos_cmd_idx_[i]].set_value(q_[i]);
    command_interfaces_[vel_cmd_idx_[i]].set_value(0.0);
    command_interfaces_[kp_idx_[i]].set_value(0.0);
    command_interfaces_[kd_idx_[i]].set_value(0.0);
  }
  if (low_stiffness_idx_ >= 0) {
    command_interfaces_[low_stiffness_idx_].set_value(0.0);
  }
  for (std::size_t c = 0; c < chain_names_.size(); ++c) {
    chain_q_[c] = state_interfaces_[chain_pos_idx_[c]].get_value();
  }
  have_state_ = true;
  have_target_ = true;
  target_received_ = false;          // 话题目标还没来，先以当前位形为 q_cmd
  t_in_band_ = 0.0;
  t_track_ = 0.0;
  {
    TargetCommand init;
    init.q = q_cmd_;
    init.seq = target_seq_.load();
    seq_seen_ = init.seq;
    target_buffer_.writeFromNonRT(init);
  }
  activation_time_ns_ = get_node()->now().nanoseconds();
  accepting_commands_ = true;
  RCLCPP_INFO(
    get_node()->get_logger(),
    "已激活：%zu 关节，初始目标为当前位形，软硬刚度切换 %s，重力补偿 %s，内部摩擦补偿 %s",
    n, std::any_of(free_enabled_.begin(), free_enabled_.end(), [](bool value) {return value;})
    ? "开" : "关", use_gravity_.load() ? "开" : "关", friction_scale_.load() > 1e-6 ? "开" : "关");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn B601MitController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  accepting_commands_ = false;
  for (auto & ci : command_interfaces_) {
    ci.set_value(0.0);      // 退出时把力矩命令（以及 kp/kd）清零
  }
  RCLCPP_INFO(get_node()->get_logger(), "已停用：力矩命令清零");
  return controller_interface::CallbackReturn::SUCCESS;
}

bool B601MitController::read_overrides(
  const GravityCompensation::_efforts_type & entries, EffortOverrides & values) const
{
  values.resize(joint_names_.size());
  for (const auto & entry : entries) {
    const auto joint = std::find(joint_names_.begin(), joint_names_.end(), entry.joint);
    if (joint == joint_names_.end() || !std::isfinite(entry.effort)) {
      return false;
    }
    auto & value = values[static_cast<std::size_t>(joint - joint_names_.begin())];
    if (value.has_value()) {
      return false;
    }
    value = entry.effort;
  }
  return true;
}

void B601MitController::write_gravity(const EffortOverrides & values, bool replace)
{
  auto samples = replace ? std::vector<GravitySample>(values.size()) : *gravity_buffer_.readFromNonRT();
  const auto expires_at = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
    std::chrono::duration<double>(b601_config::mit::gravity_feedback_timeout_sec));
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (values[i].has_value()) {
      samples[i] = GravitySample{*values[i], expires_at};
    }
  }
  gravity_buffer_.writeFromNonRT(samples);
}

void B601MitController::target_callback(
  const b601_target_interfaces::msg::MitTarget::SharedPtr command)
{
  const auto & msg = command->target;
  const auto stamp_ns = get_node()->now().nanoseconds();
  const auto n = joint_names_.size();
  if (!accepting_commands_.load() || msg.name.size() != msg.position.size() ||
    (!msg.velocity.empty() && msg.velocity.size() != msg.name.size()) ||
    (!msg.effort.empty() && msg.effort.size() != msg.name.size()))
  {
    return;
  }
  TargetCommand cmd;
  cmd.q.resize(n);
  cmd.velocity.assign(n, 0.0);
  cmd.tff.assign(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    const auto found = std::find(msg.name.begin(), msg.name.end(), joint_names_[i]);
    if (found == msg.name.end() || std::find(found + 1, msg.name.end(), *found) != msg.name.end()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), b601_config::mit::target_warning_throttle_ms,
        "目标缺少或重复关节 %s", joint_names_[i].c_str());
      return;
    }
    const auto k = static_cast<std::size_t>(found - msg.name.begin());
    cmd.q[i] = msg.position[k];
    if (!msg.velocity.empty()) {
      cmd.velocity[i] = msg.velocity[k];
    }
    if (!msg.effort.empty()) {
      cmd.tff[i] = msg.effort[k];
    }
    if (!std::isfinite(cmd.q[i]) || !std::isfinite(cmd.velocity[i]) || !std::isfinite(cmd.tff[i])) {
      return;
    }
  }
  EffortOverrides gravity;
  if (!read_overrides(command->gravity_ff, gravity) ||
    !read_overrides(command->friction_ff, cmd.friction_ff))
  {
    return;
  }
  if (!command->gravity_ff.empty()) {
    write_gravity(gravity, false);
  }
  cmd.stamp_ns = stamp_ns;
  cmd.seq = ++target_seq_;
  target_buffer_.writeFromNonRT(cmd);
  target_received_ = true;
}

// ---------------------------------------------------------------- 重力前馈计算
void B601MitController::compute_gravity(
  const std::array<double, chain::kNumJoints> & q, std::array<double, chain::kNumJoints> & tau) const
{
  tau.fill(0.0);
  // ① 正运动学：6 个可动关节的 world 位置/姿态，以及每根轴的世界方向
  Vec3 p_child[chain::kNumJoints];
  Mat3 R_child[chain::kNumJoints];
  Vec3 axis_world[chain::kNumJoints];
  Vec3 p_prev{0.0, 0.0, 0.0};
  Mat3 R_prev{1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (std::size_t i = 0; i < chain::kNumJoints; ++i) {
    const auto & j = chain::kJoints[i];
    const Vec3 xyz = from_array3(j.xyz);
    const Mat3 rot = from_array(j.rot);
    const Vec3 axis = from_array3(j.axis);
    const Vec3 p_joint = add(p_prev, mul(R_prev, xyz));
    const Mat3 R_joint = mul(R_prev, rot);
    axis_world[i] = mul(R_joint, axis);
    R_child[i] = mul(R_joint, rot_axis(axis, q[i]));
    p_child[i] = p_joint;
    p_prev = p_joint;
    R_prev = R_child[i];
  }

  // ② 逐 link 累加：τ_i += a_i · ( (m·p_com − m·p_joint_i) × g )
  const Vec3 g_vec{0.0, 0.0, -kGravity};
  for (std::size_t k = 0; k < chain::kNumGravityLinks; ++k) {
    const auto & link = chain::kGravityLinks[k];
    const double m = gravity_params_[4 * k];
    const Vec3 mu{
      gravity_params_[4 * k + 1], gravity_params_[4 * k + 2], gravity_params_[4 * k + 3]};
    if (std::abs(m) < 1e-9 && std::abs(mu[0]) + std::abs(mu[1]) + std::abs(mu[2]) < 1e-9) {
      continue;
    }
    const int d = link.joint_depth;
    const Vec3 p_link = add(p_child[d], mul(R_child[d], from_array3(link.xyz)));
    const Mat3 R_link = mul(R_child[d], from_array(link.rot));
    // m·p_com = m·p_link + R·(m·c)
    const Vec3 m_pcom = add(
      {m * p_link[0], m * p_link[1], m * p_link[2]}, mul(R_link, mu));
    for (int i = 0; i <= d; ++i) {
      const Vec3 m_pjoint{
        m * p_child[i][0], m * p_child[i][1], m * p_child[i][2]};
      tau[i] += dot(axis_world[i], cross(sub(m_pcom, m_pjoint), g_vec));
    }
  }
}

// ------------------------------------------------------------------ 主控制循环
controller_interface::return_type B601MitController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const std::size_t n = joint_names_.size();
  if (n == 0 || q_.size() != n) {
    return controller_interface::return_type::ERROR;
  }

  const double dt = std::clamp(period.seconds(), b601_config::mit::update_period_min_sec,
    b601_config::mit::update_period_max_sec);

  for (std::size_t i = 0; i < n; ++i) {
    q_[i] = state_interfaces_[pos_idx_[i]].get_value();
    qd_[i] = state_interfaces_[vel_idx_[i]].get_value();
  }
  // 整链位形（本实例没控的关节也要读：重力前馈算的是整条链）
  for (std::size_t c = 0; c < chain_names_.size(); ++c) {
    chain_q_[c] = state_interfaces_[chain_pos_idx_[c]].get_value();
  }

  // ================== ① 指令接受（整组） ==================
  // 判据不是"误差多大"，而是"上层目标有没有变"。人推只改 q、不改 q_cmd，
  // 所以永远不会被误判成"上层要大动作"（这是 Phase 1 的核心修正）。
  const auto activation_ns = activation_time_ns_.load();
  const auto now_ns = time.nanoseconds();
  const auto is_fresh = [activation_ns, now_ns](int64_t stamp_ns, double timeout_sec) {
      return stamp_ns >= activation_ns && stamp_ns <= now_ns &&
             (now_ns - stamp_ns) * 1e-9 < timeout_sec;
    };
  const auto * cmd = target_buffer_.readFromRT();
  const bool valid_target = cmd && have_target_ && target_received_.load() &&
    cmd->q.size() == n && cmd->stamp_ns >= activation_ns && cmd->stamp_ns <= now_ns;
  const bool velocity_fresh = valid_target && is_fresh(cmd->stamp_ns, target_velocity_timeout_sec_);
  const bool moving_reference = velocity_fresh &&
    std::any_of(cmd->velocity.begin(), cmd->velocity.end(), [](double v) {return v != 0.0;});
  const bool feedforward_fresh = valid_target && is_fresh(cmd->stamp_ns, task_torque_timeout_sec_);
  const auto * gravity_override = gravity_buffer_.readFromRT();
  const auto gravity_now = SteadyClock::now();
  if (valid_target && cmd->seq != seq_seen_)
  {
    bool changed = done_ && moving_reference;
    for (std::size_t i = 0; i < n; ++i) {
      if (cmd->q[i] != q_cmd_[i]) {
        changed = true;
        break;
      }
    }
    if (changed) {
      // 误差带按"本次指令的位移"算，并夹在 [min,max]：
      //   太小 → 永远到不了（低增益下垂）；太大 → 大幅动作时提前宣布到位。
      for (std::size_t i = 0; i < n; ++i) {
        if (settle_band_min_[i] > 0.0) {
          const double disp = std::abs(cmd->q[i] - q_cmd_[i]);
          settle_band_[i] = std::clamp(
            settle_band_pct_ * disp, settle_band_min_[i], settle_band_max_[i]);
        } else if (cmd->q[i] != q_cmd_[i]) {
          // 无最小误差带时，按新指令相对实测起点的位移计算。
          const double disp = std::abs(cmd->q[i] - q_[i]);
          settle_band_[i] = std::min(settle_band_pct_ * disp, settle_band_max_[i]);
        }
        if (done_ && free_enabled_[i] && free_follow_feedback_) {
          q_up_[i] = q_ref_lpf_hz_ == 0.0 ? q_[i] : q_ref_[i];
        }
        q_cmd_[i] = cmd->q[i];
      }
      done_ = false;          // 整组重新索权
      t_in_band_ = 0.0;
      t_track_ = 0.0;
      RCLCPP_INFO(
        get_node()->get_logger(), "新指令 → TRACK（带=%.4f，位移=%.4f）",
        settle_band_.front(), std::abs(q_cmd_.front() - q_ref_.front()));
    }
    seq_seen_ = cmd->seq;
  }

  // ================== ② 目标斜坡（只决定"怎么走"） ==================
  for (std::size_t i = 0; i < n; ++i) {
    q_up_[i] = move_toward(q_up_[i], q_cmd_[i], target_slew_[i] * dt);
  }

  // 速度低通仅用于主机侧摩擦补偿。
  const double a_v = -std::expm1(-dt * 2.0 * M_PI * velocity_lpf_hz_.load());
  for (std::size_t i = 0; i < n; ++i) {
    qd_f_[i] += a_v * (qd_[i] - qd_f_[i]);
  }

  // ---- 重力前馈（整链算，取本实例关节对应的行）----
  const bool gravity_enabled = use_gravity_.load();
  compensation_sources_.gravity_enabled = gravity_enabled;
  if (gravity_enabled) {
    compute_gravity(chain_q_, tau_g_);
  } else {
    tau_g_.fill(0.0);
  }

  // ================== ③ 整组到位判定（对着 q_cmd，不用斜坡数据） ==================
  if (!done_ && (!low_after_first_target_ || valid_target)) {
    bool in_band = !moving_reference;
    for (std::size_t i = 0; i < n; ++i) {
      // 允许带内小幅震荡：只看"在不在带里"，不判速度
      if (q_up_[i] != q_cmd_[i] || std::abs(q_cmd_[i] - q_[i]) > settle_band_[i]) {
        in_band = false;
        break;
      }
    }
    t_in_band_ = in_band ? t_in_band_ + dt : 0.0;
    t_track_ += dt;
    if (t_in_band_ >= settle_sec_) {
      done_ = true;          // 锁存：只有"新指令"能清
      RCLCPP_INFO(get_node()->get_logger(), "整组到位（%.1f s 在误差带内）",
        t_in_band_);
    } else if (track_timeout_sec_ > 0.0 && t_track_ > track_timeout_sec_) {
      done_ = true;
      RCLCPP_WARN(get_node()->get_logger(),
        "TRACK 超过 %.1f s 仍未到位 → 按「尽力完成」锁存（目标不可达？）", track_timeout_sec_);
    }
  }

  // ================== ④ 参考量 + 刚度目标 ==================
  const double a_ref = std::min(1.0, dt * 2.0 * M_PI * q_ref_lpf_hz_);
  for (std::size_t i = 0; i < n; ++i) {
    const bool authoritative = !done_ || !free_enabled_[i];
    if (authoritative || !free_follow_feedback_) {
      q_ref_[i] = q_up_[i];                       // 上层有权威：跟斜坡走
    } else if (q_ref_lpf_hz_ == 0.0) {
      q_ref_[i] = q_[i];                          // 保持档直接用本周期实测作为目标
    } else {
      q_ref_[i] += a_ref * (q_[i] - q_ref_[i]);   // 无权威：跟着实测（手推到哪停到哪）
    }
    q_des_[i] = q_ref_[i];    // 兼容旧字段（debug 里用过）
  }

  // ================== ⑤ 变刚度（线性速率斜坡） ==================
  const double ramp_sec = done_ ? kp_down_sec_ : kp_up_sec_;
  const double gravity_scale = gravity_scale_.load();
  const double friction_scale = friction_scale_.load();

  for (std::size_t i = 0; i < n; ++i) {
    const bool authoritative = !done_ || !free_enabled_[i];
    const double kp_tgt = authoritative ? kp_move_[i] : kp_free_[i];
    const double kd_tgt = authoritative ? kd_move_[i] : kd_free_[i];
    // 每关节各自用"该关节 free↔track 的跨度 / 斜坡时间"作为线性斜率
    const double kp_step = std::abs(kp_move_[i] - kp_free_[i]) / ramp_sec * dt;
    const double kd_step = std::abs(kd_move_[i] - kd_free_[i]) / ramp_sec * dt;
    kp_cur_[i] = move_toward(kp_cur_[i], kp_tgt, kp_step);
    kd_cur_[i] = move_toward(kd_cur_[i], kd_tgt, kd_step);

    const auto & gravity_joint_scale = authoritative ? gravity_joint_scale_ : gravity_joint_scale_free_;
    const double internal_gravity = gravity_enabled && gravity_row_[i] >= 0
      ? gravity_scale * (gravity_bias_[i] -
        gravity_joint_scale[i] * tau_g_[static_cast<std::size_t>(gravity_row_[i])])
      : 0.0;
    const double internal_friction =
      -friction_scale * friction_[i] * std::tanh(qd_f_[i] / friction_vel_eps_);
    const bool gravity_external =
      gravity_enabled && gravity_override && gravity_now < (*gravity_override)[i].expires_at;
    const bool friction_external = feedforward_fresh && cmd->friction_ff[i].has_value();
    const double gravity = gravity_external ? (*gravity_override)[i].effort : internal_gravity;
    const double friction = friction_external ? *cmd->friction_ff[i] : internal_friction;
    compensation_sources_.gravity_external[i] = gravity_external;
    compensation_sources_.friction_external[i] = friction_external;
    const double task_torque = feedforward_fresh ? cmd->tff[i] : 0.0;
    ff_components_[i] = friction;
    ff_components_[n + i] = gravity;
    ff_components_[2 * n + i] = task_torque;
    tau_cmd_[i] = gravity + friction + task_torque;

    command_interfaces_[pos_cmd_idx_[i]].set_value(q_ref_[i]);
    command_interfaces_[vel_cmd_idx_[i]].set_value(
      authoritative && velocity_fresh ? cmd->velocity[i] : 0.0);
    command_interfaces_[kp_idx_[i]].set_value(kp_cur_[i]);
    command_interfaces_[kd_idx_[i]].set_value(kd_cur_[i]);
    command_interfaces_[cmd_idx_[i]].set_value(tau_cmd_[i]);
  }
  const bool soft_stiffness = done_ && free_follow_feedback_ && joint_names_ == chain_names_ &&
    kp_cur_ == kp_free_ && kd_cur_ == kd_free_ &&
    std::all_of(free_enabled_.begin(), free_enabled_.end(), [](bool enabled) {return enabled;});
  if (low_stiffness_idx_ >= 0) {
    const bool low_stiffness = soft_stiffness && gravity_enabled && gravity_scale > 0.0;
    command_interfaces_[low_stiffness_idx_].set_value(low_stiffness ? 1.0 : 0.0);
  }

  // ---- 低频调试输出（约 10 Hz）----
  ++debug_counter_;
  if (debug_counter_ % b601_config::mit::diagnostic_divider == 0 && state_pub_) {
    std_msgs::msg::Float64MultiArray st;
    st.data.assign(n * 8 + 4, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      const double tau_grav = (gravity_row_[i] >= 0)
        ? tau_g_[static_cast<std::size_t>(gravity_row_[i])] : 0.0;
      st.data[i] = tau_cmd_[i];
      st.data[n + i] = tau_grav;
      st.data[2 * n + i] = kp_cur_[i];
      st.data[3 * n + i] = kd_cur_[i];
      st.data[4 * n + i] = q_cmd_[i] - q_[i];      // 判定用的误差
      st.data[5 * n + i] = q_ref_[i] - q_[i];      // 控制用的误差
      st.data[6 * n + i] = settle_band_[i];
      st.data[7 * n + i] = q_up_[i] - q_cmd_[i];   // 斜坡残差
    }
    st.data[8 * n + 0] = done_ ? 1.0 : 0.0;
    st.data[8 * n + 1] = static_cast<double>(seq_seen_);
    st.data[8 * n + 2] = t_in_band_;
    st.data[8 * n + 3] = t_track_;
    state_pub_->publish(st);
  }
  if (debug_counter_ % b601_config::mit::diagnostic_divider == 0 && ff_pub_) {
    std_msgs::msg::Float64MultiArray ff;
    ff.data = ff_components_;
    ff_pub_->publish(ff);
  }
  if (debug_counter_ % b601_config::mit::diagnostic_divider == 0 && compensation_sources_pub_) {
    compensation_sources_pub_->publish(compensation_sources_);
  }
  if (debug_counter_ % b601_config::mit::diagnostic_divider == 0 && soft_stiffness_pub_) {
    std_msgs::msg::Bool status;
    status.data = soft_stiffness;
    soft_stiffness_pub_->publish(status);
  }
  if (debug_counter_ % b601_config::mit::diagnostic_divider == 0 && debug_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(n * 4);
    for (std::size_t i = 0; i < n; ++i) {
      const double tau_grav = (gravity_row_[i] >= 0)
        ? tau_g_[static_cast<std::size_t>(gravity_row_[i])]
        : 0.0;
      msg.data[i] = tau_cmd_[i];
      // 这里输出**未缩放**的 τ_g：它是模型量（与 compare_taug.py 的 Python 模型逐位对拍用），
      // 缩放系数 gravity_scale 是调参旋钮，不该混进模型量里。
      msg.data[n + i] = tau_grav;
      msg.data[2 * n + i] = kp_cur_[i];
      msg.data[3 * n + i] = q_des_[i] - q_[i];
    }
    debug_pub_->publish(msg);
  }
  return controller_interface::return_type::OK;
}

}  // namespace b601_mit_controller

PLUGINLIB_EXPORT_CLASS(
  b601_mit_controller::B601MitController, controller_interface::ControllerInterface)
