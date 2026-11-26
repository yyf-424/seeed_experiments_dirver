// 电机侧 MIT 控制器：主机生成位置、速度、增益和三段力矩前馈，驱动器闭合 PD。
#pragma once

#include "b601_description/project_parameters.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "b601_mit_controller/b601_chain.hpp"
#include "b601_target_interfaces/msg/compensation_sources.hpp"
#include "b601_target_interfaces/msg/gravity_compensation.hpp"
#include "b601_target_interfaces/msg/mit_target.hpp"

namespace b601_mit_controller
{

namespace rt = realtime_tools;

class B601MitController : public controller_interface::ControllerInterface
{
public:
  B601MitController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class Mode { kHold, kMove };

  /// 用链常量 + 辨识参数算重力力矩（Newton-Euler 的静态项，见 b601_chain.hpp 注释）。
  /// 输入是整链的关节角，输出重力施加的力矩。保持力矩取其反向。
  void compute_gravity(
    const std::array<double, chain::kNumJoints> & q,
    std::array<double, chain::kNumJoints> & tau) const;

  void target_callback(const b601_target_interfaces::msg::MitTarget::SharedPtr command);
  rcl_interfaces::msg::SetParametersResult parameter_callback(
    const std::vector<rclcpp::Parameter> & params);

  /// 逐关节参数：允许写一个标量（全体同值）或 N 元数组（逐关节）
  std::vector<double> to_per_joint(
    const std::string & name, const rclcpp::Parameter & p, std::size_t n) const;
  bool read_gain_params();
  /// 声明的 chain_joints 是否与 b601_chain.hpp 的整链一致（只允许 6 个、名字对得上）
  bool validate_chain_names();

  // ---- 配置 ----
  std::vector<std::string> joint_names_;     // 本实例控制的 N 个关节
  std::vector<std::string> chain_names_;     // 重力前馈所需整链（6 个，只读 position）
  std::array<double, 4 * chain::kNumGravityLinks> gravity_params_{};
  std::vector<double> friction_;             // N：库仑摩擦（关节单位力矩）
  // 运行时可调（ros2 param set 立即生效），因此用 atomic 避免与实时线程竞争。
  // 首调安全流程就是靠它：先把 gravity_scale 设 0，确认稳定再逐级升到 1。
  std::atomic<double> gravity_scale_{b601_config::mit::gravity_scale};
  std::vector<double> gravity_joint_scale_, gravity_joint_scale_free_, gravity_bias_;
  std::atomic<double> friction_scale_{b601_config::mit::friction_scale};
  double task_torque_timeout_sec_{b601_config::mit::task_torque_timeout_sec};
  double target_velocity_timeout_sec_{b601_config::mit::target_velocity_timeout_sec};
  // ---- 指令层（Phase 1）----
  std::vector<double> kp_free_;        // 等待（无权威）档刚度
  std::vector<double> kd_free_;
  std::vector<bool> free_enabled_;     // 该关节是否参与等待模式（夹爪也参与，决策⑭）
  bool free_follow_feedback_{b601_config::mit::free_follow_feedback_default};
  bool low_after_first_target_{false};
  std::vector<double> settle_band_min_;
  std::vector<double> settle_band_max_;
  std::vector<double> settle_band_;    // 每周期按本次指令位移算出来的实际误差带
  double settle_band_pct_{b601_config::mit::settle_band_pct};
  double settle_sec_{b601_config::mit::settle_sec};
  std::vector<double> target_slew_;  // 各关节位置目标斜率
  double kp_up_sec_{b601_config::mit::kp_up_sec};
  double kp_down_sec_{b601_config::mit::kp_down_sec};
  double q_ref_lpf_hz_{b601_config::mit::q_ref_lpf_hz};
  double track_timeout_sec_{b601_config::mit::track_timeout_sec};      // 0 = 关闭（被挡场景前期不做，决策⑩）
  std::vector<double> kp_move_;
  std::vector<double> kd_move_;
  double friction_vel_eps_{b601_config::mit::friction_vel_eps};             // 关节单位/s，tanh 平滑速度
  std::atomic<bool> use_gravity_{b601_config::mit::use_gravity};
  std::atomic<double> velocity_lpf_hz_{b601_config::mit::velocity_lpf_hz};  // 摩擦补偿使用的速度低通

  // ---- 运行时 ----
  // 状态/命令接口的**按名映射**索引（-1 = 未找到）。
  // 为什么不能再用 state_interfaces_[2*i]：非 mock 后端每关节导出 3 个状态接口
  // （position/velocity/effort），硬编码 2×i 会读到别的关节的值 ——
  // 2026-11-21 实测（MuJoCo 后端）：读到的 q 与真实位形完全错位，
  // PD 用错的值算出满力矩，170 ms 就把手臂推到限位。这也很可能是 2026-11-20
  // 真机"抽风"的成因之一。
  std::vector<int> pos_idx_;                 // N：本实例关节的 position
  std::vector<int> vel_idx_;                 // N：本实例关节的 velocity
  std::vector<int> cmd_idx_;                 // N：本实例关节的 effort 命令接口
  std::vector<int> vel_cmd_idx_;
  std::vector<int> pos_cmd_idx_;             // N：position 命令接口
  std::vector<int> kp_idx_;                  // N：kp 命令接口
  std::vector<int> kd_idx_;                  // N：kd 命令接口
  int low_stiffness_idx_{-1};
  std::array<int, chain::kNumJoints> chain_pos_idx_{-1, -1, -1, -1, -1, -1};
  std::vector<int> gravity_row_;             // N：本实例第 i 关节在整链里的行号（-1 = 不参与）
  bool bind_interfaces();

  std::array<double, chain::kNumJoints> chain_q_{};   // 整链位形（重力前馈用）
  // ---- 指令层运行时状态 ----
  std::vector<double> q_cmd_;      // 上层目标（判定基准）
  std::vector<double> q_up_;       // 限斜率后的参考（运动基准）
  std::vector<double> q_ref_;      // 本周期实际参考
  double t_in_band_{0.0};          // 整组连续在带内的累计时间
  bool done_{true};                // 整组执行标记（激活时先按"已到位"处理：不许有动作）
  uint64_t seq_seen_{0};           // 已处理到的指令序号
  double t_track_{0.0};            // 本段 TRACK 已持续的时间（给 track_timeout 用）
  std::vector<double> q_, qd_, qd_f_;
  std::vector<double> q_des_;
  std::vector<double> tau_cmd_;
  std::vector<double> ff_components_;  // 摩擦、重力和任务三段
  std::atomic<int64_t> activation_time_ns_{0};
  std::atomic<bool> accepting_commands_{false};
  std::array<double, chain::kNumJoints> tau_g_{};
  std::vector<double> kp_cur_, kd_cur_; // 实际生效值（含平滑过渡）
  Mode mode_{Mode::kHold};
  bool have_target_{false};
  std::atomic<bool> target_received_{false};   // 是否真的收到过话题目标
  std::atomic<uint64_t> target_seq_{0};        // 上层每来一帧有效目标就 +1
  bool have_state_{false};

  /// 指令：目标值 + 单调序号（必须同一个结构体一起交换，否则 RT 侧会读到"新目标+旧序号"）
  struct TargetCommand
  {
    std::vector<double> q;
    std::vector<double> velocity;
    std::vector<double> tff;
    std::vector<std::optional<double>> friction_ff;
    int64_t stamp_ns{0};
    uint64_t seq{0};
  };
  rt::RealtimeBuffer<TargetCommand> target_buffer_;
  using GravityCompensation = b601_target_interfaces::msg::GravityCompensation;
  using EffortOverrides = std::vector<std::optional<double>>;
  bool read_overrides(const GravityCompensation::_efforts_type & entries, EffortOverrides & values) const;
  using SteadyClock = std::chrono::steady_clock;
  struct GravitySample
  {
    double effort{0.0};
    SteadyClock::time_point expires_at;
  };
  void write_gravity(const EffortOverrides & values, bool replace);
  rt::RealtimeBuffer<std::vector<GravitySample>> gravity_buffer_;
  rclcpp::Subscription<GravityCompensation>::SharedPtr gravity_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ff_pub_;
  rclcpp::Publisher<b601_target_interfaces::msg::CompensationSources>::SharedPtr compensation_sources_pub_;
  b601_target_interfaces::msg::CompensationSources compensation_sources_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr soft_stiffness_pub_;
  rclcpp::Subscription<b601_target_interfaces::msg::MitTarget>::SharedPtr target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  std::size_t debug_counter_{0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace b601_mit_controller
