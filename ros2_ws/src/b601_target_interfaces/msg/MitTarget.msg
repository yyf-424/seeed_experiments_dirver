# JointState.effort is the independent task feedforward.
sensor_msgs/JointState target
# Optional gravity samples for the listed joints, with independent feedback freshness.
b601_target_interfaces/JointFeedforward[] gravity_ff
# Optional friction overrides follow the target frame lifetime.
b601_target_interfaces/JointFeedforward[] friction_ff
