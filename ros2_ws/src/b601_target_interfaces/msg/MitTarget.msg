# JointState.effort is the independent task feedforward.
sensor_msgs/JointState target
# Optional per-joint overrides. Omitted joints use the internal model.
b601_target_interfaces/JointFeedforward[] gravity_ff
b601_target_interfaces/JointFeedforward[] friction_ff
