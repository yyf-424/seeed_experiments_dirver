uint8 HEARTBEAT=0
uint8 TARGET=1

uint8 kind
# Identifies each running arbiter instance. Must be nonzero.
uint64 arbiter_id
# Increments on this controller group when a source expires.
uint64 hold_revision
# TARGET: identifier for the controller's applied status.
# HEARTBEAT: most recent target still authorized for task integration, or zero.
uint64 dispatch_id
b601_target_interfaces/MitTarget command
