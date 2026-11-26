uint8 HEARTBEAT=0
uint8 TARGET=1

uint8 kind
# Identifies each running arbiter instance. Must be nonzero.
uint64 arbiter_id
# Increments on this controller group when a source expires.
uint64 hold_revision
# Identifier used to correlate dispatch with the controller's applied status.
uint64 dispatch_id
b601_target_interfaces/MitTarget command
