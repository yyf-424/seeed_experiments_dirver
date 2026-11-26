# The controller-wide gravity enable affects all listed joints.
bool gravity_enabled
# One flag per controlled joint. True means the controller selected an external value.
string[] joints
bool[] gravity_external
bool[] friction_external
