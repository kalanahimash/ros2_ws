# Rover drive command
# linear: normalised forward/backward speed  (-1.0 to +1.0)
# angular: normalised turn rate              (-1.0 to +1.0, positive = left)
# emergency_stop: when true, override all motion and stop immediately

float32 linear
float32 angular
bool emergency_stop
