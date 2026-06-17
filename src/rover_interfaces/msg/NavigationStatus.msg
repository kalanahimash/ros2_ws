# Autonomous navigation state published by rover_navigation
string state              # IDLE | FORWARD | STOP | REVERSE | TURN_LEFT | TURN_RIGHT | EMERGENCY_STOP
float32 distance_ahead    # Filtered distance reading (cm)
bool obstacle_detected    # True when distance < stop_threshold
uint32 state_age_ms       # Milliseconds spent in current state
bool autonomous_enabled   # True when autonomous mode is active
