# Arduino system status report
string mode              # IDLE | RUN | ESTOP
float32 battery_voltage  # Volts (estimated from raw ADC)
bool motors_enabled
bool watchdog_ok         # False when watchdog has fired
uint32 uptime_sec        # Seconds since Arduino power-on
int32 left_speed         # Current left motor speed (-255 to 255)
int32 right_speed        # Current right motor speed (-255 to 255)
