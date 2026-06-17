# Object tracking state published by rover_tracking
bool tracking            # True when a target is actively tracked
float32 target_x         # Target centroid X in image pixels
float32 target_y         # Target centroid Y in image pixels
float32 target_area      # Target contour area in pixels²
string state             # SEARCHING | TRACKING | LOST | DISABLED
float32 fps              # Tracking loop frames per second
