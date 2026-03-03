# ZoneStatus.msg
# Represents the status of a single safety zone

# Zone identifier (1 = innermost, N = outermost)
int32 id

# Whether this zone is enabled for detection
bool enabled

# Whether an object was detected in this zone
bool detected

# Shape of the zone: "circle" or "rectangle"
string shape

# Circle parameters (used if shape is "circle")
float32 radius_mtr

# Rectangle parameters (used if shape is "rectangle")
float32 x_dim_mtr
float32 z_dim_mtr

# Visualization color (RGBA, 0.0-1.0)
float32[4] color
