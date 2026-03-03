# ZoneStatusArray.msg
# Array of zone statuses for all configured safety zones

# Message header with timestamp
std_msgs/Header header

# Total number of configured zones
int32 num_zones

# Array of individual zone statuses
adi_3dtof_safety_bubble_detector/ZoneStatus[] zones
