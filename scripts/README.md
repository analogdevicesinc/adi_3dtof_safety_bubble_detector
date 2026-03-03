# Safety Bubble Detector Scripts

This directory contains utility scripts for monitoring and working with the Safety Bubble Detector (SBD) package.

## Scripts

### zone_status_monitor.py

A Python script that subscribes to the zone status JSON topic and displays real-time detection information.

**Key Features:**
- ✅ No custom message dependencies required
- ✅ Subscribes to standard `std_msgs/msg/String` topic
- ✅ Displays zone detection status with clear visual indicators
- ✅ Shows zone dimensions and colors
- ✅ Easy to integrate into your own applications

**Usage:**

```bash
# Run directly
python3 zone_status_monitor.py

# Or as executable
./zone_status_monitor.py
```

**Requirements:**
- ROS 2 (Humble or later)
- Python 3
- rclpy (ROS 2 Python client library)

**Topic Subscribed:**
- `/combo_safety_bubble/zone_status_json` (std_msgs/msg/String)

**Example Output:**

```
[INFO] [zone_status_monitor]: Zone Status Monitor Started
[INFO] [zone_status_monitor]: Listening to: /combo_safety_bubble/zone_status_json
============================================================
[WARN] [zone_status_monitor]: ⚠️  OBJECT DETECTED! (frame: combo_safety_bubble)
[INFO] [zone_status_monitor]:   Zone 0 [ON]: 🔴 DETECTED | circle radius=1.50m | RGBA(1.00,0.00,0.00,0.50)
[INFO] [zone_status_monitor]:   Zone 1 [ON]: 🟢 Clear | circle radius=2.50m | RGBA(0.00,1.00,0.00,0.50)
[INFO] [zone_status_monitor]:   Zone 2 [ON]: 🟢 Clear | rectangle 3.00m x 2.00m | RGBA(0.00,0.00,1.00,0.50)
------------------------------------------------------------
```

## Integration Example

You can easily integrate the JSON parsing into your own ROS 2 nodes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.subscription = self.create_subscription(
            String,
            '/combo_safety_bubble/zone_status_json',
            self.callback,
            10
        )
    
    def callback(self, msg):
        data = json.loads(msg.data)
        zones = data.get('zones', [])
        
        # Check if any zone detected
        if any(zone.get('detected', False) for zone in zones):
            # Take action when object detected
            self.get_logger().warn('Object in safety zone!')
```

## Support

For questions or issues, please refer to the main SBD package documentation.
