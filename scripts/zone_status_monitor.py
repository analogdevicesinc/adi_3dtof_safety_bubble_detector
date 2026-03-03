#!/usr/bin/env python3
"""
Safety Bubble Zone Status Monitor
Subscribes to zone status JSON topic without needing custom message types
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class ZoneStatusMonitor(Node):
    """Monitor safety bubble zone status from JSON topic"""
    
    def __init__(self):
        super().__init__('zone_status_monitor')
        
        # Subscribe to JSON zone status topic (no custom message dependency!)
        self.subscription = self.create_subscription(
            String,
            '/combo_safety_bubble/zone_status_json',
            self.zone_status_callback,
            10
        )
        
        self.get_logger().info('Zone Status Monitor Started')
        self.get_logger().info('Listening to: /combo_safety_bubble/zone_status_json')
        self.get_logger().info('=' * 60)
        
    def zone_status_callback(self, msg):
        """Process incoming zone status JSON messages"""
        try:
            # Parse JSON string
            data = json.loads(msg.data)
            
            # Extract header information
            timestamp = data.get('timestamp', 'N/A')
            frame_id = data.get('frame_id', 'unknown')
            zones = data.get('zones', [])
            
            # Check if any zone has detection
            any_detected = any(zone.get('detected', False) for zone in zones)
            
            # Print status
            if any_detected:
                self.get_logger().warn(f'⚠️  OBJECT DETECTED! (frame: {frame_id})')
            else:
                self.get_logger().info(f'✓ All Clear (frame: {frame_id})')
            
            # Print detailed zone information
            for zone in zones:
                zone_id = zone.get('id', '?')
                enabled = zone.get('enabled', False)
                detected = zone.get('detected', False)
                shape = zone.get('shape', 'unknown')
                
                # Build status string
                status = '🔴 DETECTED' if detected else '🟢 Clear'
                enabled_str = 'ON' if enabled else 'OFF'
                
                # Get dimensions based on shape
                if shape == 'circle':
                    radius = zone.get('radius_mtr', 0.0)
                    dims = f'radius={radius:.2f}m'
                elif shape == 'rectangle':
                    x_dim = zone.get('x_dim_mtr', 0.0)
                    z_dim = zone.get('z_dim_mtr', 0.0)
                    dims = f'{x_dim:.2f}m x {z_dim:.2f}m'
                else:
                    dims = 'unknown'
                
                # Get color
                color = zone.get('color', [0, 0, 0, 0])
                color_str = f'RGBA({color[0]:.2f},{color[1]:.2f},{color[2]:.2f},{color[3]:.2f})'
                
                # Log zone info
                self.get_logger().info(
                    f'  Zone {zone_id} [{enabled_str}]: {status} | '
                    f'{shape} {dims} | {color_str}'
                )
            
            self.get_logger().info('-' * 60)
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor = ZoneStatusMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
