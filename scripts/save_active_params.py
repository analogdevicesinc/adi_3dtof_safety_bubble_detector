#!/usr/bin/env python3
"""
Save Active Parameters - Uses ros2 param dump

Simple script to save current active parameters from a running ROS 2 node.
Uses the built-in ros2 param dump command with user-friendly wrapper.

Usage:
    # Save all parameters from default node
    python3 scripts/save_active_params.py

    # Save from specific node
    python3 scripts/save_active_params.py --node /cam1/adi_3dtof_safety_bubble_detector_node
    
    # Custom output file
    python3 scripts/save_active_params.py --output my_config.yaml
"""

import subprocess
import sys
import argparse
from datetime import datetime
import os


def main():
    parser = argparse.ArgumentParser(
        description='Save ROS 2 parameters using ros2 param dump',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--node', '-n',
        type=str,
        default='/adi_3dtof_safety_bubble_detector_node',
        help='Node name (default: /adi_3dtof_safety_bubble_detector_node)'
    )
    
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Output file (default: config/saved_params_<timestamp>.yaml)'
    )
    
    args = parser.parse_args()
    
    # Determine output file
    if args.output is None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_file = f'config/saved_params_{timestamp}.yaml'
    else:
        output_file = args.output
    
    # Create directory if needed
    os.makedirs(os.path.dirname(output_file) if os.path.dirname(output_file) else '.', exist_ok=True)
    
    print(f"📝 Saving parameters from: {args.node}")
    print(f"💾 Output file: {output_file}")
    
    # Use ros2 param dump
    try:
        cmd = ['ros2', 'param', 'dump', args.node, '--output-dir', os.path.dirname(output_file) or '.']
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        
        # ros2 param dump creates a file named after the node, so we need to rename it
        node_basename = args.node.replace('/', '_').strip('_')
        dump_file = os.path.join(os.path.dirname(output_file) or '.', f'{node_basename}.yaml')
        
        if os.path.exists(dump_file):
            # Rename to our desired filename
            os.rename(dump_file, output_file)
            print(f"\n✅ Parameters saved successfully!")
            print(f"\n📖 To use this configuration:")
            print(f"   ros2 launch adi_3dtof_safety_bubble_detector \\")
            print(f"       adi_3dtof_safety_bubble_detector_params_launch.py \\")
            print(f"       params_file:={output_file}")
        else:
            print(f"❌ Expected dump file not found: {dump_file}")
            return 1
            
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Error: {e.stderr}")
        print(f"\n💡 Make sure the node is running:")
        print(f"   ros2 node list | grep {args.node}")
        return 1
    except Exception as e:
        print(f"\n❌ Error: {str(e)}")
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
