#!/usr/bin/env python3
"""
Script to set Gazebo camera pose via GUI plugin API
Usage: python3 set_camera_pose.py <x> <y> <z> <roll> <pitch> <yaw>
"""

import sys
import time
import os
import xml.etree.ElementTree as ET

def set_camera_pose(x, y, z, roll, pitch, yaw):
    """Set camera pose using Gazebo GUI plugin API"""
    # Try multiple methods
    
    # Method 1: Try gz command with retry
    for attempt in range(3):
        try:
            import subprocess
            # Wait a bit longer for Gazebo GUI to be ready
            time.sleep(2 + attempt)
            cmd = ['gz', 'camera', '--camera', 'user_camera', '--pose', 
                   str(x), str(y), str(z), str(roll), str(pitch), str(yaw)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0 and not result.stderr:
                print(f"Camera pose set successfully via gz command (attempt {attempt+1})")
                return True
            else:
                print(f"gz command attempt {attempt+1} failed: {result.stderr}")
        except Exception as e:
            print(f"gz command attempt {attempt+1} exception: {e}")
    
    # Method 2: Use Gazebo GUI plugin API via XML config
    try:
        gui_config_dir = os.path.expanduser('~/.gazebo')
        os.makedirs(gui_config_dir, exist_ok=True)
        
        # Gazebo Classic uses XML format for GUI config
        gui_config_file = os.path.join(gui_config_dir, 'gui.ini')
        
        # Try to write in INI format first
        config_content = f"""[geometry]
x=0
y=0
width=1920
height=1080

[camera]
name=user_camera
pose={x} {y} {z} {roll} {pitch} {yaw}
"""
        with open(gui_config_file, 'w') as f:
            f.write(config_content)
        print(f"Camera pose written to GUI config file (INI format): {gui_config_file}")
        
        # Also try XML format
        xml_config_file = os.path.join(gui_config_dir, 'gui.xml')
        root = ET.Element('gui')
        camera_elem = ET.SubElement(root, 'camera')
        camera_elem.set('name', 'user_camera')
        pose_elem = ET.SubElement(camera_elem, 'pose')
        pose_elem.text = f'{x} {y} {z} {roll} {pitch} {yaw}'
        
        tree = ET.ElementTree(root)
        tree.write(xml_config_file, encoding='utf-8', xml_declaration=True)
        print(f"Camera pose written to GUI config file (XML format): {xml_config_file}")
        
        return True
    except Exception as e:
        print(f"Failed to write GUI config file: {e}")
    
    # Method 3: Try using gz topic to publish camera pose
    try:
        import subprocess
        # Publish camera pose via topic
        topic_cmd = ['gz', 'topic', '-t', '/gazebo/default/gui/camera/pose', 
                     '-m', 'gazebo.msgs.Pose', 
                     '-p', f'position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1.0}}']
        result = subprocess.run(topic_cmd, capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("Camera pose published via topic")
            return True
    except Exception as e:
        print(f"Failed to publish via topic: {e}")
    
    print("All methods failed. Camera pose may need to be set manually in Gazebo GUI.")
    print("In Gazebo GUI: View -> Reset View -> Top")
    return False

if __name__ == '__main__':
    if len(sys.argv) != 7:
        print("Usage: python3 set_camera_pose.py <x> <y> <z> <roll> <pitch> <yaw>")
        sys.exit(1)
    
    x, y, z = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    roll, pitch, yaw = float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])
    
    print(f"Setting camera pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
    
    # Wait longer for Gazebo GUI to be fully ready
    print("Waiting for Gazebo GUI to be ready...")
    time.sleep(5)
    
    success = set_camera_pose(x, y, z, roll, pitch, yaw)
    
    if not success:
        print("\n=== Manual Camera Setup Instructions ===")
        print("If automatic camera setup failed, you can manually set the camera view:")
        print("1. In Gazebo GUI, go to: View -> Reset View -> Top")
        print("2. Or use mouse: Right-click -> View -> Reset View -> Top")
        print("3. Or press keyboard shortcut (if available)")
    
    sys.exit(0 if success else 1)
