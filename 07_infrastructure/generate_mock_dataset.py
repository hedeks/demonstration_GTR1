import os
import json
import cv2
import numpy as np

def create_mock_dataset(base_path, num_frames=100, fps=30):
    dataset_dir = os.path.join(base_path, "mock_dataset")
    rgb_dir = os.path.join(dataset_dir, "rgb")
    depth_dir = os.path.join(dataset_dir, "depth")
    
    os.makedirs(rgb_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    # Intrinsics (camera_matrix.json) - typical RealSense/Kinect parameters
    intrinsics = {
        "fx": 525.0,
        "fy": 525.0,
        "cx": 319.5,
        "cy": 239.5,
        "width": 640,
        "height": 480,
        "depth_scale": 5000.0 # Standard TUM format (1 unit = 1/5000 meter)
    }
    
    with open(os.path.join(dataset_dir, "camera_matrix.json"), 'w') as f:
        json.dump(intrinsics, f, indent=4)
        
    associations_lines = []
    
    print(f"Generating {num_frames} frames in {dataset_dir}...")
    
    for i in range(num_frames):
        timestamp = 1600000000.0 + (i * (1.0 / fps))
        ts_str = f"{timestamp:.6f}"
        
        # 1. Generate Fake RGB (A simple blue room with a moving red ball)
        rgb_img = np.full((480, 640, 3), (200, 150, 100), dtype=np.uint8) # BG color
        
        # Ball position moving across the screen
        ball_x = int(100 + (i / num_frames) * 400)
        ball_y = 240
        ball_radius = 50
        cv2.circle(rgb_img, (ball_x, ball_y), ball_radius, (0, 0, 255), -1) # Red ball
        
        rgb_filename = f"{ts_str}.png"
        cv2.imwrite(os.path.join(rgb_dir, rgb_filename), rgb_img)
        
        # 2. Generate Fake Depth (16-bit)
        # Background is 3 meters away (3 * 5000 = 15000)
        depth_img = np.full((480, 640), 15000, dtype=np.uint16)
        
        # Ball is closer, e.g., 1 meter away (1 * 5000 = 5000)
        cv2.circle(depth_img, (ball_x, ball_y), ball_radius, 5000, -1)
        
        depth_filename = f"{ts_str}.png"
        cv2.imwrite(os.path.join(depth_dir, depth_filename), depth_img)
        
        # 3. Add to associations.txt
        # Format: timestamp rgb_file timestamp depth_file
        associations_lines.append(f"{ts_str} rgb/{rgb_filename} {ts_str} depth/{depth_filename}\n")
        
    with open(os.path.join(dataset_dir, "associations.txt"), 'w') as f:
        f.writelines(associations_lines)
        
    print("Done! Dataset is ready.")

if __name__ == "__main__":
    # Create dataset in the current directory
    create_mock_dataset("./")
