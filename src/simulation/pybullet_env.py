#!/usr/bin/env python3
"""
PyBullet Virtual Simulation Environment for GTR1
Provides closed-loop RGB-D perception and kinematic control.
Supports two sensor modes:
  1. Crisp / Clean: exact ground truth Z-buffer depth (zero noise)
  2. Emulated Sensor: realistic noise (Gaussian z-jitter, edge shadow dropouts)
"""

import sys
import os
import time
import argparse
import base64
import json
import numpy as np
import cv2
import zmq
import pybullet as p
import pybullet_data

# Ensure Protobuf bindings can be imported
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "ml_service"))
import ipc_messages_pb2 as ipc_pb
import gtr1_structures_pb2 as models_pb


class PyBulletEnv:
    def __init__(self, headless=True, sensor_mode="clean", fps=60.0):
        self.fps = fps
        self.dt = 1.0 / fps
        self.is_clean_mode = (sensor_mode.lower() == "clean")
        self.headless = headless

        # 1. Connect to PyBullet
        if not headless and "DISPLAY" in os.environ:
            try:
                self.client = p.connect(p.GUI)
                p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
                p.resetDebugVisualizerCamera(cameraDistance=3.5, cameraYaw=45, cameraPitch=-35, cameraTargetPosition=[1.0, 0.0, 0.5])
            except Exception:
                self.client = p.connect(p.DIRECT)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.dt)

        # 2. Build the office scene
        self.setup_scene()

        # 3. Setup mobile robot base & mounted RGB-D camera
        self.cam_height = 0.75    # Camera optical center height (0.75m)
        self.robot_height = 0.75  # Full robot height matching camera mount
        self.robot_radius = 0.22  # Radius of cylindrical chassis
        self.robot_pos = np.array([0.0, 0.0, self.robot_height / 2.0], dtype=np.float32) # Z = 0.375
        self.robot_yaw = 0.0  # Heading in radians
        self.setup_robot()

        # Camera intrinsics: TUM RGB-D Freiburg 1 benchmark standard
        # fx=517.3, fy=516.5, cx=318.6, cy=255.3, 640x480
        # Vertical FOV: 2 * atan((480/2) / 516.5) = 49.855 degrees
        self.width = 640
        self.height = 480
        self.fx = 517.3
        self.fy = 516.5
        self.cx = 318.6
        self.cy = 255.3
        self.fov = 49.855
        self.aspect = float(self.width) / float(self.height)
        self.near = 0.1
        self.far = 10.0
        self.depth_scale = 5000.0  # TUM RGB-D scale: 5000 units = 1.0 meter
        self.cam_pitch = np.radians(-25.0)

        # 4. ZeroMQ IPC Setup
        self.zmq_ctx = zmq.Context()
        # Sensor publisher (PyBullet -> GTR1 SLGW)
        self.pub_sensor = self.zmq_ctx.socket(zmq.PUB)
        self.pub_sensor.bind("tcp://127.0.0.1:5560")

        # Telemetry / Video publisher to Dashboard (PyBullet -> ws_bridge.py on dedicated port 5558)
        self.pub_telemetry = self.zmq_ctx.socket(zmq.PUB)
        self.pub_telemetry.bind("tcp://127.0.0.1:5558")

        # Kinematic command subscriber (GTR1 OP13 -> PyBullet)
        self.sub_cmd = self.zmq_ctx.socket(zmq.SUB)
        self.sub_cmd.connect("tcp://127.0.0.1:5561")
        self.sub_cmd.setsockopt_string(zmq.SUBSCRIBE, "")

        # Current commanded velocities (default initial forward exploration)
        self.cmd_vx = 0.20
        self.cmd_vy = 0.0
        self.cmd_wz = 0.35

        self.frame_id = 0
        print(f"[PyBullet] Environment initialized. Mode: {'CLEAN' if self.is_clean_mode else 'EMULATED_NOISE'}. Headless: {self.headless}", flush=True)

    def setup_scene(self):
        # Floor plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Office Table (at X=2.2, Y=0.0)
        self.table_id = p.loadURDF("table/table.urdf", [2.2, 0.0, 0.0])

        # Target item: Cup / Mug on table surface (tabletop height is approx 0.625m)
        # Load textured mug.urdf from pybullet_data with fallback
        try:
            self.cup_id = p.loadURDF("objects/mug.urdf", [2.0, 0.1, 0.63], globalScaling=2.5, useFixedBase=True)
        except Exception:
            cup_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.06, height=0.14)
            cup_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.06, length=0.14, rgbaColor=[0.95, 0.15, 0.15, 1.0])
            self.cup_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=cup_col,
                                            baseVisualShapeIndex=cup_vis, basePosition=[2.0, 0.1, 0.695])

        # Second item: Box / Laptop on table
        laptop_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.18, 0.02])
        laptop_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.18, 0.02], rgbaColor=[0.15, 0.25, 0.45, 1.0])
        self.laptop_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=laptop_col,
                                           baseVisualShapeIndex=laptop_vis, basePosition=[2.1, -0.25, 0.65])

        # Floor obstacle: Column / Chair between start and table (forces A* to route around)
        obs_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.18, height=0.7)
        obs_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.18, length=0.7, rgbaColor=[0.3, 0.5, 0.8, 1.0])
        self.obs_id = p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=obs_col,
                                        baseVisualShapeIndex=obs_vis, basePosition=[1.1, 0.25, 0.35])

        # 4 Boundary perimeter walls enclosing 5m x 5m room
        wall_vis_ns = p.createVisualShape(p.GEOM_BOX, halfExtents=[2.5, 0.05, 0.5], rgbaColor=[0.75, 0.75, 0.8, 0.8])
        wall_col_ns = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2.5, 0.05, 0.5])
        p.createMultiBody(0, wall_col_ns, wall_vis_ns, [1.0, 2.5, 0.5])   # North wall
        p.createMultiBody(0, wall_col_ns, wall_vis_ns, [1.0, -2.5, 0.5])  # South wall

        wall_vis_ew = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 2.5, 0.5], rgbaColor=[0.75, 0.75, 0.8, 0.8])
        wall_col_ew = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 2.5, 0.5])
        p.createMultiBody(0, wall_col_ew, wall_vis_ew, [3.5, 0.0, 0.5])   # East wall
        p.createMultiBody(0, wall_col_ew, wall_vis_ew, [-1.5, 0.0, 0.5])  # West wall

    def setup_robot(self):
        # Mobile chassis (cylinder body reaching camera height 0.75m from floor)
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.robot_radius, height=self.robot_height)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=self.robot_radius, length=self.robot_height, rgbaColor=[0.1, 0.7, 0.2, 1.0])
        self.robot_id = p.createMultiBody(baseMass=15.0, baseCollisionShapeIndex=col,
                                          baseVisualShapeIndex=vis, basePosition=self.robot_pos)

    def step_physics(self):
        # Poll incoming kinematic commands from GTR1 C++ core
        while True:
            try:
                cmd_data = self.sub_cmd.recv(flags=zmq.NOBLOCK)
                cmd = ipc_pb.KinematicCommand()
                cmd.ParseFromString(cmd_data)
                self.cmd_vx = cmd.velocity.x
                self.cmd_vy = cmd.velocity.y
                self.cmd_wz = cmd.angular_velocity.z
            except zmq.Again:
                break
            except Exception:
                break

        # Apply kinematic motion through rigid body velocity control (enforces collision boundaries)
        self.robot_yaw += self.cmd_wz * self.dt
        vx_world = float(self.cmd_vx * np.cos(self.robot_yaw) - self.cmd_vy * np.sin(self.robot_yaw))
        vy_world = float(self.cmd_vx * np.sin(self.robot_yaw) + self.cmd_vy * np.cos(self.robot_yaw))

        # Velocity drive: PyBullet physics engine prevents penetrating solid obstacles and walls
        p.resetBaseVelocity(self.robot_id,
                            linearVelocity=[vx_world, vy_world, 0.0],
                            angularVelocity=[0.0, 0.0, float(self.cmd_wz)])
        p.stepSimulation()

        # Read actual physical position after collision solving
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        clamped_x = float(np.clip(pos[0], -1.25, 3.25))
        clamped_y = float(np.clip(pos[1], -2.25, 2.25))
        self.robot_pos = np.array([clamped_x, clamped_y, self.robot_height / 2.0], dtype=np.float32)

        # Keep chassis upright with current heading
        q = p.getQuaternionFromEuler([0, 0, self.robot_yaw])
        p.resetBasePositionAndOrientation(self.robot_id, self.robot_pos.tolist(), q)

    def render_rgbd(self):
        cam_eye = np.array([self.robot_pos[0], self.robot_pos[1], self.cam_height], dtype=np.float32)
        fwd_x = np.cos(self.robot_yaw) * np.cos(self.cam_pitch)
        fwd_y = np.sin(self.robot_yaw) * np.cos(self.cam_pitch)
        fwd_z = np.sin(self.cam_pitch)
        cam_fwd = np.array([fwd_x, fwd_y, fwd_z], dtype=np.float32)
        cam_target = cam_eye + cam_fwd
        cam_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        view_mat = p.computeViewMatrix(cam_eye.tolist(), cam_target.tolist(), cam_up.tolist())
        left = -self.cx * self.near / self.fx
        right = (self.width - self.cx) * self.near / self.fx
        top = self.cy * self.near / self.fy
        bottom = -(self.height - self.cy) * self.near / self.fy
        proj_mat = p.computeProjectionMatrix(left, right, bottom, top, self.near, self.far)

        _, _, rgb_raw, depth_raw, _ = p.getCameraImage(self.width, self.height, view_mat, proj_mat)

        rgb_bgr = cv2.cvtColor(rgb_raw[:, :, :3], cv2.COLOR_RGB2BGR)
        depth_metric = self.far * self.near / (self.far - (self.far - self.near) * depth_raw)

        if not self.is_clean_mode:
            sigma = 0.001 + 0.0025 * (depth_metric ** 2)
            noise = np.random.normal(0.0, sigma, depth_metric.shape)
            depth_emulated = depth_metric + noise

            grad_x = np.abs(np.diff(depth_metric, axis=1, prepend=depth_metric[:, :1]))
            grad_y = np.abs(np.diff(depth_metric, axis=0, prepend=depth_metric[:1, :]))
            edge_mask = (grad_x > 0.08) | (grad_y > 0.08)
            depth_emulated[edge_mask] = 0.0
            depth_emulated[depth_emulated < 0.20] = 0.0
            depth_emulated[depth_emulated > 4.50] = 0.0
            final_depth_m = depth_emulated
        else:
            final_depth_m = depth_metric

        # Convert metric depth to standard 16-bit TUM RGB-D format (5000 units = 1.0 m)
        depth_raw_16u = np.clip(final_depth_m * self.depth_scale, 0, 65535).astype(np.uint16)

        z_axis = cam_fwd / np.linalg.norm(cam_fwd)
        x_axis = np.cross(z_axis, cam_up)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        T_cam_world = np.eye(4, dtype=np.float32)
        T_cam_world[0:3, 0] = x_axis
        T_cam_world[0:3, 1] = y_axis
        T_cam_world[0:3, 2] = z_axis
        T_cam_world[0:3, 3] = cam_eye

        # Эмуляция RGB-D-Inertia сенсора (IMU / VIO)
        if not self.is_clean_mode:
            # Шум измерений инерциальной позы (акселерометр/гироскоп)
            # Наземный колесный робот строго на ровном полу:
            # Высота камеры фиксирована (0.75м), крен и тангаж зафиксированы гравитацией IMU.
            # Шум VIO/одометрии действует только в горизонтальной плоскости (x, y, yaw).
            xy_noise = np.random.normal(0.0, 0.015, 2)
            yaw_noise = np.random.normal(0.0, 0.005)

            c, s = np.cos(yaw_noise), np.sin(yaw_noise)
            R_yaw = np.array([
                [c, -s, 0.0],
                [s,  c, 0.0],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)

            T_cam_out = T_cam_world.copy()
            T_cam_out[0:3, 0:3] = (R_yaw @ T_cam_out[0:3, 0:3]).astype(np.float32)
            T_cam_out[0, 3] += float(xy_noise[0])
            T_cam_out[1, 3] += float(xy_noise[1])
        else:
            T_cam_out = T_cam_world

        return rgb_bgr, depth_raw_16u, T_cam_out

    def publish_frame(self, rgb_bgr, depth_mm, T_cam_world, timestamp):
        self.frame_id += 1
        trace_id = f"SIM-{self.frame_id:05d}"

        _, rgb_bytes = cv2.imencode(".jpg", rgb_bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
        _, depth_bytes = cv2.imencode(".png", depth_mm)

        sensor_frame = ipc_pb.SensorFrame()
        sensor_frame.meta.timestamp = timestamp
        sensor_frame.meta.frame_id = self.frame_id
        sensor_frame.meta.trace_id = trace_id
        sensor_frame.rgb_image_data = rgb_bytes.tobytes()
        sensor_frame.depth_image_data = depth_bytes.tobytes()
        sensor_frame.is_clean_mode = self.is_clean_mode

        pose_flat = T_cam_world.flatten().tolist()
        sensor_frame.camera_pose.data.extend(pose_flat)

        payload = sensor_frame.SerializeToString()
        self.pub_sensor.send(payload)

        # Прямая трансляция кадра в Дашборд (порт 5558 -> ws_bridge -> 8081)
        try:
            b64_img = base64.b64encode(rgb_bytes).decode('utf-8')
            frame_telemetry = json.dumps({
                "type": "FRAME",
                "source": "SIM",
                "data": f"data:image/jpeg;base64,{b64_img}",
                "ts": round(timestamp, 3),
                "latency_ms": 4.5
            })
            self.pub_telemetry.send_string(frame_telemetry, flags=zmq.NOBLOCK)
        except Exception:
            pass

    def run_loop(self):
        print(f"[PyBullet] Streaming live RGB-D on tcp://127.0.0.1:5560 @ {self.fps} FPS...", flush=True)
        last_log = time.time()
        start_time = time.time()

        try:
            last_frame_time = time.time() - self.dt
            while True:
                t0 = time.time()
                current_time = t0 - start_time
                actual_elapsed = t0 - last_frame_time
                last_frame_time = t0

                steps = max(1, int(actual_elapsed / self.dt))
                steps = min(steps, 10) 
                
                print(f"[Debug] Stepping physics {steps} times...", flush=True)
                for _ in range(steps):
                    self.step_physics()
                    
                print("[Debug] Rendering RGB-D...", flush=True)
                rgb, depth, T_cam = self.render_rgbd()
                
                print("[Debug] Publishing frame...", flush=True)
                self.publish_frame(rgb, depth, T_cam, current_time)

                if time.time() - last_log > 1.5:
                    mode_str = "CLEAN" if self.is_clean_mode else "EMULATED"
                    print(f"[PyBullet] Frame #{self.frame_id:04d} | Mode: {mode_str} | "
                          f"Robot Pos: ({self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f}) | "
                          f"Yaw: {np.degrees(self.robot_yaw):.1f} deg | "
                          f"Cmd Vel: (vx={self.cmd_vx:.2f}, wz={self.cmd_wz:.2f})", flush=True)
                    last_log = time.time()

                elapsed = time.time() - t0
                if elapsed < 0: elapsed = 0.0 # Fix WSL time jumps
                sleep_time = max(0.0, self.dt - elapsed)
                print(f"[Debug] Sleeping for {sleep_time}...", flush=True)
                time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("[PyBullet] Interrupted by user.", flush=True)
        finally:
            p.disconnect()
            self.zmq_ctx.term()
            print("[PyBullet] Shutdown cleanly.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GTR1 PyBullet Simulation Environment")
    parser.add_argument("--headless", action="store_true", default=False, help="Run headless without GUI window")
    parser.add_argument("--sensor-mode", choices=["clean", "emulated"], default="clean",
                        help="Sensor mode: 'clean' (ideal analytical depth) or 'emulated' (realistic noise & edge dropouts)")
    parser.add_argument("--fps", type=float, default=60.0, help="Simulation and stream frame rate (60 FPS default)")
    args = parser.parse_args()

    env = PyBulletEnv(headless=args.headless, sensor_mode=args.sensor_mode, fps=args.fps)
    env.run_loop()
