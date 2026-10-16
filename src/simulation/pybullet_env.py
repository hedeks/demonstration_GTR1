#!/usr/bin/env python3
"""
PyBullet Virtual Simulation Environment for GTR1
Provides closed-loop RGB-D perception and kinematic control.
Supports two sensor modes:
  1. Crisp / Clean: exact ground truth Z-buffer depth (zero noise)
  2. Emulated Sensor: realistic noise (Gaussian z-jitter, edge shadow dropouts)
Supports two execution modes:
  - Synchronous Lock-Step (--sync-step): physics step advances strictly upon receiving GTR1 command
  - Real-Time Free Loop: physics advances at target FPS with time.sleep
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
    def __init__(self, headless=True, sensor_mode="clean", fps=60.0, sync_step=False):
        self.fps = fps
        self.dt = 1.0 / fps
        self.is_clean_mode = (sensor_mode.lower() == "clean")
        self.headless = headless
        self.sync_step = sync_step

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
        sync_str = "LOCK-STEP" if self.sync_step else "REAL-TIME"
        print(f"[PyBullet] Environment initialized. Mode: {'CLEAN' if self.is_clean_mode else 'EMULATED_NOISE'}. Sync: {sync_str}. Headless: {self.headless}", flush=True)

    def setup_scene(self):
        # Floor plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Office Table (at X=2.2, Y=0.0)
        self.table_id = p.loadURDF("table/table.urdf", [2.2, 0.0, 0.0])

        # Target Coffee Cup (on top of table, height Z ~ 0.65 - 0.72m)
        target_urdf = "cube_small.urdf"
        cup_pos = [2.2, 0.0, 0.68]
        col_cup = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.05, height=0.10)
        vis_cup = p.createVisualShape(p.GEOM_CYLINDER, radius=0.05, length=0.10, rgbaColor=[0.8, 0.1, 0.1, 1.0])
        self.target_id = p.createMultiBody(baseMass=0.2, baseCollisionShapeIndex=col_cup,
                                           baseVisualShapeIndex=vis_cup, basePosition=cup_pos)

        # Office Chairs around
        chair_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.4], rgbaColor=[0.2, 0.3, 0.8, 1.0])
        chair_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25, 0.25, 0.4])
        self.chair1_id = p.createMultiBody(baseMass=5.0, baseCollisionShapeIndex=chair_col,
                                           baseVisualShapeIndex=chair_vis, basePosition=[2.2, 0.8, 0.4])
        self.chair2_id = p.createMultiBody(baseMass=5.0, baseCollisionShapeIndex=chair_col,
                                           baseVisualShapeIndex=chair_vis, basePosition=[2.2, -0.8, 0.4])

        # Perimeters / Walls
        wall_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[2.5, 0.1, 1.0], rgbaColor=[0.7, 0.7, 0.7, 1.0])
        wall_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[2.5, 0.1, 1.0])
        p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis, basePosition=[1.0, 2.5, 1.0])
        p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall_col, baseVisualShapeIndex=wall_vis, basePosition=[1.0, -2.5, 1.0])

        wall2_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 2.5, 1.0], rgbaColor=[0.7, 0.7, 0.7, 1.0])
        wall2_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 2.5, 1.0])
        p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall2_col, baseVisualShapeIndex=wall2_vis, basePosition=[3.5, 0.0, 1.0])
        p.createMultiBody(baseMass=0.0, baseCollisionShapeIndex=wall2_col, baseVisualShapeIndex=wall2_vis, basePosition=[-1.5, 0.0, 1.0])

    def setup_robot(self):
        # Mobile chassis (cylinder body reaching camera height 0.75m from floor)
        col = p.createCollisionShape(p.GEOM_CYLINDER, radius=self.robot_radius, height=self.robot_height)
        vis = p.createVisualShape(p.GEOM_CYLINDER, radius=self.robot_radius, length=self.robot_height, rgbaColor=[0.1, 0.7, 0.2, 1.0])
        self.robot_id = p.createMultiBody(baseMass=15.0, baseCollisionShapeIndex=col,
                                          baseVisualShapeIndex=vis, basePosition=self.robot_pos)

    def step_physics_single(self):
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
            except (zmq.Again, Exception):
                break

        self.step_physics_single()

    def render_rgbd(self):
        cam_eye = np.array([self.robot_pos[0], self.robot_pos[1], self.cam_height], dtype=np.float32)
        fwd_x = np.cos(self.robot_yaw) * np.cos(self.cam_pitch)
        fwd_y = np.sin(self.robot_yaw) * np.cos(self.cam_pitch)
        fwd_z = np.sin(self.cam_pitch)
        cam_fwd = np.array([fwd_x, fwd_y, fwd_z], dtype=np.float32)
        cam_target = cam_eye + cam_fwd

        cam_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        view_mat = p.computeViewMatrix(cam_eye.tolist(), cam_target.tolist(), cam_up.tolist())
        proj_mat = p.computeProjectionMatrixFOV(fov=self.fov, aspect=self.aspect, nearVal=self.near, farVal=self.far)

        w, h, rgb_arr, depth_arr, _ = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=view_mat,
            projectionMatrix=proj_mat,
            renderer=p.ER_BULLET_HARDWARE_OPENGL if not self.headless else p.ER_TINY_RENDERER
        )

        rgb_bgr = cv2.cvtColor(rgb_arr[:, :, :3], cv2.COLOR_RGB2BGR)

        # Analytical depth recovery from OpenGL non-linear buffer [0.0, 1.0]
        depth_gl = depth_arr.astype(np.float32)
        depth_metric = self.far * self.near / (self.far - (self.far - self.near) * depth_gl)

        if not self.is_clean_mode:
            # Emulate sensor noise: quadratic depth noise sigma ~ z^2 / (2 * fx * baseline)
            sigma_z = 0.0012 * (depth_metric ** 2) + 0.002
            noise = np.random.normal(0.0, sigma_z, depth_metric.shape).astype(np.float32)
            depth_emulated = depth_metric + noise

            # Edge shadow noise (flying edge pixels)
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
                "tick_id": self.frame_id,
                "data": f"data:image/jpeg;base64,{b64_img}",
                "ts": round(timestamp, 3),
                "latency_ms": 4.5
            })
            self.pub_telemetry.send_string(frame_telemetry, flags=zmq.NOBLOCK)
        except Exception:
            pass

    def run_loop(self):
        sync_mode_str = "LOCK-STEP (Synchronous)" if self.sync_step else "REAL-TIME (Asynchronous)"
        print(f"[PyBullet] Streaming live RGB-D on tcp://127.0.0.1:5560 @ {self.fps} FPS ({sync_mode_str})...", flush=True)
        last_log = time.time()
        start_time = time.time()

        try:
            last_frame_time = time.time() - self.dt
            while True:
                t0 = time.time()
                current_time = t0 - start_time

                if self.sync_step:
                    # 1. Рендеринг и публикация кадра с frame_id
                    rgb, depth, T_cam = self.render_rgbd()
                    self.publish_frame(rgb, depth, T_cam, current_time)

                    # 2. Ожидание ответной команды от GTR1 C++ ядра (Lock-step)
                    wait_t0 = time.time()
                    while time.time() - wait_t0 < 0.25:
                        try:
                            cmd_data = self.sub_cmd.recv(flags=zmq.NOBLOCK)
                            cmd = ipc_pb.KinematicCommand()
                            cmd.ParseFromString(cmd_data)
                            self.cmd_vx = cmd.velocity.x
                            self.cmd_vy = cmd.velocity.y
                            self.cmd_wz = cmd.angular_velocity.z
                            break
                        except zmq.Again:
                            time.sleep(0.0005)
                        except Exception:
                            break

                    # 3. Ровно один шаг физики строго после применения команды
                    self.step_physics_single()

                else:
                    # Асинхронный режим с ограничением FPS
                    actual_elapsed = t0 - last_frame_time
                    last_frame_time = t0

                    steps = max(1, int(actual_elapsed / self.dt))
                    steps = min(steps, 10) 
                    for _ in range(steps):
                        self.step_physics()

                    rgb, depth, T_cam = self.render_rgbd()
                    self.publish_frame(rgb, depth, T_cam, current_time)

                    elapsed = time.time() - t0
                    if elapsed < 0: elapsed = 0.0
                    sleep_time = max(0.0, self.dt - elapsed)
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                if time.time() - last_log > 1.5:
                    mode_str = "CLEAN" if self.is_clean_mode else "EMULATED"
                    print(f"[PyBullet] Frame #{self.frame_id:04d} | Mode: {mode_str} | "
                          f"Robot Pos: ({self.robot_pos[0]:.2f}, {self.robot_pos[1]:.2f}) | "
                          f"Yaw: {np.degrees(self.robot_yaw):.1f} deg | "
                          f"Cmd Vel: (vx={self.cmd_vx:.2f}, wz={self.cmd_wz:.2f})", flush=True)
                    last_log = time.time()

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
    parser.add_argument("--sync-step", action="store_true", default=False, help="Run physics in strict lock-step with GTR1 core commands")
    args = parser.parse_args()

    env = PyBulletEnv(headless=args.headless, sensor_mode=args.sensor_mode, fps=args.fps, sync_step=args.sync_step)
    env.run_loop()
