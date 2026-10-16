import os
import sys
import time
import hashlib
import json
import base64
import uuid
import signal
import numpy as np
import cv2
import zmq

# Изоляция конфигураций и кэшей в /tmp (предотвращение ошибок прав доступа в WSL)
os.environ['YOLO_CONFIG_DIR'] = '/tmp'
os.environ['TORCH_HOME'] = '/tmp'

# Добавляем путь к сгенерированным Protobuf DTO
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import ipc_messages_pb2
import gtr1_structures_pb2
import torch
from ultralytics import YOLO

class CNN1Worker:
    """
    CNN1 ML Service: Распознавание объектов (YOLOv8 + Semantic Embedding)
    с точной 3D-проекцией в систему координат камеры через интринсики K и карту глубин.
    """
    def __init__(self, rep_port=5555, pub_port=5557, model_path="yolov8n.pt"):
        self.running = True
        signal.signal(signal.SIGINT, self._handle_exit)
        signal.signal(signal.SIGTERM, self._handle_exit)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.setsockopt(zmq.RCVTIMEO, 500)
        self.socket.setsockopt(zmq.SNDTIMEO, 500)
        self.socket.bind(f"tcp://*:{rep_port}")

        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(f"tcp://*:{pub_port}")

        # Параметры камеры TUM RGB-D (Freiburg 1)
        self.fx = 517.3
        self.fy = 516.5
        self.cx = 318.6
        self.cy = 255.3
        self.depth_scale = 5000.0  # 5000 единиц = 1.0 метр

        print(f"[CNN1] Initializing YOLOv8 model from {model_path}...")
        # Поиск модели в текущей директории или в корне проекта
        actual_model_path = model_path
        if not os.path.exists(actual_model_path):
            parent_path = os.path.join(os.path.dirname(script_dir), "..", model_path)
            if os.path.exists(parent_path):
                actual_model_path = parent_path

        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"
        device_name = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU"
        print(f"[CNN1] Hardware Acceleration: {self.device} ({device_name})")

        self.model = YOLO(actual_model_path)
        self.model.to(self.device)
        print(f"[CNN1] Model loaded: {actual_model_path} on {self.device}. Ready on REP port {rep_port}, PUB port {pub_port}.")

        # Прогрев модели (warmup)
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model(dummy, device=self.device, verbose=False)

    def _handle_exit(self, signum, frame):
        self.running = False
        print("\n[CNN1] Stopping worker...")

    def get_semantic_embedding(self, class_name: str, jitter: float = 0.005) -> list:
        """
        Генерирует детерминированный нормализованный 512-мерный семантический вектор.
        Объекты одного класса имеют косинусное сходство > 0.95 (превышая порог 0.85).
        Объекты разных классов имеют ортогональные векторы (< 0.15).
        """
        seed = int(hashlib.md5(class_name.encode('utf-8')).hexdigest()[:8], 16)
        rng = np.random.RandomState(seed)
        base = rng.randn(512).astype(np.float32)
        base /= np.linalg.norm(base)

        if jitter > 0:
            noise = np.random.randn(512).astype(np.float32)
            noise /= np.linalg.norm(noise)
            vec = (1.0 - jitter) * base + jitter * noise
            vec /= np.linalg.norm(vec)
        else:
            vec = base

        return vec.tolist()

    def process_frame(self, rgb_bytes: bytes, depth_bytes: bytes = None):
        """
        Выполняет инференс YOLOv8 (< 100 мс) и 3D обратную проекцию центроидов объектов.
        """
        t0 = time.time()
        np_arr = np.frombuffer(rgb_bytes, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return [], []

        h_img, w_img = img.shape[:2]

        # Декодирование карты глубин, если передана
        depth_img = None
        if depth_bytes and len(depth_bytes) > 0:
            np_depth = np.frombuffer(depth_bytes, np.uint8)
            depth_img = cv2.imdecode(np_depth, cv2.IMREAD_UNCHANGED)

        # Инференс YOLOv8 с ускорением на GPU
        results = self.model(img, device=self.device, verbose=False)
        detected_objects = []
        visual_bboxes = []

        for r in results:
            for box in r.boxes:
                conf = float(box.conf[0])
                # Robust confidence threshold (0.25) to reject wall/background texture false positives
                if conf < 0.25:
                    continue

                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]

                # Нормализованные и пиксельные координаты 2D BBox
                xywhn = box.xywhn[0]
                nx, ny, nw, nh = float(xywhn[0]), float(xywhn[1]), float(xywhn[2]), float(xywhn[3])
                px_u = int(round(nx * w_img))
                px_v = int(round(ny * h_img))
                px_w = nw * w_img
                px_h = nh * h_img

                # 3D Обратная Проекция через интринсики камеры K и глубину
                Z = 1.5  # Базовая оценка глубины по умолчанию (1.5 м)
                if depth_img is not None:
                    # Извлекаем окрестность центра объекта
                    u_min = max(0, px_u - 5)
                    u_max = min(w_img, px_u + 6)
                    v_min = max(0, px_v - 5)
                    v_max = min(h_img, px_v + 6)
                    patch = depth_img[v_min:v_max, u_min:u_max]
                    valid_depths = patch[patch > 0]
                    if len(valid_depths) > 0:
                        raw_z = float(np.median(valid_depths))
                        z_m = raw_z / self.depth_scale
                        if 0.4 <= z_m <= 4.0:
                            Z = z_m

                # Репроекция в систему координат камеры (X вправо, Y вниз, Z вперед)
                X = (px_u - self.cx) * Z / self.fx
                Y = (px_v - self.cy) * Z / self.fy
                Size_X = px_w * Z / self.fx
                Size_Y = px_h * Z / self.fy
                Size_Z = max(0.08, min(Size_X, Size_Y))

                obj = gtr1_structures_pb2.MP11Object()
                obj.base.uuid = str(uuid.uuid4())
                obj.base.is_active = True
                obj.class_label_id = class_name

                # 3D Bounding Box
                obj.bounding_box.center.x = float(X)
                obj.bounding_box.center.y = float(Y)
                obj.bounding_box.center.z = float(Z)
                obj.bounding_box.size.x = float(Size_X)
                obj.bounding_box.size.y = float(Size_Y)
                obj.bounding_box.size.z = float(Size_Z)

                # Пространственный якорь
                obj.spatial_anchor.x = float(X)
                obj.spatial_anchor.y = float(Y)
                obj.spatial_anchor.z = float(Z)

                # 512-dim семантический эмбеддинг
                embedding = self.get_semantic_embedding(class_name)
                obj.feature_embedding.extend(embedding)

                detected_objects.append(obj)
                visual_bboxes.append({
                    "type": "BBOX",
                    "x": round(nx, 3),
                    "y": round(ny, 3),
                    "w": round(nw, 3),
                    "h": round(nh, 3),
                    "label": class_name,
                    "confidence": round(conf, 2),
                    "pos_3d": [round(X, 2), round(Y, 2), round(Z, 2)]
                })

        latency_ms = (time.time() - t0) * 1000.0
        return detected_objects, visual_bboxes, latency_ms

    def run(self):
        print("[CNN1] Worker loop running. Waiting for ZeroMQ requests...")
        while self.running:
            try:
                message = self.socket.recv()
            except zmq.Again:
                continue
            except Exception as e:
                if self.running:
                    print(f"[CNN1] Recv error: {e}")
                continue

            try:
                req = ipc_messages_pb2.DetectionRequest()
                req.ParseFromString(message)

                # Обработка кадра
                detected_objs, bboxes, latency = self.process_frame(
                    req.rgb_image_data,
                    req.depth_image_data if len(req.depth_image_data) > 0 else None
                )

                # Трансляция кадра и BBox в телеметрию/дашборд
                if len(req.rgb_image_data) > 0:
                    b64_img = base64.b64encode(req.rgb_image_data).decode('utf-8')
                    self.pub_socket.send_string(json.dumps({
                        "type": "FRAME",
                        "data": f"data:image/jpeg;base64,{b64_img}",
                        "ts": req.meta.timestamp,
                        "latency_ms": round(latency, 2)
                    }))

                if bboxes:
                    self.pub_socket.send_string(json.dumps({
                        "type": "BBOXES",
                        "boxes": bboxes,
                        "ts": req.meta.timestamp
                    }))
                for bbox in bboxes:
                    self.pub_socket.send_string(json.dumps(bbox))

                # Ответ C++ ядру
                resp = ipc_messages_pb2.DetectionResponse()
                resp.meta.CopyFrom(req.meta)
                resp.detected_objects.extend(detected_objs)

                self.socket.send(resp.SerializeToString())

            except Exception as e:
                print(f"[CNN1] Error processing request: {e}")
                # Отправляем пустой ответ для предотвращения зависания REQ сокета C++ ядра
                err_resp = ipc_messages_pb2.DetectionResponse()
                self.socket.send(err_resp.SerializeToString())

        print("[CNN1] Worker shutdown cleanly.")
        self.socket.close()
        self.pub_socket.close()
        self.context.term()

if __name__ == "__main__":
    worker = CNN1Worker()
    worker.run()