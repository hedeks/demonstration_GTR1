import zmq
import cv2
import numpy as np
import uuid

# Сгенерированные файлы Protobuf (ожидаются в PYTHONPATH)
import ipc_messages_pb2
import gtr1_structures_pb2

import os
os.environ['YOLO_CONFIG_DIR'] = '/tmp/ultralytics'
from ultralytics import YOLO

class CNN1Worker:
    def __init__(self, port=5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        self.socket.bind(f"tcp://*:{port}")
        
        # PUB сокет для отправки кадров в Дашборд
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind("tcp://*:5557")
        
        print(f"[CNN1] Worker started on port {port}. Waiting for images...")
        
        print("[CNN1] Loading YOLOv8m model...")
        self.model = YOLO('yolov8m.pt') 
        print("[CNN1] Model loaded successfully.")
        
    def process_image(self, image_bytes):
        """Эмуляция нейросети. Декодирует JPG и возвращает список объектов."""
        np_arr = np.frombuffer(image_bytes, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if img is None:
            return []

        objects = []
        
        # ----------------------------------------------------
        # Запуск YOLOv8
        # ----------------------------------------------------
        results = self.model(img, verbose=False)
        
        for r in results:
            for box in r.boxes:
                # Получаем имя класса
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                conf = float(box.conf[0])
                
                # Фильтруем только уверенные детекции
                if conf < 0.3:
                    continue
                    
                # Получаем нормализованные координаты (0.0 - 1.0)
                xywhn = box.xywhn[0]
                nx, ny, nw, nh = float(xywhn[0]), float(xywhn[1]), float(xywhn[2]), float(xywhn[3])
                
                obj = gtr1_structures_pb2.MP11Object()
                obj.base.uuid = str(uuid.uuid4())
                obj.base.is_active = True
                obj.class_label_id = class_name
                
                # 3D Координаты (Эмуляция проекции 2D на 3D без Depth-карты)
                # Предполагаем камеру в 0,0,0 смотрящую вдоль Z.
                # В CV координатах Y направлен ВНИЗ (положительный)
                obj.bounding_box.center.x = (nx - 0.5) * 4.0  # X вправо
                obj.bounding_box.center.y = (ny - 0.5) * 3.0  # Y вниз
                obj.bounding_box.center.z = 2.0               # Z вперед
                obj.bounding_box.size.x = nw * 2.0
                obj.bounding_box.size.y = nh * 2.0
                obj.bounding_box.size.z = 0.5
                
                # Эмуляция 512-dim вектора от CLIP
                obj.feature_embedding.extend([0.1] * 512)
                
                # Сохраняем для 2D визуализации (костыль: прокидываем nx, ny в BBOX JSON ниже)
                # Мы передадим их через дополнительные атрибуты, не сериализуя в protobuf,
                # но для простоты мы просто в run() достанем их обратно
                
                objects.append((obj, nx, ny, nw, nh, conf))
                
        return objects

    def run(self):
        while True:
            # 1. Ждем запрос от C++ (SLGW)
            message = self.socket.recv()
            
            req = ipc_messages_pb2.DetectionRequest()
            req.ParseFromString(message)
            
            # Отправляем кадр в Дашборд
            import base64
            import json
            b64_img = base64.b64encode(req.rgb_image_data).decode('utf-8')
            frame_msg = json.dumps({
                "type": "FRAME",
                "data": f"data:image/jpeg;base64,{b64_img}"
            })
            self.pub_socket.send_string(frame_msg)
            
            # 2. Инференс
            detected_objs_tuple = self.process_image(req.rgb_image_data)
            detected_objs = [t[0] for t in detected_objs_tuple]
            
            # Отправляем BBOX для каждого объекта
            for o_tuple in detected_objs_tuple:
                o, nx, ny, nw, nh, conf = o_tuple
                bbox_msg = json.dumps({
                    "type": "BBOX",
                    "x": nx,
                    "y": ny,
                    "w": nw,
                    "h": nh,
                    "label": o.class_label_id,
                    "confidence": round(conf, 2)
                })
                self.pub_socket.send_string(bbox_msg)
            
            # 3. Формируем Protobuf ответ
            resp = ipc_messages_pb2.DetectionResponse()
            resp.meta.CopyFrom(req.meta)
            resp.detected_objects.extend(detected_objs)
            
            # 4. Отправляем ответ обратно C++ ядру
            self.socket.send(resp.SerializeToString())

if __name__ == "__main__":
    worker = CNN1Worker()
    worker.run()
