import zmq
import time

class MLZeroMQClient:
    """
    ZeroMQ Клиент для связи Python ML-микросервиса с C++ Ядром.
    Паттерн: REQ/REP (Синхронный запрос) или SUB/PUB.
    Для MVP используем REQ/REP: Python-воркер просит кадр, Ядро отдает.
    """
    def __init__(self, core_address="tcp://localhost:5555"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.connect(core_address)
        print(f"[ZMQ] Connected to C++ Core at {core_address}")

    def request_frame(self):
        """
        Запрашивает новый RGB-кадр у ядра.
        """
        # Отправляем пустой запрос (или Protobuf сообщение с метаданными)
        self.socket.send_string("GET_FRAME")
        
        # Получаем бинарные данные картинки (сжатые в JPG/PNG для скорости)
        message = self.socket.recv()
        
        # В реальной системе здесь будет десериализация Protobuf DetectionRequest
        # Возвращаем сырые байты
        return message

    def send_detections(self, serialized_protobuf_response):
        """
        Отправляет массив распознанных объектов (MP11) обратно в ядро.
        """
        self.socket.send(serialized_protobuf_response)
        # Ожидаем подтверждения от ядра
        ack = self.socket.recv_string()
        return ack == "OK"
