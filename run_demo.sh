#!/bin/bash
set -e

echo "==========================================="
echo "   GTR1 Neuro-Symbolic MVP Startup Script  "
echo "==========================================="

echo "[0/6] Cleaning up old processes..."
pkill -f "ws_bridge.py" || true
pkill -f "cnn1_worker.py" || true
pkill -f "http.server 8000" || true
pkill -f "gtr1_main" || true
sleep 1

# 1. Генерация Python-биндингов для Protobuf
echo "[1/6] Generating Python Protobuf bindings..."
protoc -I=02_data_models -I=04_interfaces --python_out=src/ml_service 02_data_models/gtr1_structures.proto 04_interfaces/ipc_messages.proto

# 2. Сборка C++ проекта (генерирует C++ protobuf и компилирует ядро)
echo "[2/6] Building C++ Core..."
mkdir -p build && cd build
cmake ..
make -j4
cd ..

# 3. Запуск Python WebSocket Bridge (в фоне)
echo "[3/6] Starting WebSockets Bridge (Port 8080)..."
python3 src/ml_service/ws_bridge.py &
WS_PID=$!

# 4. Запуск Python ML Воркера CNN1 (в фоне)
echo "[4/6] Starting CNN1 ML Service (Port 5555)..."
python3 src/ml_service/cnn1_worker.py &
CNN_PID=$!

# 5. Запуск HTTP Сервера для Dashboard (в фоне)
echo "[5/6] Starting Dashboard UI (Port 8000)..."
cd 07_infrastructure/dashboard
python3 -m http.server 8000 &
HTTP_PID=$!
cd ../..

# 6. Запуск основного ядра C++ (блокирующий вызов)
echo "[6/6] Starting GTR1 C++ Core..."
sleep 2 # Ждем, пока биндинги ZMQ поднимутся
./build/gtr1_main

# При завершении C++ ядра убиваем фоновые Python-процессы
kill $WS_PID $CNN_PID $HTTP_PID
echo "GTR1 Shutdown complete."
