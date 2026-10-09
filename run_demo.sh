#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Defaults
MODE="sim"
SENSOR_MODE="clean"
HEADLESS="--headless"
MAIN_ARGS="--sim"

# Parse CLI arguments
for arg in "$@"
do
    case $arg in
        --dataset)
            MODE="dataset"
            MAIN_ARGS="--dataset"
            ;;
        --emulated)
            SENSOR_MODE="emulated"
            ;;
        --clean)
            SENSOR_MODE="clean"
            ;;
        --gui)
            HEADLESS=""
            ;;
        --headless)
            HEADLESS="--headless"
            ;;
    esac
done

echo "=========================================================="
echo "      🚀 GTR1 Neuro-Symbolic MVP Orchestrator           "
echo "  Mode: $MODE | Sensor: $SENSOR_MODE | Display: ${HEADLESS:-GUI} "
echo "=========================================================="

echo "[1/7] Cleaning up previous processes and ports..."
fuser -k 5555/tcp 5556/tcp 5557/tcp 5558/tcp 5560/tcp 5561/tcp 8000/tcp 8081/tcp 2>/dev/null || true
pkill -9 -f "ws_bridge" 2>/dev/null || true
pkill -9 -f "cnn1_worker" 2>/dev/null || true
pkill -9 -f "pybullet" 2>/dev/null || true
pkill -9 -f "http.server 8000" 2>/dev/null || true
pkill -9 -f "gtr1_main" 2>/dev/null || true
sleep 1

# Trap сигналов для гарантированного завершения фоновых процессов
cleanup() {
    echo ""
    echo "[!] Terminating GTR1 background services..."
    kill "$WS_PID" "$CNN_PID" "$HTTP_PID" "$SIM_PID" 2>/dev/null || true
    echo "[OK] GTR1 MVP Stack Shutdown complete."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT

# 1. Генерация Protobuf для Python
echo "[2/7] Verifying Protobuf Python bindings..."
protoc -I=02_data_models -I=04_interfaces --python_out=src/ml_service     02_data_models/gtr1_structures.proto 04_interfaces/ipc_messages.proto

# 2. Сборка C++ проекта
echo "[3/7] Building C++ Core & Tests..."
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
cd ..

# 3. Запуск Python WebSocket Bridge (порт 8081)
echo "[4/7] Starting WebSockets Bridge (Port 8081)..."
python3 -u src/ml_service/ws_bridge.py > /tmp/gtr1_ws_bridge.log 2>&1 &
WS_PID=$!

# 4. Запуск Python ML Воркера CNN1 (порт 5555 REP, 5557 PUB)
echo "[5/7] Starting CNN1 ML Service (YOLOv8 + 3D Projection)..."
YOLO_CONFIG_DIR=/tmp/ultralytics python3 -u src/ml_service/cnn1_worker.py > /tmp/gtr1_cnn1_worker.log 2>&1 &
CNN_PID=$!

# 5. Запуск PyBullet симулятора (если выбран режим sim)
SIM_PID=""
if [ "$MODE" = "sim" ]; then
    echo "[6/7] Starting PyBullet Virtual Environment ($SENSOR_MODE mode)..."
    python3 -u src/simulation/pybullet_env.py $HEADLESS --sensor-mode "$SENSOR_MODE" > /tmp/gtr1_pybullet.log 2>&1 &
    SIM_PID=$!
else
    echo "[6/7] Skipping PyBullet (using recorded TUM dataset)..."
fi

# 6. Запуск HTTP Сервера для Web Dashboard (порт 8000)
echo "[7/7] Launching Three.js Dashboard UI (http://localhost:8000)..."
(cd 07_infrastructure/dashboard && python3 -m http.server 8000 > /tmp/gtr1_dashboard.log 2>&1) &
HTTP_PID=$!

sleep 2 # Ожидание инициализации ZMQ сокетов и HTTP сервера

echo "=========================================================="
echo " ✅ Dashboard is LIVE at: http://localhost:8000           "
echo " 📡 WebSockets Bridge at: ws://localhost:8081            "
echo " 🤖 Running GTR1 C++ Core Pipeline ($MAIN_ARGS)...        "
echo "=========================================================="

./build/gtr1_main $MAIN_ARGS
