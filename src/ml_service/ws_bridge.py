import asyncio
import websockets
import zmq
import zmq.asyncio

async def bridge():
    print("[WS Bridge] Starting ZeroMQ to WebSocket Bridge...")
    
    # ZeroMQ SUB socket (Слушаем телеметрию от C++ ядра)
    ctx = zmq.asyncio.Context()
    sub_socket = ctx.socket(zmq.SUB)
    sub_socket.connect("tcp://127.0.0.1:5556") # От C++ Core
    sub_socket.connect("tcp://127.0.0.1:5557") # От Python CNN1
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    
    connected_clients = set()
    
    async def handler(websocket):
        print(f"[WS Bridge] New Web Client Connected: {websocket.remote_address}")
        connected_clients.add(websocket)
        try:
            await websocket.wait_closed()
        finally:
            connected_clients.remove(websocket)
            print("[WS Bridge] Web Client Disconnected")

    # Запускаем WebSocket сервер для Дашборда (0.0.0.0 для Docker/WSL)
    start_server = websockets.serve(handler, host=None, port=8081, max_size=None)
    await start_server
    
    print("[WS Bridge] Listening for ZMQ telemetry on port 5556, bridging to WS port 8081...")
    
    while True:
        try:
            # Ждем сообщение от C++
            message = await sub_socket.recv_string()
            
            # Рассылаем всем подключенным браузерам
            if connected_clients:
                await asyncio.gather(*(client.send(message) for client in connected_clients))
        except Exception as e:
            print(f"[WS Bridge] Error bridging message: {e}")

if __name__ == "__main__":
    asyncio.run(bridge())
