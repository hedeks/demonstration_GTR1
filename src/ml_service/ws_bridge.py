import asyncio
import signal
import sys
import websockets
import zmq
import zmq.asyncio

async def bridge():
    print("[WS Bridge] Starting ZeroMQ to WebSocket Bridge on port 8081...")
    
    ctx = zmq.asyncio.Context()
    sub_socket = ctx.socket(zmq.SUB)
    sub_socket.connect("tcp://127.0.0.1:5556")  # C++ Core Telemetry
    sub_socket.connect("tcp://127.0.0.1:5557")  # Python CNN1 Video & BBoxes
    sub_socket.connect("tcp://127.0.0.1:5558")  # PyBullet Live Camera Stream (RGB)
    sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")
    
    connected_clients = set()
    
    pub_socket = ctx.socket(zmq.PUB)
    pub_socket.bind("tcp://127.0.0.1:5562")

    async def handler(websocket):
        client_ip = websocket.remote_address
        print(f"[WS Bridge] New Dashboard Client Connected: {client_ip}")
        connected_clients.add(websocket)
        try:
            async for msg in websocket:
                if isinstance(msg, str) and (msg.startswith("TARGET:") or msg.startswith("CMD:") or msg.startswith("NAV_POSE:")):
                    print(f"[WS Bridge] Forwarding command: {msg}")
                    await pub_socket.send_string(msg)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            connected_clients.discard(websocket)
            print(f"[WS Bridge] Dashboard Client Disconnected: {client_ip}")

    # Запускаем WebSocket сервер на 0.0.0.0:8081 для поддержки браузера из Windows и WSL
    server = await websockets.serve(handler, host="0.0.0.0", port=8081, max_size=None)
    print("[WS Bridge] Listening for ZMQ telemetry on 5556/5557/5558 -> WebSocket ws://0.0.0.0:8081")
    
    while True:
        try:
            message = await sub_socket.recv_string()
            if connected_clients:
                # Рассылка всем активным вкладкам дашборда
                await asyncio.gather(
                    *(client.send(message) for client in list(connected_clients)),
                    return_exceptions=True
                )
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"[WS Bridge] Message bridging warning: {e}")

    server.close()
    await server.wait_closed()
    sub_socket.close()
    ctx.term()
    print("[WS Bridge] Clean shutdown completed.")

if __name__ == "__main__":
    try:
        asyncio.run(bridge())
    except KeyboardInterrupt:
        print("\n[WS Bridge] Stopped by user.")