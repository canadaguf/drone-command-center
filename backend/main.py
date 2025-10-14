# backend/main.py
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response
from typing import List
import json
import logging


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("drone-backend")

app = FastAPI(
    title="Drone Command Backend",
    description="WebSocket relay between web UI and drone"
)


# Store connected clients
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self.drone_connection: WebSocket = None

    async def connect(self, websocket: WebSocket, is_drone: bool = False):
        await websocket.accept()
        if is_drone:
            self.drone_connection = websocket
            logger.info("Drone connected")
        else:
            self.active_connections.append(websocket)
            logger.info(f"Web client connected. Total: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket, is_drone: bool = False):
        if is_drone:
            self.drone_connection = None
            logger.info("Drone disconnected")
        else:
            self.active_connections.remove(websocket)
            logger.info(f"Web client disconnected. Total: {len(self.active_connections)}")

    async def send_to_drone(self, message: str):
        if self.drone_connection:
            await self.drone_connection.send_text(message)
        else:
            logger.warning("Drone not connected. Dropping command.")

    async def broadcast_to_web(self, message: str):
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except WebSocketDisconnect:
                disconnected.append(connection)
        # Clean up dead connections
        for conn in disconnected:
            self.active_connections.remove(conn)

manager = ConnectionManager()


@app.get("/")
async def root():
    return {"message": "Drone Command Backend is running. Use /health or /ws."}


@app.get("/favicon.ico")
async def favicon():
    return Response(status_code=204)  # No content


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    # Determine if this is the drone or a web client
    # Simple heuristic: drone connects with query param ?client=drone
    client_type = websocket.query_params.get("client")
    is_drone = (client_type == "drone")

    await manager.connect(websocket, is_drone=is_drone)
    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
                logger.info(f"Received: {msg}")

                if msg.get("source") == "web":
                    # Forward command to drone
                    await manager.send_to_drone(data)
                elif msg.get("source") == "drone":
                    # Broadcast telemetry/detections to all web clients
                    await manager.broadcast_to_web(data)
                else:
                    logger.warning(f"Unknown message source: {msg}")

            except json.JSONDecodeError:
                logger.error("Invalid JSON received")
    except WebSocketDisconnect:
        manager.disconnect(websocket, is_drone=is_drone)

@app.get("/health")
async def health_check():
    return JSONResponse({
        "status": "ok",
        "drone_connected": manager.drone_connection is not None
    })