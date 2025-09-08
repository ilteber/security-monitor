from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, ConfigDict
from typing import List, Dict, Optional, Any
from datetime import datetime
import asyncio
import json
import random
import uuid

# Pydantic models
class ThreatAlert(BaseModel):
    id: str
    type: str
    severity: str
    confidence: int
    location: Dict[str, float]
    timestamp: str
    description: Optional[str] = None

class SystemStatus(BaseModel):
    active_threats: int
    system_health: Dict[str, str]
    last_update: datetime

class ControlCommand(BaseModel):
    command: str
    parameters: Optional[Dict[str, Any]] = None
    model_config = ConfigDict(arbitrary_types_allowed=True)

# Create FastAPI app
app = FastAPI(
    title="Security Monitor API",
    description="Real-time security monitoring system",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global state
active_threats: List[ThreatAlert] = []
connected_clients: List[WebSocket] = []

# Threat generation
def generate_random_threat() -> ThreatAlert:
    threat_types = ["pedestrian", "vehicle", "obstacle", "suspicious", "weather", "sensor_malfunction"]
    severities = ["low", "medium", "high", "critical"]
    
    return ThreatAlert(
        id=str(uuid.uuid4()),
        type=random.choice(threat_types),
        severity=random.choice(severities),
        confidence=random.randint(60, 95),
        location={
            "x": round(random.uniform(-50, 50), 2),
            "y": round(random.uniform(-50, 50), 2),
            "z": round(random.uniform(0, 10), 2)
        },
        timestamp=datetime.now().isoformat(),
        description=f"Detected {random.choice(threat_types)} at location"
    )

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try:
                await connection.send_text(message)
            except:
                pass

manager = ConnectionManager()

# Background task for generating threats
async def threat_generator():
    while True:
        # Randomly generate threats
        if random.random() < 0.3:  # 30% chance every 5 seconds
            threat = generate_random_threat()
            active_threats.append(threat)
            
            # Keep only last 10 threats
            if len(active_threats) > 10:
                active_threats.pop(0)
            
            # Broadcast to WebSocket clients
            await manager.broadcast(json.dumps({
                "type": "threat_update",
                "threats": [threat.model_dump() for threat in active_threats]
            }))
        
        await asyncio.sleep(5)

# Start background task
@app.on_event("startup")
async def startup_event():
    asyncio.create_task(threat_generator())

# API Endpoints
@app.get("/")
async def root():
    return {"message": "Security Monitor API", "status": "running"}

@app.get("/health")
async def health():
    return {
        "status": "healthy",
        "timestamp": datetime.now().isoformat(),
        "uptime_seconds": 0  # You can implement proper uptime tracking
    }

@app.get("/status")
async def get_status():
    return SystemStatus(
        active_threats=len(active_threats),
        system_health={"sensors": "operational", "ml_models": "loaded"},
        last_update=datetime.now()
    )

@app.get("/threats")
async def get_threats():
    return [threat.model_dump() for threat in active_threats]

@app.post("/control")
async def control_system(command: ControlCommand):
    if command.command == "test_threat":
        threat = generate_random_threat()
        active_threats.append(threat)
        return {"message": "Test threat generated", "threat": threat.model_dump()}
    elif command.command == "clear_threats":
        active_threats.clear()
        return {"message": "All threats cleared"}
    else:
        return {"message": f"Command '{command.command}' executed"}

@app.get("/analytics/summary")
async def get_analytics():
    if not active_threats:
        return {"total_threats": 0, "severity_breakdown": {}, "avg_confidence": 0}
    
    severity_count = {}
    for threat in active_threats:
        severity_count[threat.severity] = severity_count.get(threat.severity, 0) + 1
    
    avg_confidence = sum(t.confidence for t in active_threats) / len(active_threats)
    
    return {
        "total_threats": len(active_threats),
        "severity_breakdown": severity_count,
        "avg_confidence": round(avg_confidence, 2)
    }

# WebSocket endpoint
@app.websocket("/ws/threats")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # Keep connection alive
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

if __name__ == "__main__":
    import uvicorn
    print("Starting Security Monitor API Server...")
    print("API Documentation available at: http://localhost:8000/docs")
    print("WebSocket endpoint: ws://localhost:8000/ws/threats")
    uvicorn.run(app, host="0.0.0.0", port=8000)
