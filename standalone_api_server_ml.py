#!/usr/bin/env python3
"""
Enhanced Security Monitor API Server with ML Integration
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
import asyncio
import json
import random
import time
import uuid
from datetime import datetime

app = FastAPI(title="Security Monitor API", version="2.0.0")

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Pydantic models
class Threat(BaseModel):
    id: str
    type: str
    severity: str
    confidence: float
    location: Dict[str, float]
    timestamp: str
    description: str

class MLDetection(BaseModel):
    class_name: str
    confidence: float
    bbox: List[float]  # [x1, y1, x2, y2]
    timestamp: float

class SystemStatus(BaseModel):
    status: str
    active_threats: int
    ml_detections: int
    system_health: str
    uptime: float

class ControlCommand(BaseModel):
    action: str
    parameters: Optional[Dict[str, Any]] = None

class AnalyticsSummary(BaseModel):
    total_threats: int
    threats_by_type: Dict[str, int]
    average_confidence: float
    detection_rate: float

# Global state
active_threats = {}
ml_detections = []
connected_clients = []
system_start_time = time.time()

def generate_random_threat():
    threat_types = [
        "pedestrian_crossing", "vehicle_collision_risk", "obstacle_detected",
        "suspicious_activity", "weather_hazard", "sensor_malfunction"
    ]
    severities = {"low": 0.6, "medium": 0.75, "high": 0.9, "critical": 1.0}
    
    threat_type = random.choice(threat_types)
    severity_level = random.choice(list(severities.keys()))
    confidence = random.randint(60, 99)
    
    location = {
        "x": round(random.uniform(-50.0, 50.0), 2),
        "y": round(random.uniform(-50.0, 50.0), 2),
        "z": round(random.uniform(0.0, 10.0), 2)
    }
    
    description = f"Detected {threat_type.replace('_', ' ')} at location"
    
    threat_id = str(uuid.uuid4())
    timestamp = datetime.now().isoformat()
    
    return Threat(
        id=threat_id,
        type=threat_type,
        severity=severity_level,
        confidence=confidence,
        location=location,
        timestamp=timestamp,
        description=description
    )

def generate_ml_detection():
    classes = ['person', 'car', 'bicycle', 'motorcycle', 'bus', 'truck']
    class_name = random.choice(classes)
    confidence = random.uniform(0.6, 0.95)
    
    # Generate random bounding box
    x1 = random.uniform(0, 500)
    y1 = random.uniform(0, 400)
    x2 = x1 + random.uniform(50, 150)
    y2 = y1 + random.uniform(50, 100)
    
    return MLDetection(
        class_name=class_name,
        confidence=confidence,
        bbox=[x1, y1, x2, y2],
        timestamp=time.time()
    )

async def threat_generator():
    """Background task to generate threats and ML detections"""
    while True:
        # Generate threats with 30% probability
        if random.random() < 0.3:
            threat = generate_random_threat()
            active_threats[threat.id] = threat
            
            # Remove old threats (older than 30 seconds)
            current_time = time.time()
            threats_to_remove = []
            for threat_id, threat_obj in active_threats.items():
                threat_time = datetime.fromisoformat(threat_obj.timestamp).timestamp()
                if current_time - threat_time > 30:
                    threats_to_remove.append(threat_id)
            
            for threat_id in threats_to_remove:
                del active_threats[threat_id]
        
        # Generate ML detections with 50% probability
        if random.random() < 0.5:
            detection = generate_ml_detection()
            ml_detections.append(detection)
            
            # Keep only last 100 detections
            if len(ml_detections) > 100:
                ml_detections.pop(0)
        
        # Send updates to connected clients
        if connected_clients:
            update_data = {
                "type": "update",
                "threats": [threat.dict() for threat in active_threats.values()],
                "ml_detections": [detection.dict() for detection in ml_detections[-10:]],  # Last 10
                "timestamp": time.time()
            }
            
            for client in connected_clients[:]:  # Copy list to avoid modification during iteration
                try:
                    await client.send_text(json.dumps(update_data))
                except:
                    connected_clients.remove(client)
        
        await asyncio.sleep(1)

@app.on_event("startup")
async def startup_event():
    asyncio.create_task(threat_generator())

# API Endpoints
@app.get("/health")
async def health_check():
    return {"status": "healthy", "timestamp": time.time()}

@app.get("/status", response_model=SystemStatus)
async def get_status():
    uptime = time.time() - system_start_time
    return SystemStatus(
        status="operational",
        active_threats=len(active_threats),
        ml_detections=len(ml_detections),
        system_health="good",
        uptime=uptime
    )

@app.get("/threats", response_model=List[Threat])
async def get_threats():
    return list(active_threats.values())

@app.get("/ml_detections", response_model=List[MLDetection])
async def get_ml_detections():
    return ml_detections[-20:]  # Return last 20 detections

@app.post("/control")
async def control_system(command: ControlCommand):
    if command.action == "clear_threats":
        active_threats.clear()
        return {"message": "All threats cleared"}
    elif command.action == "clear_ml_detections":
        ml_detections.clear()
        return {"message": "All ML detections cleared"}
    elif command.action == "set_detection_threshold":
        if command.parameters and "threshold" in command.parameters:
            # In a real system, this would update the ML detector
            return {"message": f"Detection threshold set to {command.parameters['threshold']}"}
    else:
        raise HTTPException(status_code=400, detail="Unknown command")

@app.get("/analytics", response_model=AnalyticsSummary)
async def get_analytics():
    threats_by_type = {}
    total_confidence = 0
    threat_count = 0
    
    for threat in active_threats.values():
        threat_type = threat.type
        threats_by_type[threat_type] = threats_by_type.get(threat_type, 0) + 1
        total_confidence += threat.confidence
        threat_count += 1
    
    avg_confidence = total_confidence / threat_count if threat_count > 0 else 0
    detection_rate = len(ml_detections) / max(1, time.time() - system_start_time)
    
    return AnalyticsSummary(
        total_threats=threat_count,
        threats_by_type=threats_by_type,
        average_confidence=avg_confidence,
        detection_rate=detection_rate
    )

# WebSocket endpoint
@app.websocket("/ws/threats")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_clients.append(websocket)
    
    try:
        while True:
            # Keep connection alive
            await websocket.receive_text()
    except WebSocketDisconnect:
        connected_clients.remove(websocket)

if __name__ == "__main__":
    import uvicorn
    print("Starting Enhanced Security Monitor API Server with ML Integration...")
    print("API Documentation available at: http://localhost:8000/docs")
    print("WebSocket endpoint: ws://localhost:8000/ws/threats")
    uvicorn.run(app, host="0.0.0.0", port=8000)
