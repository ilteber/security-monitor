#!/usr/bin/env python3

"""
Standalone FastAPI server for Security Monitor API
Provides REST endpoints for security monitoring data and control
"""

import asyncio
import json
from datetime import datetime
from typing import Dict, List, Optional, Any
import uvicorn
from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel

# Pydantic models for API
class ThreatAlert(BaseModel):
    timestamp: datetime
    threat_type: str
    severity: str
    confidence: float
    location: Optional[Dict[str, float]] = None
    description: str

class SystemStatus(BaseModel):
    status: str
    uptime: float
    active_threats: int
    system_health: Dict[str, str]
    last_update: datetime

class ControlCommand(BaseModel):
    command: str
    parameters: Optional[Dict[str, Any]] = None

# Create FastAPI app
app = FastAPI(
    title="Security Monitor API",
    description="REST API for autonomous vehicle security monitoring",
    version="0.4.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Security
security = HTTPBearer()

# Global state
system_start_time = datetime.now()
active_threats = []
system_health = {
    "security_detector": "healthy",
    "sensor_monitor": "healthy", 
    "ml_detector": "healthy",
    "api_server": "healthy"
}

@app.get("/")
async def root():
    """Root endpoint with basic info"""
    return {
        "message": "Security Monitor API",
        "version": "0.4.0",
        "status": "running",
        "endpoints": {
            "health": "/health",
            "threats": "/threats", 
            "status": "/status",
            "control": "/control",
            "docs": "/docs"
        }
    }

@app.get("/health")
async def health_check():
    """API health check endpoint"""
    return {
        "status": "healthy",
        "timestamp": datetime.now(),
        "uptime_seconds": (datetime.now() - system_start_time).total_seconds()
    }

@app.get("/threats", response_model=List[ThreatAlert])
async def get_current_threats():
    """Get current active security threats"""
    return active_threats

@app.post("/threats")
async def add_threat(threat: ThreatAlert):
    """Add a new threat alert"""
    active_threats.append(threat)
    return {"message": "Threat added successfully", "threat_id": len(active_threats)}

@app.delete("/threats/{threat_id}")
async def remove_threat(threat_id: int):
    """Remove a threat by ID"""
    if 0 <= threat_id < len(active_threats):
        removed_threat = active_threats.pop(threat_id)
        return {"message": "Threat removed", "threat": removed_threat}
    else:
        raise HTTPException(status_code=404, detail="Threat not found")

@app.get("/status", response_model=SystemStatus)
async def get_system_status():
    """Get comprehensive system status"""
    uptime = (datetime.now() - system_start_time).total_seconds()
    
    return SystemStatus(
        status="operational",
        uptime=uptime,
        active_threats=len(active_threats),
        system_health=system_health,
        last_update=datetime.now()
    )

@app.post("/control")
async def send_control_command(command: ControlCommand):
    """Send control commands to the security system"""
    
    if command.command == "emergency_stop":
        # Simulate emergency stop
        return {
            "message": "Emergency stop initiated",
            "command": command.command,
            "timestamp": datetime.now()
        }
    elif command.command == "reset_system":
        # Simulate system reset
        global active_threats
        active_threats.clear()
        return {
            "message": "System reset completed",
            "command": command.command,
            "timestamp": datetime.now()
        }
    elif command.command == "test_threat":
        # Add a test threat
        test_threat = ThreatAlert(
            timestamp=datetime.now(),
            threat_type="test",
            severity="low",
            confidence=0.8,
            description="Test threat for demonstration"
        )
        active_threats.append(test_threat)
        return {
            "message": "Test threat added",
            "command": command.command,
            "timestamp": datetime.now()
        }
    else:
        return {
            "message": f"Command '{command.command}' received",
            "command": command.command,
            "timestamp": datetime.now()
        }

@app.get("/analytics/summary")
async def get_analytics_summary():
    """Get security analytics summary"""
    threat_types = {}
    for threat in active_threats:
        threat_type = threat.threat_type
        threat_types[threat_type] = threat_types.get(threat_type, 0) + 1
    
    return {
        "total_threats": len(active_threats),
        "threat_types": threat_types,
        "system_uptime": (datetime.now() - system_start_time).total_seconds(),
        "last_updated": datetime.now()
    }

# WebSocket endpoint for real-time updates
@app.websocket("/ws/threats")
async def websocket_threats(websocket):
    """WebSocket endpoint for real-time threat updates"""
    await websocket.accept()
    
    try:
        while True:
            # Send current threat count
            await websocket.send_json({
                "type": "threat_count",
                "count": len(active_threats),
                "timestamp": datetime.now().isoformat()
            })
            
            # Send individual threats
            for i, threat in enumerate(active_threats):
                await websocket.send_json({
                    "type": "threat",
                    "id": i,
                    "data": threat.dict()
                })
            
            await asyncio.sleep(1)  # Update every second
            
    except Exception as e:
        print(f"WebSocket error: {e}")

if __name__ == "__main__":
    print("Starting Security Monitor API Server...")
    print("API Documentation available at: http://localhost:8000/docs")
    print("WebSocket endpoint: ws://localhost:8000/ws/threats")
    
    uvicorn.run(
        app, 
        host="0.0.0.0", 
        port=8000,
        log_level="info"
    )
