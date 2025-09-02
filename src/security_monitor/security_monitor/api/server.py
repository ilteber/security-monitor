#!/usr/bin/env python3

"""
FastAPI server for Security Monitor API
Provides REST endpoints for security monitoring data and control
"""

import asyncio
import json
from datetime import datetime
from typing import Dict, List, Optional
import uvicorn
from fastapi import FastAPI, HTTPException, Depends, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Security Monitor imports
from ..security_detector import SecurityDetector
from ..threat_classifier import ThreatClassifier
from ..emergency_handler import EmergencyHandler

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
    parameters: Optional[Dict[str, any]] = None

class SecurityMonitorAPI(Node):
    def __init__(self):
        super().__init__('security_monitor_api')
        
        # Initialize FastAPI app
        self.app = FastAPI(
            title="Security Monitor API",
            description="REST API for autonomous vehicle security monitoring",
            version="1.0.0"
        )
        
        # Add CORS middleware
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        
        # Security
        self.security = HTTPBearer()
        
        # Data storage
        self.current_threats: List[ThreatAlert] = []
        self.system_status = SystemStatus(
            status="operational",
            uptime=0.0,
            active_threats=0,
            system_health={},
            last_update=datetime.now()
        )
        
        # ROS2 publishers and subscribers
        self.control_publisher = self.create_publisher(Twist, '/vehicle/control', 10)
        self.status_publisher = self.create_publisher(String, '/api/status', 10)
        
        # Timer for status updates
        self.timer = self.create_timer(1.0, self.update_status)
        
        # Setup API routes
        self.setup_routes()
        
        self.get_logger().info('Security Monitor API server initialized')

    def setup_routes(self):
        """Setup FastAPI routes"""
        
        @self.app.get("/")
        async def root():
            return {"message": "Security Monitor API", "version": "1.0.0"}
        
        @self.app.get("/health")
        async def health_check():
            return {"status": "healthy", "timestamp": datetime.now()}
        
        @self.app.get("/threats", response_model=List[ThreatAlert])
        async def get_current_threats():
            """Get current active threats"""
            return self.current_threats
        
        @self.app.get("/status", response_model=SystemStatus)
        async def get_system_status():
            """Get current system status"""
            return self.system_status
        
        @self.app.post("/control")
        async def send_control_command(
            command: ControlCommand,
            background_tasks: BackgroundTasks,
            credentials: HTTPAuthorizationCredentials = Depends(self.security)
        ):
            """Send control command to vehicle"""
            try:
                if command.command == "emergency_stop":
                    # Publish emergency stop command
                    twist = Twist()
                    twist.linear.x = -8.0  # Emergency deceleration
                    twist.angular.z = 0.0
                    self.control_publisher.publish(twist)
                    
                    # Log the command
                    background_tasks.add_task(self.log_control_command, command)
                    
                    return {"status": "success", "message": "Emergency stop initiated"}
                
                elif command.command == "resume":
                    # Resume normal operation
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.control_publisher.publish(twist)
                    
                    return {"status": "success", "message": "Normal operation resumed"}
                
                else:
                    raise HTTPException(status_code=400, detail="Unknown command")
                    
            except Exception as e:
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.get("/analytics/summary")
        async def get_analytics_summary():
            """Get security analytics summary"""
            return {
                "total_threats_detected": len(self.current_threats),
                "threat_types": self.get_threat_type_summary(),
                "system_performance": self.get_performance_metrics(),
                "timestamp": datetime.now()
            }
        
        @self.app.websocket("/ws/threats")
        async def websocket_threats(websocket):
            """WebSocket for real-time threat updates"""
            await websocket.accept()
            try:
                while True:
                    # Send current threats
                    await websocket.send_json({
                        "threats": [threat.dict() for threat in self.current_threats],
                        "timestamp": datetime.now().isoformat()
                    })
                    await asyncio.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {e}")

    def update_status(self):
        """Update system status periodically"""
        self.system_status.uptime += 1.0
        self.system_status.active_threats = len(self.current_threats)
        self.system_status.last_update = datetime.now()
        
        # Publish status
        msg = String()
        msg.data = json.dumps(self.system_status.dict())
        self.status_publisher.publish(msg)

    def get_threat_type_summary(self) -> Dict[str, int]:
        """Get summary of threat types"""
        summary = {}
        for threat in self.current_threats:
            summary[threat.threat_type] = summary.get(threat.threat_type, 0) + 1
        return summary

    def get_performance_metrics(self) -> Dict[str, float]:
        """Get system performance metrics"""
        return {
            "cpu_usage": 45.2,
            "memory_usage": 67.8,
            "detection_latency": 0.15,
            "api_response_time": 0.05
        }

    async def log_control_command(self, command: ControlCommand):
        """Log control command asynchronously"""
        self.get_logger().info(f"Control command executed: {command.command}")

def create_app() -> FastAPI:
    """Create and configure FastAPI application"""
    rclpy.init()
    api_node = SecurityMonitorAPI()
    
    # Store node reference for cleanup
    api_node.app.state.ros_node = api_node
    
    return api_node.app

def main():
    """Main function to run the API server"""
    app = create_app()
    
    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8000,
        log_level="info"
    )

if __name__ == "__main__":
    main()
