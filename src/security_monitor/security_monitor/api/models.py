#!/usr/bin/env python3

"""
Pydantic models for Security Monitor API
"""

from datetime import datetime
from typing import Dict, List, Optional, Any
from pydantic import BaseModel, Field
from enum import Enum

class ThreatSeverity(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

class ThreatType(str, Enum):
    COLLISION = "collision"
    PEDESTRIAN = "pedestrian"
    VEHICLE = "vehicle"
    OBSTACLE = "obstacle"
    WEATHER = "weather"
    SYSTEM = "system"
    COMMUNICATION = "communication"

class SystemStatus(str, Enum):
    OPERATIONAL = "operational"
    DEGRADED = "degraded"
    CRITICAL = "critical"
    OFFLINE = "offline"

class Location(BaseModel):
    x: float = Field(..., description="X coordinate in meters")
    y: float = Field(..., description="Y coordinate in meters")
    z: Optional[float] = Field(None, description="Z coordinate in meters")

class ThreatAlert(BaseModel):
    id: str = Field(..., description="Unique threat identifier")
    timestamp: datetime = Field(..., description="Detection timestamp")
    threat_type: ThreatType = Field(..., description="Type of threat detected")
    severity: ThreatSeverity = Field(..., description="Severity level")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Detection confidence")
    location: Optional[Location] = Field(None, description="Threat location")
    description: str = Field(..., description="Detailed threat description")
    sensor_data: Optional[Dict[str, Any]] = Field(None, description="Raw sensor data")
    mitigation_actions: List[str] = Field(default_factory=list, description="Applied mitigation actions")

class SystemHealth(BaseModel):
    cpu_usage: float = Field(..., ge=0.0, le=100.0, description="CPU usage percentage")
    memory_usage: float = Field(..., ge=0.0, le=100.0, description="Memory usage percentage")
    gpu_usage: Optional[float] = Field(None, ge=0.0, le=100.0, description="GPU usage percentage")
    detection_latency: float = Field(..., ge=0.0, description="Detection latency in seconds")
    api_response_time: float = Field(..., ge=0.0, description="API response time in seconds")
    sensor_status: Dict[str, str] = Field(..., description="Status of each sensor")
    model_status: Dict[str, str] = Field(..., description="Status of ML models")

class SystemStatusResponse(BaseModel):
    status: SystemStatus = Field(..., description="Overall system status")
    uptime: float = Field(..., ge=0.0, description="System uptime in seconds")
    active_threats: int = Field(..., ge=0, description="Number of active threats")
    system_health: SystemHealth = Field(..., description="Detailed system health")
    last_update: datetime = Field(..., description="Last status update timestamp")

class ControlCommand(BaseModel):
    command: str = Field(..., description="Control command to execute")
    parameters: Optional[Dict[str, Any]] = Field(None, description="Command parameters")
    priority: int = Field(default=1, ge=1, le=10, description="Command priority (1-10)")

class ControlResponse(BaseModel):
    status: str = Field(..., description="Command execution status")
    message: str = Field(..., description="Status message")
    timestamp: datetime = Field(..., description="Command execution timestamp")
    command_id: Optional[str] = Field(None, description="Unique command identifier")

class AnalyticsSummary(BaseModel):
    total_threats_detected: int = Field(..., ge=0, description="Total threats detected")
    threat_types: Dict[str, int] = Field(..., description="Count by threat type")
    system_performance: SystemHealth = Field(..., description="System performance metrics")
    detection_accuracy: float = Field(..., ge=0.0, le=1.0, description="Overall detection accuracy")
    false_positive_rate: float = Field(..., ge=0.0, le=1.0, description="False positive rate")
    timestamp: datetime = Field(..., description="Analytics timestamp")

class WebSocketMessage(BaseModel):
    message_type: str = Field(..., description="Type of WebSocket message")
    data: Dict[str, Any] = Field(..., description="Message data")
    timestamp: datetime = Field(..., description="Message timestamp")

class ErrorResponse(BaseModel):
    error: str = Field(..., description="Error message")
    detail: Optional[str] = Field(None, description="Detailed error information")
    timestamp: datetime = Field(..., description="Error timestamp")
    request_id: Optional[str] = Field(None, description="Request identifier")
