#!/usr/bin/env python3

"""
Client library for Security Monitor API
"""

import asyncio
import aiohttp
import json
from datetime import datetime
from typing import Dict, List, Optional, Any
from .api.models import (
    ThreatAlert, SystemStatusResponse, ControlCommand, 
    ControlResponse, AnalyticsSummary
)

class SecurityMonitorClient:
    """Client for interacting with Security Monitor API"""
    
    def __init__(self, base_url: str = "http://localhost:8000", api_key: Optional[str] = None):
        self.base_url = base_url.rstrip('/')
        self.api_key = api_key
        self.session: Optional[aiohttp.ClientSession] = None
        
    async def __aenter__(self):
        """Async context manager entry"""
        self.session = aiohttp.ClientSession(
            headers={"Authorization": f"Bearer {self.api_key}"} if self.api_key else {}
        )
        return self
        
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        if self.session:
            await self.session.close()
    
    async def health_check(self) -> Dict[str, Any]:
        """Check API health"""
        async with self.session.get(f"{self.base_url}/health") as response:
            return await response.json()
    
    async def get_current_threats(self) -> List[ThreatAlert]:
        """Get current active threats"""
        async with self.session.get(f"{self.base_url}/threats") as response:
            data = await response.json()
            return [ThreatAlert(**threat) for threat in data]
    
    async def get_system_status(self) -> SystemStatusResponse:
        """Get current system status"""
        async with self.session.get(f"{self.base_url}/status") as response:
            data = await response.json()
            return SystemStatusResponse(**data)
    
    async def send_control_command(self, command: ControlCommand) -> ControlResponse:
        """Send control command to vehicle"""
        async with self.session.post(
            f"{self.base_url}/control",
            json=command.dict()
        ) as response:
            data = await response.json()
            return ControlResponse(**data)
    
    async def get_analytics_summary(self) -> AnalyticsSummary:
        """Get security analytics summary"""
        async with self.session.get(f"{self.base_url}/analytics/summary") as response:
            data = await response.json()
            return AnalyticsSummary(**data)
    
    async def emergency_stop(self) -> ControlResponse:
        """Trigger emergency stop"""
        command = ControlCommand(command="emergency_stop", priority=10)
        return await self.send_control_command(command)
    
    async def resume_operation(self) -> ControlResponse:
        """Resume normal operation"""
        command = ControlCommand(command="resume", priority=1)
        return await self.send_control_command(command)

class WebSocketClient:
    """WebSocket client for real-time threat updates"""
    
    def __init__(self, ws_url: str = "ws://localhost:8000/ws/threats"):
        self.ws_url = ws_url
        self.websocket: Optional[aiohttp.ClientWebSocketResponse] = None
    
    async def connect(self):
        """Connect to WebSocket"""
        self.websocket = await aiohttp.ClientSession().ws_connect(self.ws_url)
    
    async def listen_for_threats(self, callback):
        """Listen for threat updates and call callback function"""
        if not self.websocket:
            await self.connect()
        
        async for msg in self.websocket:
            if msg.type == aiohttp.WSMsgType.TEXT:
                data = json.loads(msg.data)
                await callback(data)
            elif msg.type == aiohttp.WSMsgType.ERROR:
                print(f'WebSocket error: {self.websocket.exception()}')
                break
    
    async def close(self):
        """Close WebSocket connection"""
        if self.websocket:
            await self.websocket.close()

# Example usage
async def main():
    """Example usage of the API client"""
    async with SecurityMonitorClient() as client:
        # Check health
        health = await client.health_check()
        print(f"API Health: {health}")
        
        # Get current threats
        threats = await client.get_current_threats()
        print(f"Active threats: {len(threats)}")
        
        # Get system status
        status = await client.get_system_status()
        print(f"System status: {status.status}")
        
        # Get analytics
        analytics = await client.get_analytics_summary()
        print(f"Total threats detected: {analytics.total_threats_detected}")

if __name__ == "__main__":
    asyncio.run(main())
