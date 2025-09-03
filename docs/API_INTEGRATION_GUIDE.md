# API Integration Guide

## Overview

The Security Monitor provides a comprehensive REST API and WebSocket interface for real-time security monitoring and vehicle control. This guide covers API usage, authentication, and integration examples.

## Quick Start

### Starting the API Server

```bash
# Using ROS2 launch
ros2 launch security_monitor api_server.launch.py

# Or directly with Python
python3 src/security_monitor/security_monitor/api/server.py
```

The API server will be available at `http://localhost:8000`

### API Documentation

Interactive API documentation is available at:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Authentication

The API uses Bearer token authentication:

```python
headers = {"Authorization": "Bearer your_api_key"}
```

## Core Endpoints

### Health Check
```http
GET /health
```

### Get Current Threats
```http
GET /threats
```

### Get System Status
```http
GET /status
```

### Send Control Commands
```http
POST /control
Content-Type: application/json

{
  "command": "emergency_stop",
  "parameters": {},
  "priority": 10
}
```

### Get Analytics Summary
```http
GET /analytics/summary
```

## WebSocket Integration

### Real-time Threat Updates

```python
import asyncio
import aiohttp
import json

async def listen_for_threats():
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect('ws://localhost:8000/ws/threats') as ws:
            async for msg in ws:
                if msg.type == aiohttp.WSMsgType.TEXT:
                    data = json.loads(msg.data)
                    print(f"New threat data: {data}")

asyncio.run(listen_for_threats())
```

## Python Client Library

### Basic Usage

```python
from security_monitor.api_client import SecurityMonitorClient

async def main():
    async with SecurityMonitorClient() as client:
        # Check system health
        health = await client.health_check()
        print(f"System health: {health}")
        
        # Get current threats
        threats = await client.get_current_threats()
        print(f"Active threats: {len(threats)}")
        
        # Emergency stop if needed
        if len(threats) > 0:
            response = await client.emergency_stop()
            print(f"Emergency stop: {response.status}")

asyncio.run(main())
```

### Advanced Integration

```python
from security_monitor.api_client import SecurityMonitorClient, WebSocketClient

class SecurityController:
    def __init__(self):
        self.api_client = SecurityMonitorClient()
        self.ws_client = WebSocketClient()
        self.threat_threshold = 5
    
    async def start_monitoring(self):
        # Start WebSocket listener
        await self.ws_client.connect()
        
        # Monitor threats in real-time
        await self.ws_client.listen_for_threats(self.handle_threat_update)
    
    async def handle_threat_update(self, data):
        threats = data.get('threats', [])
        
        if len(threats) >= self.threat_threshold:
            # Trigger emergency response
            async with self.api_client as client:
                await client.emergency_stop()
                print("Emergency stop triggered due to high threat count")
```

## Error Handling

The API returns standard HTTP status codes:

- `200 OK`: Successful request
- `400 Bad Request`: Invalid request parameters
- `401 Unauthorized`: Authentication required
- `404 Not Found`: Resource not found
- `500 Internal Server Error`: Server error

Error responses include detailed information:

```json
{
  "error": "Invalid command",
  "detail": "Command 'invalid_cmd' is not supported",
  "timestamp": "2025-09-03T10:30:00Z",
  "request_id": "req_12345"
}
```

## Rate Limiting

The API implements rate limiting to prevent abuse:
- 100 requests per minute per IP
- 10 control commands per minute per authenticated user

## Security Considerations

1. **Authentication**: Always use HTTPS in production
2. **API Keys**: Rotate API keys regularly
3. **Input Validation**: Validate all input parameters
4. **Rate Limiting**: Respect rate limits to avoid blocking
5. **Error Handling**: Implement proper error handling and retry logic

## Integration Examples

### ROS2 Node Integration

```python
import rclpy
from rclpy.node import Node
from security_monitor.api_client import SecurityMonitorClient

class ROS2APIIntegration(Node):
    def __init__(self):
        super().__init__('ros2_api_integration')
        self.api_client = SecurityMonitorClient()
        self.timer = self.create_timer(5.0, self.check_threats)
    
    async def check_threats(self):
        async with self.api_client as client:
            threats = await client.get_current_threats()
            if threats:
                self.get_logger().warn(f"Active threats detected: {len(threats)}")
```

### Web Dashboard Integration

```javascript
// JavaScript example for web dashboard
class SecurityDashboard {
    constructor(apiUrl) {
        this.apiUrl = apiUrl;
        this.ws = null;
    }
    
    async connect() {
        this.ws = new WebSocket(`${this.apiUrl.replace('http', 'ws')}/ws/threats`);
        
        this.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            this.updateThreatDisplay(data.threats);
        };
    }
    
    async getSystemStatus() {
        const response = await fetch(`${this.apiUrl}/status`);
        return await response.json();
    }
    
    async emergencyStop() {
        const response = await fetch(`${this.apiUrl}/control`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${this.apiKey}`
            },
            body: JSON.stringify({
                command: 'emergency_stop',
                priority: 10
            })
        });
        return await response.json();
    }
}
```

## Performance Optimization

1. **Connection Pooling**: Reuse HTTP connections
2. **Async Operations**: Use async/await for non-blocking operations
3. **Caching**: Cache frequently accessed data
4. **Batch Operations**: Combine multiple requests when possible

## Troubleshooting

### Common Issues

1. **Connection Refused**: Ensure API server is running
2. **Authentication Failed**: Check API key validity
3. **Rate Limited**: Implement exponential backoff
4. **WebSocket Disconnected**: Implement reconnection logic

### Debug Mode

Enable debug logging:

```bash
ros2 launch security_monitor api_server.launch.py log_level:=debug
```

## Support

For API support and questions:
- Email: ayvaci.ilteber@gmail.com
- Documentation: See `/docs` directory
- Issues: Create GitHub issue for bugs or feature requests
