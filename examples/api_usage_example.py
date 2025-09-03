#!/usr/bin/env python3

"""
Example usage of Security Monitor API
Demonstrates various API endpoints and WebSocket integration
"""

import asyncio
import aiohttp
import json
from datetime import datetime
from security_monitor.api_client import SecurityMonitorClient, WebSocketClient

class SecurityMonitorExample:
    def __init__(self, api_url="http://localhost:8000"):
        self.api_url = api_url
        self.client = SecurityMonitorClient(api_url)
        self.ws_client = WebSocketClient(api_url.replace('http', 'ws'))
    
    async def run_examples(self):
        """Run all API examples"""
        print("🚗 Security Monitor API Examples")
        print("=" * 40)
        
        # Example 1: Health Check
        await self.example_health_check()
        
        # Example 2: Get System Status
        await self.example_system_status()
        
        # Example 3: Get Current Threats
        await self.example_get_threats()
        
        # Example 4: Control Commands
        await self.example_control_commands()
        
        # Example 5: Analytics
        await self.example_analytics()
        
        # Example 6: WebSocket Integration
        await self.example_websocket()
    
    async def example_health_check(self):
        """Example: Health Check"""
        print("\n1. Health Check")
        print("-" * 20)
        
        async with self.client as client:
            health = await client.health_check()
            print(f"✅ API Health: {health['status']}")
            print(f"   Timestamp: {health['timestamp']}")
    
    async def example_system_status(self):
        """Example: System Status"""
        print("\n2. System Status")
        print("-" * 20)
        
        async with self.client as client:
            status = await client.get_system_status()
            print(f"✅ System Status: {status.status}")
            print(f"   Uptime: {status.uptime:.1f} seconds")
            print(f"   Active Threats: {status.active_threats}")
            print(f"   CPU Usage: {status.system_health.cpu_usage:.1f}%")
            print(f"   Memory Usage: {status.system_health.memory_usage:.1f}%")
    
    async def example_get_threats(self):
        """Example: Get Current Threats"""
        print("\n3. Current Threats")
        print("-" * 20)
        
        async with self.client as client:
            threats = await client.get_current_threats()
            print(f"✅ Found {len(threats)} active threats")
            
            for i, threat in enumerate(threats[:3]):  # Show first 3 threats
                print(f"   Threat {i+1}: {threat.threat_type} ({threat.severity})")
                print(f"   Confidence: {threat.confidence:.2f}")
                print(f"   Description: {threat.description}")
    
    async def example_control_commands(self):
        """Example: Control Commands"""
        print("\n4. Control Commands")
        print("-" * 20)
        
        async with self.client as client:
            # Example emergency stop (commented out for safety)
            print("⚠️  Emergency stop command (simulated)")
            print("   Command: emergency_stop")
            print("   Priority: 10")
            print("   Status: Would trigger emergency braking")
            
            # Example resume command
            print("\n✅ Resume command (simulated)")
            print("   Command: resume")
            print("   Priority: 1")
            print("   Status: Would resume normal operation")
    
    async def example_analytics(self):
        """Example: Analytics Summary"""
        print("\n5. Analytics Summary")
        print("-" * 20)
        
        async with self.client as client:
            analytics = await client.get_analytics_summary()
            print(f"✅ Analytics Summary")
            print(f"   Total Threats: {analytics.total_threats_detected}")
            print(f"   Detection Accuracy: {analytics.detection_accuracy:.2f}")
            print(f"   False Positive Rate: {analytics.false_positive_rate:.2f}")
            print(f"   Threat Types: {analytics.threat_types}")
    
    async def example_websocket(self):
        """Example: WebSocket Integration"""
        print("\n6. WebSocket Integration")
        print("-" * 20)
        
        print("🔌 Connecting to WebSocket...")
        
        try:
            await self.ws_client.connect()
            print("✅ WebSocket connected")
            
            # Listen for 5 seconds
            print("👂 Listening for threat updates (5 seconds)...")
            
            async def handle_threat_update(data):
                timestamp = data.get('timestamp', 'Unknown')
                threats = data.get('threats', [])
                print(f"   📡 Update at {timestamp}: {len(threats)} threats")
            
            # Create a task that will timeout after 5 seconds
            listen_task = asyncio.create_task(
                self.ws_client.listen_for_threats(handle_threat_update)
            )
            
            # Wait for 5 seconds then cancel
            await asyncio.sleep(5)
            listen_task.cancel()
            
            print("✅ WebSocket example completed")
            
        except Exception as e:
            print(f"❌ WebSocket error: {e}")
        finally:
            await self.ws_client.close()

async def main():
    """Main function"""
    example = SecurityMonitorExample()
    
    try:
        await example.run_examples()
        print("\n🎉 All examples completed successfully!")
        
    except Exception as e:
        print(f"\n❌ Error running examples: {e}")
        print("💡 Make sure the API server is running:")
        print("   ros2 launch security_monitor api_server.launch.py")

if __name__ == "__main__":
    asyncio.run(main())
