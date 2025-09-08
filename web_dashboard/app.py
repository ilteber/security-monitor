#!/usr/bin/env python3
"""
Security Monitor Web Dashboard
Real-time web interface for threat monitoring
"""

from flask import Flask, render_template, jsonify, request
import requests
import json
import time
from datetime import datetime
import threading

app = Flask(__name__)

# API configuration
API_BASE_URL = "http://localhost:8000"

class ThreatMonitor:
    def __init__(self):
        self.threats = []
        self.system_status = {}
        self.last_update = None
        
    def fetch_data(self):
        """Fetch data from API server"""
        try:
            # Get current threats
            response = requests.get(f"{API_BASE_URL}/threats", timeout=2)
            if response.status_code == 200:
                self.threats = response.json()
            
            # Get system status
            response = requests.get(f"{API_BASE_URL}/status", timeout=2)
            if response.status_code == 200:
                self.system_status = response.json()
                
            self.last_update = datetime.now()
        except Exception as e:
            print(f"Error fetching data: {e}")

# Global monitor instance
monitor = ThreatMonitor()

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('dashboard.html')

@app.route('/api/threats')
def get_threats():
    """Get current threats"""
    monitor.fetch_data()
    return jsonify(monitor.threats)

@app.route('/api/status')
def get_status():
    """Get system status"""
    monitor.fetch_data()
    return jsonify(monitor.system_status)

@app.route('/api/control', methods=['POST'])
def control_command():
    """Send control commands"""
    data = request.get_json()
    try:
        response = requests.post(f"{API_BASE_URL}/control", json=data, timeout=5)
        return jsonify(response.json())
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/api/analytics')
def get_analytics():
    """Get analytics data"""
    try:
        response = requests.get(f"{API_BASE_URL}/analytics/summary", timeout=2)
        return jsonify(response.json())
    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    print("🌐 Starting Security Monitor Web Dashboard...")
    print("📊 Dashboard: http://localhost:5000")
    print("🔗 API Server: http://localhost:8000")
    app.run(host='0.0.0.0', port=5000, debug=True)
