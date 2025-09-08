export interface ThreatAlert {
  id: string;
  type: string;
  severity: 'low' | 'medium' | 'high' | 'critical';
  confidence: number;
  location: {
    x: number;
    y: number;
    z: number;
  };
  timestamp: string;
  description?: string;
}

export interface SystemStatus {
  active_threats: number;
  system_health: Record<string, string>;
  last_update: string;
}

export interface ControlCommand {
  command: string;
  parameters?: Record<string, any>;
}
