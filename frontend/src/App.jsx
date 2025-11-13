// frontend/src/App.jsx
import { useState } from 'react'; // ← add this import
import './index.css';
import ControlPanel from './components/ControlPanel';
import PersonTracker from './components/PersonTracker';
import TelemetryPanel from './components/TelemetryPanel';
import DistanceModeSelector from './components/DistanceModeSelector';
import useDroneWebSocket from './hooks/useDroneWebSocket';

function App() {
  const { isConnected, telemetry, detections, error, sendCommand } = useDroneWebSocket();
  const [followedId, setFollowedId] = useState(null); // ← track followed ID

  return (
    <div className="container">
      <div>
        <div className="panel">
          <h2 className="section-title">
            Flight Controls {isConnected ? '🟢' : '🔴'}
            {error && <span style={{ color: 'red', marginLeft: '8px' }}>{error}</span>}
          </h2>
          <ControlPanel sendCommand={sendCommand} />
        </div>

        <div className="panel" style={{ marginTop: '20px' }}>
          <h2 className="section-title">Detected Chairs</h2>
          <PersonTracker
            detections={detections}
            sendCommand={sendCommand}
            followedId={followedId}
            setFollowedId={setFollowedId}
          />
        </div>

        <div className="panel" style={{ marginTop: '20px' }}>
          <h2 className="section-title">Tracking Settings</h2>
          <DistanceModeSelector 
            sendCommand={sendCommand} 
            currentMode={telemetry.distance_mode}
          />
        </div>
      </div>

      <div className="panel">
        <h2 className="section-title">Live Telemetry</h2>
        <TelemetryPanel telemetry={telemetry} />
      </div>
    </div>
  );
}

export default App;