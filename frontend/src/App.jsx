// frontend/src/App.jsx
import './index.css';
import ControlPanel from './components/ControlPanel';
import PersonTracker from './components/PersonTracker';
import TelemetryPanel from './components/TelemetryPanel';

function App() {
  return (
    <div className="container">
      <div>
        <div className="panel">
          <h2 className="section-title">Flight Controls</h2>
          <ControlPanel />
        </div>

        <div className="panel" style={{ marginTop: '20px' }}>
          <h2 className="section-title">Detected Persons</h2>
          <PersonTracker />
        </div>
      </div>

      <div className="panel">
        <h2 className="section-title">Live Telemetry</h2>
        <TelemetryPanel />
      </div>
    </div>
  );
}

export default App;