// frontend/src/components/ControlPanel.jsx
import { useState } from 'react';
import ConfirmationModal from './ConfirmationModal';

// Status: 'idle' | 'sending' | 'success' | 'error'
const initialStatus = {
  connection: 'idle',
  prearm: 'idle',
  arm: 'idle',
  disarm: 'idle',
  liftoff: 'idle',
  landing: 'idle',
  freeze: 'idle',
};

export default function ControlPanel() {
  const [status, setStatus] = useState(initialStatus);
  const [showArmModal, setShowArmModal] = useState(false);
  const [showDisarmModal, setShowDisarmModal] = useState(false);

  // Simulate sending command (replace later with WebSocket)
  const sendCommand = async (action, key) => {
    setStatus(prev => ({ ...prev, [key]: 'sending' }));
    try {
      console.log('Sending command:', action);
      // TODO: await websocket.send(...)
      await new Promise(resolve => setTimeout(resolve, 300)); // simulate delay
      setStatus(prev => ({ ...prev, [key]: 'success' }));
    } catch (err) {
      setStatus(prev => ({ ...prev, [key]: 'error' }));
    }
    // Auto-reset status after 2s
    setTimeout(() => {
      setStatus(prev => ({ ...prev, [key]: 'idle' }));
    }, 2000);
  };

  const getStatusIndicator = (key) => {
    const s = status[key];
    if (s === 'sending') return <span style={{ color: '#f39c12' }}>🟡</span>; // yellow = pending
    if (s === 'success') return <span style={{ color: '#2ecc71' }}>🟢</span>;
    if (s === 'error') return <span style={{ color: '#e74c3c' }}>🔴</span>;
    return <span style={{ color: '#95a5a6' }}>⚪</span>; // gray = idle
  };

  return (
    <div>
      {/* Checks Group */}
      <div style={groupStyle}>
        <h3 style={groupTitleStyle}>Checks</h3>
        <div style={buttonRowStyle}>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => sendCommand('check_connection', 'connection')}>
              Check Drone Connection
            </button>
            {getStatusIndicator('connection')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => sendCommand('prearm_checks', 'prearm')}>
              Pre-Arm Checks
            </button>
            {getStatusIndicator('prearm')}
          </div>
        </div>
      </div>

      {/* Arming Group */}
      <div style={groupStyle}>
        <h3 style={groupTitleStyle}>Arming</h3>
        <div style={buttonRowStyle}>
          <div style={buttonWithStatusStyle}>
            <button
              className="control-btn" onClick={() => setShowArmModal(true)}
              disabled={status.arm === 'sending'}
            >
              Arm
            </button>
            {getStatusIndicator('arm')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button
              className="control-btn" onClick={() => setShowDisarmModal(true)}
              disabled={status.disarm === 'sending'}
            >
              Disarm
            </button>
            {getStatusIndicator('disarm')}
          </div>
        </div>
      </div>

      {/* Flight Modes Group */}
      <div style={groupStyle}>
        <h3 style={groupTitleStyle}>Flight Modes</h3>
        <div style={buttonRowStyle}>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => sendCommand('takeoff', 'liftoff')}>
              Liftoff
            </button>
            {getStatusIndicator('liftoff')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => sendCommand('land', 'landing')}>
              Landing
            </button>
            {getStatusIndicator('landing')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => sendCommand('freeze', 'freeze')}>
              Freeze
            </button>
            {getStatusIndicator('freeze')}
          </div>
        </div>
      </div>

      {/* Modals */}
      <ConfirmationModal
        isOpen={showArmModal}
        title="Arm Drone?"
        message="Arming will enable motors. Ensure propellers are clear and drone is on level ground."
        onConfirm={() => {
          sendCommand('arm', 'arm');
          setShowArmModal(false);
        }}
        onCancel={() => setShowArmModal(false)}
      />

      <ConfirmationModal
        isOpen={showDisarmModal}
        title="Disarm Drone?"
        message="Disarming will cut power to motors. Only do this when drone is landed and stable."
        onConfirm={() => {
          sendCommand('disarm', 'disarm');
          setShowDisarmModal(false);
        }}
        onCancel={() => setShowDisarmModal(false)}
      />
    </div>
  );
}

// Styles
const groupStyle = {
  marginBottom: '20px',
  paddingBottom: '16px',
  borderBottom: '1px solid #eee',
};

const groupTitleStyle = {
  fontSize: '1rem',
  fontWeight: '600',
  marginBottom: '12px',
  color: '#2c3e50',
};

const buttonRowStyle = {
  display: 'flex',
  gap: '12px',
  flexWrap: 'wrap',
  alignItems: 'center',
};

const buttonWithStatusStyle = {
  display: 'flex',
  alignItems: 'center',
  gap: '8px',
};

// Reuse button styles from index.css