// frontend/src/components/ControlPanel.jsx
import React, { useState, useEffect } from 'react';
import ConfirmationModal from './ConfirmationModal';
import useDroneWebSocket from '../hooks/useDroneWebSocket';

const initialStatus = {
  connection: 'idle',
  prearm: 'idle',
  arm: 'idle',
  disarm: 'idle',
  liftoff: 'idle',
  landing: 'idle',
  loiter: 'idle',
};

export default function ControlPanel() {
  // Get sendCommand from hook — NOT from props
  const { sendCommand } = useDroneWebSocket();
  const [status, setStatus] = useState(initialStatus);
  const [showArmModal, setShowArmModal] = useState(false);
  const [showDisarmModal, setShowDisarmModal] = useState(false);

  // Map command actions to status keys
  const commandToKey = {
    'arm': 'arm',
    'disarm': 'disarm',
    'takeoff': 'liftoff',
    'land': 'landing',
    'freeze': 'loiter',
    'prearm_checks': 'prearm',
    'check_connection': 'connection'
  };

  // Listen for command responses
  useEffect(() => {
    const handleCommandResponse = (event) => {
      const { type, payload } = event.detail;
      const success = type.endsWith('_success');
      const action = type.replace('_success', '').replace('_error', '');
      const key = commandToKey[action];
      
      if (key) {
        if (success) {
          setStatus(prev => ({ ...prev, [key]: 'success' }));
          // Reset to idle after 2 seconds
          setTimeout(() => {
            setStatus(prev => ({ ...prev, [key]: 'idle' }));
          }, 2000);
        } else {
          setStatus(prev => ({ ...prev, [key]: 'error' }));
          // Reset to idle after 3 seconds on error
          setTimeout(() => {
            setStatus(prev => ({ ...prev, [key]: 'idle' }));
          }, 3000);
          console.error(`Command ${action} failed:`, payload.message || 'Unknown error');
        }
      }
    };

    window.addEventListener('droneCommandResponse', handleCommandResponse);
    return () => {
      window.removeEventListener('droneCommandResponse', handleCommandResponse);
    };
  }, []);

  const handleCommand = (action, key) => {
    // Update UI status
    setStatus(prev => ({ ...prev, [key]: 'sending' }));

    // Send real command via WebSocket
    sendCommand(action);
  };

  const getStatusIndicator = (key) => {
    const s = status[key];
    if (s === 'sending') return <span style={{ color: '#f39c12' }}>🟡</span>;
    if (s === 'success') return <span style={{ color: '#2ecc71' }}>🟢</span>;
    if (s === 'error') return <span style={{ color: '#e74c3c' }}>🔴</span>;
    return <span style={{ color: '#95a5a6' }}>⚪</span>;
  };

  return (
    <div>
      {/* Checks Group */}
      <div style={groupStyle}>
        <h3 style={groupTitleStyle}>Проверки</h3>
        <div style={buttonRowStyle}>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => handleCommand('check_connection', 'connection')}>
              Проверка подключения дрона
            </button>
            {getStatusIndicator('connection')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => handleCommand('prearm_checks', 'prearm')}>
              Предпусковые проверки
            </button>
            {getStatusIndicator('prearm')}
          </div>
        </div>
      </div>

      {/* Arming Group */}
      <div style={groupStyle}>
        <h3 style={groupTitleStyle}>Запуск</h3>
        <div style={buttonRowStyle}>
          <div style={buttonWithStatusStyle}>
            <button
              className="control-btn"
              onClick={() => setShowArmModal(true)}
              disabled={status.arm === 'sending'}
            >
              ARM
            </button>
            {getStatusIndicator('arm')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button
              className="control-btn"
              onClick={() => setShowDisarmModal(true)}
              disabled={status.disarm === 'sending'}
            >
              DISARM
            </button>
            {getStatusIndicator('disarm')}
          </div>
        </div>
      </div>

      {/* Flight Modes Group */}
      <div style={groupStyle}>
        <h3 style={groupTitleStyle}>Режимы полёта</h3>
        <div style={buttonRowStyle}>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => handleCommand('takeoff', 'liftoff')}>
              Взлёт
            </button>
            {getStatusIndicator('liftoff')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => handleCommand('land', 'landing')}>
              Посадка
            </button>
            {getStatusIndicator('landing')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button className="control-btn" onClick={() => handleCommand('freeze', 'loiter')}>
              Зависание
            </button>
            {getStatusIndicator('loiter')}
          </div>
        </div>
      </div>

      {/* Modals */}
      <ConfirmationModal
        isOpen={showArmModal}
        title="Запуск?"
        message="Запуск дрона активирует моторы. Убедитесь, что пропеллеры закреплены и дрон находится на ровной поверхности."
        onConfirm={() => {
          handleCommand('arm', 'arm');
          setShowArmModal(false);
        }}
        onCancel={() => setShowArmModal(false)}
      />

      <ConfirmationModal
        isOpen={showDisarmModal}
        title="Выключение?"
        message="При выключении дрона, питание к моторам перестанет поступать. Убедитесь, что дрон приземлился."
        onConfirm={() => {
          handleCommand('disarm', 'disarm');
          setShowDisarmModal(false);
        }}
        onCancel={() => setShowDisarmModal(false)}
      />
    </div>
  );
}

// Styles (unchanged)
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