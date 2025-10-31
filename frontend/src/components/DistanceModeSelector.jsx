// Distance mode selector component
import { useState, useEffect } from 'react';

export default function DistanceModeSelector({ sendCommand, currentMode = null }) {
  // Use currentMode from props (from telemetry) if available, otherwise default to 'medium'
  const [selectedMode, setSelectedMode] = useState(currentMode || 'medium');
  
  // Sync with backend state when telemetry updates
  useEffect(() => {
    if (currentMode) {
      setSelectedMode(currentMode);
    }
  }, [currentMode]);

  const distanceModes = [
    { value: 'close', label: 'Close (2m)', description: 'Indoor testing' },
    { value: 'medium', label: 'Medium (4m)', description: 'Balanced tracking' },
    { value: 'far', label: 'Far (6m)', description: 'Outdoor safety' }
  ];

  const handleModeChange = (mode) => {
    // Optimistically update UI
    setSelectedMode(mode);
    // Send command to backend
    sendCommand('set_distance_mode', { mode });
    console.log(`Distance mode changed to: ${mode}`);
  };

  return (
    <div style={styles.container}>
      <h3 style={styles.title}>Tracking Distance</h3>
      <div style={styles.modeGrid}>
        {distanceModes.map(mode => (
          <div
            key={mode.value}
            style={{
              ...styles.modeOption,
              ...(selectedMode === mode.value ? styles.selectedMode : {})
            }}
            onClick={() => handleModeChange(mode.value)}
          >
            <div style={styles.modeLabel}>{mode.label}</div>
            <div style={styles.modeDescription}>{mode.description}</div>
          </div>
        ))}
      </div>
    </div>
  );
}

const styles = {
  container: {
    marginBottom: '20px',
    padding: '16px',
    border: '1px solid #ddd',
    borderRadius: '8px',
    backgroundColor: '#f9f9f9'
  },
  title: {
    margin: '0 0 12px 0',
    fontSize: '1rem',
    fontWeight: '600',
    color: '#2c3e50'
  },
  modeGrid: {
    display: 'grid',
    gridTemplateColumns: 'repeat(auto-fit, minmax(120px, 1fr))',
    gap: '8px'
  },
  modeOption: {
    padding: '12px',
    border: '2px solid #ddd',
    borderRadius: '6px',
    cursor: 'pointer',
    textAlign: 'center',
    transition: 'all 0.2s ease',
    backgroundColor: 'white'
  },
  selectedMode: {
    borderColor: '#3498db',
    backgroundColor: '#e3f2fd',
    boxShadow: '0 2px 4px rgba(52, 152, 219, 0.2)'
  },
  modeLabel: {
    fontWeight: '600',
    fontSize: '0.9rem',
    color: '#2c3e50',
    marginBottom: '4px'
  },
  modeDescription: {
    fontSize: '0.8rem',
    color: '#7f8c8d'
  }
};
