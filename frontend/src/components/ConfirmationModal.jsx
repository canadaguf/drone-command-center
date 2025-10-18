// frontend/src/components/ConfirmationModal.jsx
// Inside ControlPanel.jsx

import ConfirmationModal from './ConfirmationModal';
import useDroneWebSocket from '../hooks/useDroneWebSocket';

export default function ControlPanel() {
  const { sendCommand } = useDroneWebSocket();

  const [showArmModal, setShowArmModal] = useState(false);
  const [showDisarmModal, setShowDisarmModal] = useState(false);

  // Handlers
  const handleArmConfirm = () => {
    sendCommand("arm");           // ← This sends: { source: "web", type: "command", payload: { action: "arm" } }
    setShowArmModal(false);
  };

  const handleDisarmConfirm = () => {
    sendCommand("disarm");        // ← Sends: { ... action: "disarm" }
    setShowDisarmModal(false);
  };

  return (
    <>
      {/* Arming Group */}
      <div style={groupStyle}>
        <h3 style={groupTitleStyle}>Arming</h3>
        <div style={buttonRowStyle}>
          <div style={buttonWithStatusStyle}>
            <button
              className="control-btn"
              onClick={() => setShowArmModal(true)}
              disabled={status.arm === 'sending'}
            >
              Arm
            </button>
            {getStatusIndicator('arm')}
          </div>
          <div style={buttonWithStatusStyle}>
            <button
              className="control-btn"
              onClick={() => setShowDisarmModal(true)}
              disabled={status.disarm === 'sending'}
            >
              Disarm
            </button>
            {getStatusIndicator('disarm')}
          </div>
        </div>
      </div>

      {/* Modals */}
      <ConfirmationModal
        isOpen={showArmModal}
        title="Confirm Arming"
        message="Are you sure you want to arm the drone? Propellers will spin!"
        onConfirm={handleArmConfirm}
        onCancel={() => setShowArmModal(false)}
      />

      <ConfirmationModal
        isOpen={showDisarmModal}
        title="Confirm Disarming"
        message="Are you sure you want to disarm the drone?"
        onConfirm={handleDisarmConfirm}
        onCancel={() => setShowDisarmModal(false)}
      />
    </>
  );
}

const styles = {
  overlay: {
    position: 'fixed',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    backgroundColor: 'rgba(0,0,0,0.5)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    zIndex: 1000,
  },
  modal: {
    backgroundColor: 'white',
    padding: '24px',
    borderRadius: '12px',
    width: '90%',
    maxWidth: '400px',
  },
  cancelBtn: {
    padding: '8px 16px',
    border: '1px solid #ccc',
    borderRadius: '6px',
    background: 'white',
    cursor: 'pointer',
  },
  confirmBtn: {
    padding: '8px 16px',
    border: 'none',
    borderRadius: '6px',
    background: '#e74c3c',
    color: 'white',
    cursor: 'pointer',
  }
};