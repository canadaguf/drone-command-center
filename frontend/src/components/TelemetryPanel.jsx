export default function TelemetryPanel({ telemetry }) {
  // Show telemetry even without GPS (GPS might not be available indoors)
  // Just check if we have any telemetry data
  if (!telemetry || (!telemetry.mode && !telemetry.battery && !telemetry.tracking_status)) {
    return <p>Waiting for telemetry...</p>;
  }

  const getTrackingStatusColor = (status) => {
    switch (status) {
      case 'TRACKING': return '#2ecc71';
      case 'LOST': return '#f39c12';
      case 'HOVERING': return '#e67e22';
      case 'LANDING': return '#e74c3c';
      default: return '#95a5a6';
    }
  };

  const getBatteryColor = (battery) => {
    if (battery >= 50) return '#2ecc71';
    if (battery >= 20) return '#f39c12';
    return '#e74c3c';
  };

  return (
    <div style={{ lineHeight: '1.8' }}>
      {/* GPS and Position */}
      <div style={sectionStyle}>
        <h4 style={sectionTitleStyle}>Position</h4>
        <p><strong>GPS:</strong> {telemetry.gps?.lat?.toFixed(4) ?? '—'}, {telemetry.gps?.lon?.toFixed(4) ?? '—'}</p>
        {telemetry.gps?.satellites !== undefined && (
          <p><strong>GPS Satellites:</strong> {telemetry.gps.satellites}</p>
        )}
        <p><strong>Altitude:</strong> {telemetry.altitude !== null && telemetry.altitude !== undefined ? `${telemetry.altitude.toFixed(2)} m` : '—'}</p>
        <p><strong>Relative Alt:</strong> {telemetry.relative_alt !== null && telemetry.relative_alt !== undefined ? `${telemetry.relative_alt.toFixed(2)} m` : '—'}</p>
      </div>

      {/* Flight Status */}
      <div style={sectionStyle}>
        <h4 style={sectionTitleStyle}>Flight Status</h4>
        <p><strong>Mode:</strong> {telemetry.mode ?? '—'}</p>
        <p><strong>Armed:</strong> {telemetry.armed ? '🟢 YES' : '🔴 NO'}</p>
        <p><strong>Velocity:</strong> {telemetry.velocity !== null && telemetry.velocity !== undefined ? `${telemetry.velocity.toFixed(2)} m/s` : '—'}</p>
        <p><strong>Heading:</strong> {telemetry.heading !== null && telemetry.heading !== undefined ? `${telemetry.heading.toFixed(1)}°` : '—'}</p>
      </div>

      {/* Tracking Status */}
      <div style={sectionStyle}>
        <h4 style={sectionTitleStyle}>Tracking</h4>
        <p>
          <strong>Status:</strong> 
          <span style={{ 
            color: getTrackingStatusColor(telemetry.tracking_status),
            fontWeight: 'bold',
            marginLeft: '8px'
          }}>
            {telemetry.tracking_status ?? 'DISCONNECTED'}
          </span>
        </p>
        {telemetry.distance_mode && (
          <p>
            <strong>Distance Mode:</strong> 
            <span style={{ 
              fontWeight: 'bold',
              marginLeft: '8px',
              textTransform: 'capitalize'
            }}>
              {telemetry.distance_mode}
            </span>
          </p>
        )}
        {telemetry.rc_override_active && (
          <p style={{ color: '#e74c3c', fontWeight: 'bold' }}>
            ⚠️ MANUAL CONTROL ACTIVE
          </p>
        )}
      </div>

      {/* Sensors */}
      <div style={sectionStyle}>
        <h4 style={sectionTitleStyle}>Sensors</h4>
        <p><strong>TOF Forward:</strong> {telemetry.tof_forward ?? '—'} m</p>
        <p><strong>TOF Down:</strong> {telemetry.tof_down ?? '—'} m</p>
      </div>

      {/* Battery */}
      <div style={sectionStyle}>
        <h4 style={sectionTitleStyle}>Power</h4>
        <p>
          <strong>Battery:</strong> 
          <span style={{ 
            color: getBatteryColor(telemetry.battery),
            fontWeight: 'bold',
            marginLeft: '8px'
          }}>
            {telemetry.battery !== null && telemetry.battery !== undefined ? `${telemetry.battery.toFixed(1)}%` : '—'}
          </span>
        </p>
        {telemetry.battery_voltage !== null && telemetry.battery_voltage !== undefined && (
          <p><strong>Voltage:</strong> {telemetry.battery_voltage.toFixed(1)} V</p>
        )}
      </div>
    </div>
  );
}

const sectionStyle = {
  marginBottom: '16px',
  padding: '12px',
  border: '1px solid #eee',
  borderRadius: '6px',
  backgroundColor: '#fafafa'
};

const sectionTitleStyle = {
  margin: '0 0 8px 0',
  fontSize: '0.9rem',
  fontWeight: '600',
  color: '#2c3e50',
  borderBottom: '1px solid #ddd',
  paddingBottom: '4px'
};