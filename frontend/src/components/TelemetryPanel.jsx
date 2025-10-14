// frontend/src/components/TelemetryPanel.jsx
export default function TelemetryPanel() {
  // Later: real data from drone
  const telemetry = {
    gps: { lat: 55.7512, lon: 37.6184 },
    altitude: 12.4,
    battery: 92,
    mode: 'LOITER',
    velocity: 0.3
  };

  return (
    <div style={{ lineHeight: '1.8' }}>
      <p><strong>GPS:</strong> {telemetry.gps.lat.toFixed(4)}, {telemetry.gps.lon.toFixed(4)}</p>
      <p><strong>Altitude:</strong> {telemetry.altitude} m</p>
      <p><strong>Battery:</strong> {telemetry.battery}%</p>
      <p><strong>Flight Mode:</strong> {telemetry.mode}</p>
      <p><strong>Velocity:</strong> {telemetry.velocity} m/s</p>
    </div>
  );
}