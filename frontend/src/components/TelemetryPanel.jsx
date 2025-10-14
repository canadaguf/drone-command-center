export default function TelemetryPanel({ telemetry }) {
  if (!telemetry.gps?.lat) {
    return <p>Waiting for telemetry...</p>;
  }

  return (
    <div style={{ lineHeight: '1.8' }}>
      <p><strong>GPS:</strong> {telemetry.gps.lat?.toFixed(4)}, {telemetry.gps.lon?.toFixed(4)}</p>
      <p><strong>Altitude:</strong> {telemetry.altitude ?? '—'} m</p>
      <p><strong>Battery:</strong> {telemetry.battery ?? '—'}%</p>
      <p><strong>Flight Mode:</strong> {telemetry.mode ?? '—'}</p>
      <p><strong>Velocity:</strong> {telemetry.velocity ?? '—'} m/s</p>
    </div>
  );
}