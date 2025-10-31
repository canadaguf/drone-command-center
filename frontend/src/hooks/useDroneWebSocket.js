// frontend/src/hooks/useDroneWebSocket.js
import { useState, useEffect, useCallback } from 'react';

const RENDER_BACKEND_URL = 'wss://drone-command-center.onrender.com/ws';

export default function useDroneWebSocket() {
  const [ws, setWs] = useState(null);
  const [isConnected, setIsConnected] = useState(false);
  const [telemetry, setTelemetry] = useState({
    gps: { lat: null, lon: null },
    altitude: null,
    battery: null,
    mode: 'DISCONNECTED',
    velocity: null,
    distance_mode: null,  // Current tracking distance mode
  });
  const [detections, setDetections] = useState([]);
  const [error, setError] = useState(null);

  // Handle incoming messages
  const handleMessage = useCallback((event) => {
    try {
      const msg = JSON.parse(event.data);
      console.log('Received from backend:', msg);

      if (msg.source === 'drone') {
        if (msg.type === 'telemetry') {
          setTelemetry(msg.payload);
        } else if (msg.type === 'detections') {
          setDetections(msg.payload.persons || []);
        } else if (msg.type === 'follow_success' || msg.type === 'stop_following_success' || msg.type === 'set_distance_mode_success') {
          // Commands are acknowledged - UI updates happen optimistically
          console.log(`Command ${msg.type} acknowledged`);
          // For distance mode, we'll sync from telemetry, but log the success
          if (msg.type === 'set_distance_mode_success') {
            console.log('Distance mode changed successfully:', msg.payload);
          }
        } else if (msg.type === 'error') {
          console.error('Error from drone:', msg.payload);
          setError(msg.payload.message || 'Unknown error');
        }
      }
    } catch (e) {
      console.error('Invalid message:', event.data, e);
    }
  }, []);

  // Send command to drone
  const sendCommand = useCallback((action, payload = {}) => {
    if (!isConnected) {
      console.warn('Cannot send command: not connected');
      return;
    }

    const command = {
      source: 'web',
      type: 'command',
      payload: { action, ...payload },
    };

    console.log('Sending command:', command);
    ws.send(JSON.stringify(command));
  }, [isConnected, ws]);

  // Establish connection
  useEffect(() => {
    const socket = new WebSocket(RENDER_BACKEND_URL);

    socket.onopen = () => {
      console.log('WebSocket connected to backend');
      setIsConnected(true);
      setError(null);
    };

    socket.onmessage = handleMessage;

    socket.onerror = (err) => {
      console.error('WebSocket error:', err);
      setError('Connection error');
    };

    socket.onclose = () => {
      console.log('WebSocket disconnected. Reconnecting in 3s...');
      setIsConnected(false);
      // Auto-reconnect
      setTimeout(() => {
        window.location.reload(); // Force reconnect
      }, 3000);
    };

    setWs(socket);

    // Cleanup on unmount
    return () => {
      socket.close();
    };
  }, [handleMessage]); // Reconnect if handleMessage changes (unlikely)

  return {
    isConnected,
    telemetry,
    detections,
    error,
    sendCommand,
  };
}