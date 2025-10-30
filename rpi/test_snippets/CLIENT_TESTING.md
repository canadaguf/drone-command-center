# Client Testing Guide

## Overview
This guide helps you test the drone client connection and verify commands work from the frontend.

## Quick Test Sequence

### 1. Test Individual Components

```bash
# Test MAVLink connection
python3 rpi/test_snippets/simple_heartbeat_test.py

# Test camera
python3 rpi/test_snippets/simple_camera_test.py

# Test client connections (MAVLink + WebSocket)
python3 rpi/test_snippets/test_client_connection.py
```

### 2. Start the Client

```bash
# Make sure you're in the right directory
cd /home/ilya/drone-command-center/rpi

# Activate venv
source /home/ilya/drone_client/venv/bin/activate

# Run the client
python3 -m drone_client.main
```

### 3. Verify Frontend Connection

1. Open your frontend URL (e.g., `https://your-frontend-url.com`)
2. Check if telemetry is updating (if client is sending telemetry)
3. Try the "Check Connection" button (if available)
4. Try the "Arm" button

## Configuration Check

Before starting, verify your configuration:

```bash
# Check config file exists
cat /home/ilya/drone_client/config/production.yaml

# Verify MAVLink settings
# Should show: connection: "/dev/ttyAMA0" (or your working port)
#              baud: 256000

# Verify backend URL
# Should show: ws_url: "wss://drone-command-center.onrender.com/ws?client=drone"
```

If config file doesn't exist, copy it:

```bash
cp /home/ilya/drone-command-center/rpi/drone_client/config/production.yaml \
   /home/ilya/drone_client/config/production.yaml
```

## Troubleshooting

### Client Won't Start

**Check logs:**
```bash
python3 -m drone_client.main 2>&1 | tee client.log
```

**Common issues:**
1. **MAVLink connection fails**
   - Check flight controller is powered on
   - Verify serial port in config matches working port
   - Test with: `python3 rpi/test_snippets/simple_heartbeat_test.py`

2. **WebSocket connection fails**
   - Check internet connection
   - Verify backend URL is correct
   - Check if backend is running

3. **Camera initialization fails**
   - Camera might not be critical for basic commands
   - Test with: `python3 rpi/test_snippets/simple_camera_test.py`

4. **Import errors**
   - Make sure venv is activated
   - Install dependencies: `pip install -r rpi/requirements.txt`

### Commands Don't Work from Frontend

**Check WebSocket messages:**
- Client should log: `Command received: arm`
- Check browser console for WebSocket messages
- Verify message format matches expected format

**Expected message format from frontend:**
```json
{
  "source": "web",
  "type": "command",
  "payload": {
    "action": "arm"
  }
}
```

**Expected response from drone:**
```json
{
  "source": "drone",
  "type": "arm_success",
  "payload": {
    "message": "Arm command executed"
  }
}
```

### Arm Command Not Working

1. **Check pre-arm checks:**
   - GPS lock (if required)
   - Battery level
   - Calibration status
   - Safety switches

2. **Check logs:**
   - Look for "Arm command received" in client logs
   - Look for MAVLink errors

3. **Test directly:**
   ```bash
   python3 rpi/test_snippets/worked_best/basic_arm.py
   ```

## Testing Command Flow

### Manual Test

1. Start client:
   ```bash
   python3 -m drone_client.main
   ```

2. Watch logs for:
   - "WebSocket connected"
   - "Heartbeat received from flight controller"
   - "Camera initialized successfully" (if camera works)

3. Open frontend and click "Arm"

4. Check logs for:
   - "Command received: arm"
   - MAVLink arm command execution
   - Response sent back

### Automated Test (Future)

You can create a test script that:
1. Connects to WebSocket
2. Sends arm command
3. Verifies response
4. Checks MAVLink state

## Next Steps

Once basic commands work:
1. Test all commands (arm, disarm, takeoff, land, freeze)
2. Test telemetry streaming
3. Test detection streaming (if YOLO model available)
4. Test tracking functionality

## See Also

- `test_client_connection.py` - Connection test script
- `simple_heartbeat_test.py` - MAVLink test
- `simple_camera_test.py` - Camera test
- `../README.md` - Main project documentation

