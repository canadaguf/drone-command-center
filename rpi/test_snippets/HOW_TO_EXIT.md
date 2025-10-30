# How to Exit the Drone Client Process

## Quick Answer

Press **Ctrl+C** in the terminal where the process is running.

The client is set up to handle `SIGINT` (Ctrl+C) gracefully and will:
1. Stop all loops
2. Disconnect from WebSocket
3. Disconnect from MAVLink
4. Cleanup camera and other resources
5. Exit cleanly

## Detailed Explanation

### Graceful Shutdown

The drone client has signal handlers set up for:
- **SIGINT** (Ctrl+C) - Keyboard interrupt
- **SIGTERM** - Termination signal (used by systemd)

When you press Ctrl+C:
1. Signal handler sets `self.running = False`
2. All async loops check this flag and exit
3. `shutdown()` method is called to cleanup resources
4. Process exits

### If Ctrl+C Doesn't Work

If Ctrl+C doesn't work (rare, but can happen):

1. **Try Ctrl+C twice** - Sometimes first one is caught, second forces exit

2. **Find and kill the process:**
   ```bash
   # Find the process
   ps aux | grep drone_client
   
   # Kill it (replace PID with actual process ID)
   kill <PID>
   
   # Or force kill if needed
   kill -9 <PID>
   ```

3. **Kill by name:**
   ```bash
   pkill -f "drone_client.main"
   ```

### Running in Background

If you want to run in background and still be able to stop it:

```bash
# Run in background
nohup python3 -m drone_client.main > drone_client.log 2>&1 &

# Find the process ID
ps aux | grep drone_client

# Stop it later
kill <PID>
```

Or use systemd service (if installed):
```bash
sudo systemctl stop drone-client
```

## Warning Signs Before Exiting

Before exiting, make sure:
- ✅ Drone is disarmed (if it was armed)
- ✅ All loops have stopped
- ✅ Resources are cleaned up

The shutdown() method handles this automatically, but if you're concerned, manually disarm first:
1. Use frontend to disarm
2. Wait for confirmation
3. Then press Ctrl+C

