# Serial Port Configuration for Raspberry Pi OS Lite

## Overview
This guide helps you configure the serial port (`/dev/ttyAMA0`) for MAVLink communication with your drone's flight controller on Raspberry Pi OS Lite.

## Quick Setup Steps

### 1. Disable Serial Console (CRITICAL)

The serial console must be disabled for MAVLink to work. On Raspberry OS Lite, this is usually enabled by default.

**Option A: Using raspi-config (Recommended)**
```bash
sudo raspi-config
```
- Navigate to: **Interface Options** → **Serial Port**
- Disable: **"Serial login shell"** (select No)
- Enable: **"Serial interface"** (select Yes)
- Exit and reboot: `sudo reboot`

**Option B: Manual Configuration**
```bash
# Edit config.txt
sudo nano /boot/config.txt

# Ensure these lines exist:
enable_uart=1

# Remove or comment out any line like:
# console=serial0,115200
# console=ttyAMA0,115200

# Disable serial console service
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl stop serial-getty@ttyAMA0.service

# Reboot
sudo reboot
```

### 2. Add User to dialout Group

You need to be in the `dialout` group to access serial ports:

```bash
sudo usermod -a -G dialout $USER
```

**Important:** Logout and login again (or reboot) for this to take effect.

Verify with:
```bash
groups
# Should see 'dialout' in the list
```

### 3. Verify Serial Port

After reboot, check if the serial port is available:

```bash
ls -l /dev/ttyAMA0
# Should show: crw-rw---- 1 root dialout ...

# Check if port is accessible
test -r /dev/ttyAMA0 && test -w /dev/ttyAMA0 && echo "Port accessible" || echo "Port NOT accessible"
```

### 4. Test Connection

Run the test script:

```bash
cd /path/to/drone-command-center/rpi/test_snippets
python3 simple_heartbeat_test.py
# or
python3 test_connection.py
```

## Troubleshooting

### Port Not Found
- **Symptom:** `FileNotFoundError: [Errno 2] No such file or directory: '/dev/ttyAMA0'`
- **Solution:** 
  - Check `/boot/config.txt` has `enable_uart=1`
  - Verify serial interface is enabled in raspi-config
  - Reboot after changes

### Permission Denied
- **Symptom:** `PermissionError: [Errno 13] Permission denied: '/dev/ttyAMA0'`
- **Solution:**
  ```bash
  sudo usermod -a -G dialout $USER
  # Logout and login again, or reboot
  ```

### No Heartbeat Received
- **Symptom:** Script waits but never receives heartbeat
- **Check:**
  1. Flight controller is powered on
  2. Physical connections:
     - TX on Pi → RX on FC
     - RX on Pi → TX on FC
     - GND connected
  3. Correct baud rate (try both 115200 and 256000)
  4. Serial console is disabled (check with: `dmesg | grep tty`)
  5. No other process is using the port: `sudo lsof /dev/ttyAMA0`

### Serial Console Still Active
- **Check:** `dmesg | grep tty`
- **If you see:** `serial0: ttyAMA0 at MMIO...` with console messages
- **Fix:** Disable serial console as described in step 1

## Common Baud Rates

Different flight controllers use different baud rates:
- **115200** - Common default, slower but more reliable
- **256000** - Higher speed, used by some newer setups
- **57600** - Older setups

Test scripts try both 115200 and 256000 automatically.

## Alternative Serial Ports

On some Raspberry Pi models, the serial port might be:
- `/dev/serial0` (symlink to actual port)
- `/dev/ttyS0` (on newer Pi models)

The test scripts check `/dev/ttyAMA0` and `/dev/serial0` automatically.

## Verification Commands

```bash
# Check serial port exists
ls -l /dev/ttyAMA0 /dev/serial0

# Check permissions
groups | grep dialout

# Check if serial console is disabled
dmesg | grep -i serial

# Check config.txt settings
grep -E "enable_uart|console=" /boot/config.txt

# Check if port is in use
sudo lsof /dev/ttyAMA0
```

## Next Steps

Once heartbeat is working:
1. Test with `simple_heartbeat_test.py` - Basic connection
2. Test with `test_connection.py` - Comprehensive diagnostics
3. Test with `basic_config_check.py` - Full system info
4. Run full system: `python3 -m drone_client.main`


