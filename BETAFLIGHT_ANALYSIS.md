# Betaflight Migration Analysis

## Key Findings from Betaflight Configuration

### Protocol Differences
- **Current Project**: Uses MAVLink protocol (ArduPilot)
- **Target Project**: Uses MSP (MultiWii Serial Protocol) with Betaflight
- **Communication**: Both use serial RX/TX, but different protocols

### Betaflight Serial Configuration
From the dump:
- `serial 0 2048 115200` - MSP enabled (2048 = MSP flag)
- `serial 4 1 500000` - MSP at 500000 baud
- `serial 5 1 500000` - MSP at 500000 baud
- `msp_override_channels_mask = 47` - Channels 0-5 can be overridden via MSP
- `msp_override_failsafe = ON` - Failsafe enabled for MSP override

### Device Info
- Project: "Lupynis"
- Control Type: MSP
- Hardware: QUAD with OV5647 camera
- Software settings include tracking parameters

## Files to Look For in SD-Card Image

### 1. Python MSP Communication Library
**Location**: `/home/pi/` or `/opt/` or project directories
**Files to find**:
- `msp.py` or `msp_*.py` - MSP protocol implementation
- `betaflight.py` or `betaflight_*.py` - Betaflight-specific wrapper
- `multiwii.py` - MultiWii protocol (MSP predecessor)
- `flight_controller.py` - Main flight controller interface
- `controller.py` - Controller implementation

**What to look for**:
- MSP message encoding/decoding
- Serial communication setup
- RC channel override implementation
- Telemetry reading

### 2. Main Application Code
**Location**: `/home/pi/` or project root
**Files to find**:
- `main.py` or `drone.py` or `lupynis.py` - Main application entry point
- `client.py` - Client implementation
- Any file matching project name "Lupynis" or "B1"

**What to look for**:
- How MSP connection is established
- How RC commands are sent
- How telemetry is read
- Integration with vision/tracking

### 3. Configuration Files
**Location**: `/home/pi/` or `/etc/` or project config directory
**Files to find**:
- `config.json` or `config.yaml` or `config.ini`
- `device_info.json` (already found)
- `betaflight_config.json`
- `serial_config.json`

**What to look for**:
- Serial port path (likely `/dev/ttyAMA0` or `/dev/ttyAMA10`)
- Baud rate (likely 115200 or 500000 based on Betaflight config)
- MSP channel mappings
- RC channel ranges

### 4. Dependencies/Requirements
**Location**: Project root or `/home/pi/`
**Files to find**:
- `requirements.txt`
- `setup.py` or `pyproject.toml`
- `Pipfile` or `Pipfile.lock`

**What to look for**:
- MSP-related packages: `pymsp`, `multiwii`, `betaflight-msp`, `msp-python`
- Serial communication: `pyserial`
- Any custom MSP libraries

### 5. Vision/Tracking Code
**Location**: `/home/pi/` or project directories
**Files to find**:
- `tracking.py` or `tracker.py`
- `vision.py` or `detection.py`
- `person_tracker.py`
- `yolo_*.py` or `detector.py`

**What to look for**:
- How tracking commands are converted to RC values
- How RC override is implemented
- Integration with MSP controller

### 6. MSP Message Examples
**Location**: Test directories or `/home/pi/`
**Files to find**:
- `test_msp.py` or `test_betaflight.py`
- `examples/` directory
- `scripts/` directory with MSP-related scripts

**What to look for**:
- MSP message structure
- How to send RC override commands
- How to read telemetry
- Connection examples

### 7. System Service Files
**Location**: `/etc/systemd/system/` or `/home/pi/.config/systemd/user/`
**Files to find**:
- `*.service` files (e.g., `lupynis.service`, `drone.service`)
- Service configuration showing startup command

**What to look for**:
- Main application entry point
- Command-line arguments
- Environment variables

### 8. Documentation
**Location**: Project root or `/home/pi/`
**Files to find**:
- `README.md`
- `MSP.md` or `BETAFLIGHT.md`
- `SETUP.md` or `INSTALL.md`
- Any documentation about MSP communication

## Critical Information Needed

### MSP Communication Details
1. **MSP Library**: Which Python library is used for MSP?
   - Common options: `pymsp`, `multiwii`, custom implementation
   
2. **RC Override**: How are RC channels overridden?
   - MSP command: `MSP_SET_RAW_RC` (command ID 200)
   - Channel values: 1000-2000 range (same as ArduPilot)
   
3. **Telemetry**: How is telemetry read?
   - MSP commands: `MSP_ATTITUDE`, `MSP_ALTITUDE`, `MSP_RAW_IMU`, etc.
   
4. **Connection**: Serial port and baud rate
   - Based on config: likely `/dev/ttyAMA0` at 115200 or 500000 baud

### Key MSP Commands for Betaflight
- `MSP_SET_RAW_RC` (200) - Set RC channel values (for control)
- `MSP_ATTITUDE` (108) - Get attitude (roll, pitch, yaw)
- `MSP_ALTITUDE` (109) - Get altitude
- `MSP_RAW_IMU` (102) - Get IMU data
- `MSP_STATUS` (101) - Get flight status
- `MSP_ANALOG` (110) - Get battery voltage/current

## Next Steps After Finding Files

1. **Identify MSP Library**: Find which library/implementation is used
2. **Map Commands**: Map current MAVLink commands to MSP equivalents
3. **Create BetaflightController**: Replace MAVLinkController with MSP-based controller
4. **Test Compatibility**: Ensure RC override and telemetry work correctly

## Application Structure from app.log

Based on the log file, the application has these components:
- **Engine** - Main engine/coordinator
- **Main** - Main application entry
- **Autopilot** - Autopilot control module
- **MSP_RC** - MSP RC control module (CRITICAL - this is the MSP communication!)
- **Tracker** - Tracking module

These appear to be module names, so look for:
- `Engine.py` or `engine.py` or `Engine/` directory
- `Main.py` or `main.py`
- `Autopilot.py` or `autopilot.py` or `Autopilot/` directory
- `MSP_RC.py` or `msp_rc.py` or `MSP_RC/` directory (THIS IS KEY!)
- `Tracker.py` or `tracker.py` or `Tracker/` directory

## Search Commands for SD-Card Image

### 1. Find Application Modules (Based on Log)
```bash
# Search for module names from app.log
find /mnt/sdcard -iname "*engine*" -o -iname "*main*" -o -iname "*autopilot*" -o -iname "*msp*" -o -iname "*tracker*"

# Specifically look for MSP_RC module (most important!)
find /mnt/sdcard -iname "*msp_rc*" -o -iname "*MSP_RC*" -o -iname "*msp*rc*"

# Find directories matching module names
find /mnt/sdcard -type d -iname "*engine*" -o -type d -iname "*autopilot*" -o -type d -iname "*msp*" -o -type d -iname "*tracker*"
```

### 2. Find Compiled Python Files (.pyc)
```bash
# Find all .pyc files (compiled Python)
find /mnt/sdcard -name "*.pyc" -type f

# Find __pycache__ directories
find /mnt/sdcard -type d -name "__pycache__"

# Look inside __pycache__ for module names
find /mnt/sdcard -type d -name "__pycache__" -exec ls -la {} \;
```

### 3. Find Python Packages/Libraries
```bash
# Look in site-packages (installed packages)
find /mnt/sdcard -path "*/site-packages/*" -name "*msp*" -o -path "*/site-packages/*" -name "*betaflight*"

# Look for .egg-info or .dist-info directories
find /mnt/sdcard -type d -name "*.egg-info" -o -name "*.dist-info"

# Find installed packages that might be custom
find /mnt/sdcard -path "*/site-packages/*" -type d | grep -E "(msp|betaflight|multiwii|lupynis)"
```

### 4. Find Executables and Entry Points
```bash
# Find executable Python scripts
find /mnt/sdcard -name "*.py" -type f -executable

# Find scripts that might be entry points
find /mnt/sdcard -path "*/bin/*" -type f

# Find files that import the modules from log
find /mnt/sdcard -name "*.py" -type f | xargs grep -l "Engine\|MSP_RC\|Autopilot\|Tracker" 2>/dev/null
```

### 5. Find Systemd Service (Shows Entry Point)
```bash
# Find systemd service files
find /mnt/sdcard -path "*/systemd/system/*.service" -o -path "*/.config/systemd/user/*.service"

# Look for service files with "lupynis", "drone", "b1" in name
find /mnt/sdcard -name "*.service" -type f | xargs grep -l "python\|python3" 2>/dev/null
```

### 6. Find Configuration and Log Files
```bash
# Find app.log (already found) - check its directory
find /mnt/sdcard -name "app.log" -exec dirname {} \;

# Find all .log files
find /mnt/sdcard -name "*.log" -type f

# Find config files near app.log location
# (Check the directory where app.log was found)
```

### 7. Search for MSP-Specific Code
```bash
# Search for MSP command IDs (200 = MSP_SET_RAW_RC)
find /mnt/sdcard -name "*.py" -type f | xargs grep -l "200\|MSP_SET_RAW_RC\|msp.*200" 2>/dev/null

# Search for MSP message structure
find /mnt/sdcard -name "*.py" -type f | xargs grep -l "MSP_ATTITUDE\|MSP_ALTITUDE\|MSP_STATUS" 2>/dev/null

# Search for serial communication with MSP
find /mnt/sdcard -name "*.py" -type f | xargs grep -l "serial.*msp\|msp.*serial" 2>/dev/null
```

### 8. Check Common Installation Locations
```bash
# Check /opt/ (common for installed applications)
ls -la /mnt/sdcard/opt/

# Check /usr/local/ (local installations)
ls -la /mnt/sdcard/usr/local/

# Check /home/pi/ (user directory)
ls -la /mnt/sdcard/home/pi/

# Check for hidden directories
find /mnt/sdcard/home/pi -name ".*" -type d
```

### 9. Find Where app.log is Located
```bash
# Find app.log and check its directory structure
find /mnt/sdcard -name "app.log" -exec ls -la {} \;
find /mnt/sdcard -name "app.log" -exec dirname {} \;

# List all files in the same directory as app.log
# (Replace /path/to/app.log with actual path)
find /mnt/sdcard -name "app.log" -exec sh -c 'ls -la "$(dirname {})"' \;
```

## CRITICAL DISCOVERY: Application Location

Based on `deploy.sh` and `zram.sh` scripts:

### Application Structure
- **Main Entry Point**: `/media/src/main_rpi.py`
- **Virtual Environment**: `/home/pi/env/bin/python`
- **Storage**: Application runs from **zram** (compressed RAM disk) mounted at `/media`
- **Setup Scripts**: Located in `/home/pi/decrypt/`

### Key Files to Find
1. **`/media/src/main_rpi.py`** - Main application entry point
2. **`/media/src/`** directory - Contains all source code
3. **Decryption/Extraction Process** - The `dcd` command in deploy.sh likely decrypts/decompresses source files to `/media/src/`

### What This Means
The application source code is likely:
- **Encrypted/compressed** on the SD-card
- **Extracted to `/media/src/`** at runtime via `dcd` command
- **Stored in RAM** (zram filesystem) during execution
- **Not directly visible** in the SD-card image as plain Python files

### Where to Look in SD-Card Image

1. **Look for encrypted/compressed source files**:
   ```bash
   # Find dcd executable or script
   find /mnt/sdcard -name "dcd" -o -name "*dcd*"
   
   # Find compressed/encrypted archives
   find /mnt/sdcard -name "*.tar.gz" -o -name "*.zip" -o -name "*.enc" -o -name "*.crypt"
   
   # Look in /home/pi/decrypt/ directory
   ls -la /mnt/sdcard/home/pi/decrypt/
   ```

2. **Check for source extraction location**:
   ```bash
   # Look for src directory anywhere
   find /mnt/sdcard -type d -name "src"
   
   # Look for main_rpi.py
   find /mnt/sdcard -name "main_rpi.py"
   ```

3. **Check deploy scripts location**:
   ```bash
   # Find deploy.sh
   find /mnt/sdcard -name "deploy.sh"
   
   # Check /home/pi/decrypt/ directory contents
   ls -la /mnt/sdcard/home/pi/decrypt/
   ```

4. **Look for encrypted/compressed source**:
   ```bash
   # Check for large binary files that might be encrypted source
   find /mnt/sdcard -type f -size +1M | grep -v ".img\|.iso\|.bin"
   
   # Look for files in /home/pi/decrypt/ or similar
   find /mnt/sdcard/home/pi -type f -size +100k
   ```

## Most Likely Locations

Based on the log structure and deploy scripts:

1. **`/home/pi/decrypt/`** - Deployment scripts location (already found)
2. **Encrypted/compressed source** - Likely in `/home/pi/` or `/opt/` as a binary/archive file
3. **`/media/src/`** - Runtime location (only exists when mounted, not in SD-card image)
4. **Virtual environment**: `/home/pi/env/` - Python virtual environment
5. **Systemd service** - Should reference deploy.sh or main_rpi.py

## Status: Source Files Encrypted

**Finding**: `dcd` and `dcs` files are encrypted, preventing direct source code access.

**What We Know**:
- Application structure: Engine, Main, Autopilot, **MSP_RC**, Tracker
- Main entry: `/media/src/main_rpi.py`
- Protocol: MSP (MultiWii Serial Protocol)
- Betaflight config: MSP enabled on serial ports
- RC override: Channels 0-5 via MSP

**What We Can Still Do**:
Even without the source code, we have enough information to create an MSP controller because:
1. MSP protocol is well-documented (open standard)
2. Betaflight MSP commands are standardized
3. We know the exact configuration from the dump
4. We know the application structure from app.log

## Next Steps: Create MSP Controller from Documentation

Since the source is encrypted, we can create a Betaflight MSP controller based on:

### 1. MSP Protocol Documentation
- MSP is an open protocol used by Betaflight, MultiWii, Cleanflight, etc.
- Standard MSP commands are documented online
- Python MSP libraries exist (pymsp, multiwii-python, etc.)

### 2. Betaflight MSP Commands We Need
Based on your current MAVLink implementation, we need these MSP commands:

**RC Control**:
- `MSP_SET_RAW_RC` (200) - Send RC channel values (1000-2000 range)
  - Channels: Roll, Pitch, Throttle, Yaw, Aux1-12

**Telemetry**:
- `MSP_ATTITUDE` (108) - Get roll, pitch, yaw angles
- `MSP_ALTITUDE` (109) - Get altitude (baro/sonar)
- `MSP_RAW_IMU` (102) - Get IMU data (accelerometer, gyro, magnetometer)
- `MSP_STATUS` (101) - Get flight status (armed, mode, etc.)
- `MSP_ANALOG` (110) - Get battery voltage, current, RSSI
- `MSP_RC` (105) - Get current RC channel values

**Connection**:
- Serial port: Based on Betaflight config, likely `/dev/ttyAMA0` or `/dev/ttyAMA10`
- Baud rate: 115200 (serial 0) or 500000 (serial 4/5)
- Protocol: MSP v1 or MSP v2 (Betaflight 4.5.1 uses MSP API 1.46)

### 3. Implementation Strategy

We can create a `BetaflightController` class similar to your `MAVLinkController`:

```python
class BetaflightController:
    """MSP controller for Betaflight communication."""
    
    def __init__(self, config):
        self.connection_string = config.get('connection', '/dev/ttyAMA0')
        self.baud = config.get('baud', 115200)
        # MSP connection via serial
        
    def connect(self):
        # Open serial connection
        # Send MSP heartbeat/version request
        
    def send_rc_override(self, roll, pitch, yaw, throttle):
        # Convert to RC values (1000-2000)
        # Send MSP_SET_RAW_RC command
        
    def get_telemetry(self):
        # Request MSP_ATTITUDE, MSP_ALTITUDE, MSP_STATUS, MSP_ANALOG
        # Parse responses
        # Return dict similar to MAVLink telemetry
        
    def arm(self):
        # Send arm command via MSP_SET_RAW_RC (throttle low, yaw right)
        # Or use MSP_SET_MOTOR (if available)
        
    def disarm(self):
        # Send disarm command via MSP_SET_RAW_RC (throttle low, yaw left)
```

## Alternative: Extract from Running System

If you can boot the SD-card image (in a VM or on actual hardware):

1. **Boot the system** - Let it decrypt and mount `/media/src/`
2. **Copy decrypted files** - Once running, copy `/media/src/` to a safe location
3. **Extract MSP_RC module** - This will show us the exact MSP implementation

**Commands to run on live system**:
```bash
# After system boots and decrypts
cp -r /media/src /tmp/src_backup
# Or SSH in and copy files
```

## Recommendation

**Option 1: Build from Documentation (Recommended)**
- Use MSP protocol documentation
- Use existing Python MSP libraries (pymsp, multiwii-python)
- Create BetaflightController based on your MAVLinkController structure
- Test with your Betaflight flight controller

**Option 2: Extract from Live System**
- Boot the SD-card image
- Copy `/media/src/` after decryption
- Analyze MSP_RC.py to see exact implementation

**Option 3: Hybrid Approach**
- Start with Option 1 (build from docs)
- If needed, use Option 2 to verify/refine implementation

Would you like me to proceed with **Option 1** and create a Betaflight MSP controller based on MSP protocol documentation and your current MAVLink structure?

## Decryption Analysis: Finding Keys and Decryption Code

Since you have the complete SD-card image, the decryption keys and code must be somewhere on it. Let's search for:

### 1. Search for Decryption Scripts/Code
```bash
# Find files with "decrypt" in name
find /mnt/sdcard -iname "*decrypt*" -type f

# Find Python files that might decrypt
find /mnt/sdcard -name "*.py" -type f | xargs grep -l "decrypt\|decrypt\|crypt\|AES\|Fernet\|Crypto" 2>/dev/null

# Find shell scripts that decrypt
find /mnt/sdcard -name "*.sh" -type f | xargs grep -l "decrypt\|dcd\|dcs" 2>/dev/null
```

### 2. Search for Key Files
```bash
# Find key files
find /mnt/sdcard -name "*.key" -o -name "*.pem" -o -name "*.pub" -o -name "*key*" -type f

# Find files with "key" in name
find /mnt/sdcard -iname "*key*" -type f

# Check common key locations
ls -la /mnt/sdcard/home/pi/.ssh/
ls -la /mnt/sdcard/etc/ssl/
ls -la /mnt/sdcard/home/pi/decrypt/
```

### 3. Analyze dcd/dcs Files
```bash
# Check file type of dcd/dcs
file /mnt/sdcard/home/pi/decrypt/dcd
file /mnt/sdcard/home/pi/decrypt/dcs

# Check if they're Python bytecode
python3 -m py_compile --help  # Check if dcd is .pyc
strings /mnt/sdcard/home/pi/decrypt/dcd | head -20

# Check file headers (first bytes)
hexdump -C /mnt/sdcard/home/pi/decrypt/dcd | head -5
hexdump -C /mnt/sdcard/home/pi/decrypt/dcs | head -5
```

### 4. Find What Calls dcd
```bash
# Find scripts that call dcd
find /mnt/sdcard -name "*.sh" -type f | xargs grep -l "dcd" 2>/dev/null
find /mnt/sdcard -name "*.py" -type f | xargs grep -l "dcd" 2>/dev/null

# Check deploy.sh more carefully
cat /mnt/sdcard/home/pi/decrypt/deploy.sh

# Find systemd services that might decrypt
find /mnt/sdcard -name "*.service" -type f | xargs grep -l "dcd\|decrypt" 2>/dev/null
```

### 5. Check for Encryption Libraries
```bash
# Find Python crypto libraries
find /mnt/sdcard -path "*/site-packages/*" -iname "*crypto*" -o -iname "*cryptography*"

# Check if cryptography is installed
find /mnt/sdcard -path "*/site-packages/cryptography*" -type d
find /mnt/sdcard -path "*/site-packages/pycryptodome*" -type d
find /mnt/sdcard -path "*/site-packages/pycrypto*" -type d
```

### 6. Check Environment Variables and Config
```bash
# Find config files that might have keys
find /mnt/sdcard -name "*.conf" -o -name "*.config" -o -name "*.ini" | xargs grep -l "key\|password\|secret" 2>/dev/null

# Check .bashrc, .profile for key exports
find /mnt/sdcard/home/pi -name ".bashrc" -o -name ".profile" -o -name ".bash_profile" | xargs grep -i "key\|decrypt" 2>/dev/null
```

### 7. Check Systemd Services (Decryption Happens Before Launch)
```bash
# Find all systemd services
find /mnt/sdcard -path "*/systemd/system/*.service" -type f

# Check service dependencies (what runs before deploy.sh)
find /mnt/sdcard -path "*/systemd/system/*.service" -type f | xargs grep -l "deploy.sh\|zram.sh\|dcd" 2>/dev/null

# Check for init scripts
find /mnt/sdcard -path "*/systemd/system/*.service" -type f -exec cat {} \; | grep -A 5 -B 5 "dcd\|decrypt"
```

### 8. Check /var/lib/tfl/atg (from deploy.sh)
```bash
# deploy.sh checks for /var/lib/tfl/atg - what is this?
ls -la /mnt/sdcard/var/lib/tfl/
cat /mnt/sdcard/var/lib/tfl/atg  # Might contain key or flag

# Find what creates this file
find /mnt/sdcard -name "*.sh" -o -name "*.py" | xargs grep -l "/var/lib/tfl/atg" 2>/dev/null
```

## Decryption Strategy

### Approach 1: Find the Decryption Script
The `dcd` command in `deploy.sh` must be either:
- A script that decrypts `dcs` → extracts to `/media/src/`
- An executable that does the same
- A Python script (might be `.pyc` compiled)

### Approach 2: Reverse Engineer from Runtime
If the system boots:
1. Monitor `/media/src/` as it gets populated
2. Check process list during boot
3. Check systemd logs for decryption process

### Approach 3: Analyze File Structure
- Check if `dcd`/`dcs` are:
  - Encrypted archives (tar.gz.enc, zip.enc)
  - Python bytecode (.pyc)
  - Compiled binaries
  - Custom encrypted format

## What to Provide

Please run these commands and share the output:

1. **File types**:
   ```bash
   file /path/to/sdcard/home/pi/decrypt/dcd
   file /path/to/sdcard/home/pi/decrypt/dcs
   ```

2. **First bytes** (to identify format):
   ```bash
   hexdump -C /path/to/sdcard/home/pi/decrypt/dcd | head -10
   hexdump -C /path/to/sdcard/home/pi/decrypt/dcs | head -10
   ```

3. **All files in decrypt directory**:
   ```bash
   ls -lah /path/to/sdcard/home/pi/decrypt/
   ```

4. **Search results for decrypt/crypto**:
   ```bash
   find /path/to/sdcard -iname "*decrypt*" -type f
   find /path/to/sdcard -name "*.py" | xargs grep -l "crypt\|decrypt" 2>/dev/null | head -10
   ```

5. **Systemd services**:
   ```bash
   find /path/to/sdcard -name "*.service" -type f | xargs grep -l "dcd\|deploy" 2>/dev/null
   ```

With this information, we can determine the decryption method and extract the source code!

## Finding: /var/lib/tfl/atg is Also Encrypted

**Status**: `/var/lib/tfl/atg` exists but appears encrypted.

**Analysis Needed**:
1. **Check file type and size**:
   ```bash
   file /path/to/sdcard/var/lib/tfl/atg
   ls -lh /path/to/sdcard/var/lib/tfl/atg
   ```

2. **Check file header** (first bytes):
   ```bash
   hexdump -C /path/to/sdcard/var/lib/tfl/atg | head -10
   ```

3. **Check if it matches dcd/dcs format**:
   ```bash
   # Compare headers
   hexdump -C /path/to/sdcard/var/lib/tfl/atg | head -5
   hexdump -C /path/to/sdcard/home/pi/decrypt/dcd | head -5
   hexdump -C /path/to/sdcard/home/pi/decrypt/dcs | head -5
   ```

4. **List all files in /var/lib/tfl/**:
   ```bash
   ls -lah /path/to/sdcard/var/lib/tfl/
   ```

5. **Find what creates/uses this file**:
   ```bash
   # Search for references to /var/lib/tfl/atg
   find /path/to/sdcard -name "*.sh" -o -name "*.py" | xargs grep -l "/var/lib/tfl/atg" 2>/dev/null
   
   # Search for "tfl" or "atg" references
   find /path/to/sdcard -name "*.sh" -o -name "*.py" | xargs grep -l "tfl\|atg" 2>/dev/null
   ```

## Decryption Process Hypothesis

Based on `deploy.sh`:
```bash
if [ -f /var/lib/tfl/atg ]; then
    dcd
    ...
fi
```

**Possible scenarios**:
1. **`atg` is a key file**: `dcd` reads `atg` to decrypt `dcs`
2. **`atg` is a flag**: Presence of `atg` enables decryption (key might be hardcoded)
3. **`atg` contains encrypted key**: `dcd` decrypts `atg` first, then uses it for `dcs`

## Next Steps to Find Decryption Method

### 1. Analyze dcd Executable
Since `dcd` is called directly, it's likely:
- A shell script wrapper
- A Python script (possibly `.pyc`)
- A compiled binary
- A symlink to another file

**Check**:
```bash
# Check if dcd is executable
ls -lah /path/to/sdcard/home/pi/decrypt/dcd

# Check if it's a symlink
file /path/to/sdcard/home/pi/decrypt/dcd

# Try to read first few lines (if it's a script)
head -20 /path/to/sdcard/home/pi/decrypt/dcd

# Check if it's Python bytecode
python3 -c "import marshal; f=open('/path/to/sdcard/home/pi/decrypt/dcd','rb'); print(marshal.load(f))" 2>&1 | head -5
```

### 2. Search for Decryption Patterns
Look for common encryption methods:

```bash
# Search for encryption library imports
find /path/to/sdcard -name "*.py" | xargs grep -l "from cryptography\|import cryptography\|from Crypto\|import Crypto\|Fernet\|AES\|DES" 2>/dev/null

# Search for base64 (often used with encryption)
find /path/to/sdcard -name "*.py" | xargs grep -l "base64\|b64" 2>/dev/null

# Search for "decrypt" function definitions
find /path/to/sdcard -name "*.py" | xargs grep -l "def decrypt\|decrypt(" 2>/dev/null
```

### 3. Check Python Site-Packages
The decryption might use a library:

```bash
# List all installed packages
ls -la /path/to/sdcard/home/pi/env/lib/python*/site-packages/ 2>/dev/null

# Check for crypto libraries
find /path/to/sdcard -path "*/site-packages/*" -iname "*crypto*" -type d
```

### 4. Check for Hidden/Obscured Files
```bash
# Find all files in /home/pi/decrypt/ including hidden ones
ls -lah /path/to/sdcard/home/pi/decrypt/

# Find files starting with dot
find /path/to/sdcard/home/pi/decrypt/ -name ".*" -type f
```

### 5. Check Boot Process
The decryption might happen during boot:

```bash
# Find rc.local or boot scripts
find /path/to/sdcard -name "rc.local" -o -path "*/systemd/system/*.service" | xargs grep -l "dcd\|decrypt" 2>/dev/null

# Check /etc/init.d/
ls -la /path/to/sdcard/etc/init.d/ | grep -i decrypt
```

## Alternative: Runtime Extraction

If decryption is too complex, we can extract from running system:

1. **Boot the SD-card image** (VM or hardware)
2. **Wait for decryption** (check `/media/src/` appears)
3. **Copy decrypted files**:
   ```bash
   # On running system
   cp -r /media/src /tmp/src_backup
   tar -czf /tmp/src_backup.tar.gz /media/src
   ```
4. **Transfer to analysis machine**

## What to Share Next

Please provide:

1. **File types**:
   ```bash
   file /path/to/sdcard/home/pi/decrypt/dcd
   file /path/to/sdcard/home/pi/decrypt/dcs
   file /path/to/sdcard/var/lib/tfl/atg
   ```

2. **File sizes**:
   ```bash
   ls -lh /path/to/sdcard/home/pi/decrypt/dcd
   ls -lh /path/to/sdcard/home/pi/decrypt/dcs
   ls -lh /path/to/sdcard/var/lib/tfl/atg
   ```

3. **First bytes comparison**:
   ```bash
   hexdump -C /path/to/sdcard/home/pi/decrypt/dcd | head -5
   hexdump -C /path/to/sdcard/home/pi/decrypt/dcs | head -5
   hexdump -C /path/to/sdcard/var/lib/tfl/atg | head -5
   ```

4. **All files in decrypt directory**:
   ```bash
   ls -lah /path/to/sdcard/home/pi/decrypt/
   ```

5. **What dcd actually is**:
   ```bash
   head -50 /path/to/sdcard/home/pi/decrypt/dcd  # If it's text
   strings /path/to/sdcard/home/pi/decrypt/dcd | head -20  # If it's binary
   ```

With this information, we can determine if `dcd` is decryptable or if we need to extract from a running system!

## CRITICAL FINDING: dcd is a Compiled Go Binary

**Analysis of dcd file header**:
- **File Type**: ELF (Executable and Linkable Format) binary
- **Architecture**: ARM64 (aarch64) - `/lib/ld-linux-aarch64.so.1`
- **Language**: Go (Golang) - Contains `_cgo_topofstack`, `_cgo_panic`, `crosscall2` symbols
- **Status**: Compiled binary, not a script

**What this means**:
- `dcd` is a compiled Go program that decrypts `dcs`
- The decryption logic is compiled into the binary
- We cannot easily read the source code or decryption method
- Reverse engineering would require decompiling the Go binary

## Options Moving Forward

### Option 1: Runtime Extraction (RECOMMENDED)
Since `dcd` is a working binary, we can:

1. **Boot the SD-card image** (VM or hardware)
2. **Let it decrypt automatically** - `dcd` will decrypt `dcs` → extract to `/media/src/`
3. **Copy decrypted files**:
   ```bash
   # On running system, after decryption completes
   cp -r /media/src /tmp/src_backup
   tar -czf /tmp/src_backup.tar.gz /media/src
   
   # Or copy specific files
   cp -r /media/src/MSP_RC* /tmp/
   cp -r /media/src/*.py /tmp/
   ```

**Advantages**:
- No reverse engineering needed
- Get actual source code
- See exact MSP implementation
- Fastest method

**Steps**:
1. Boot SD-card image
2. Wait for system to boot and decrypt (check `/media/src/` appears)
3. SSH in or use console
4. Copy `/media/src/` to external location
5. Transfer to analysis machine

### Option 2: Extract Strings from Binary
We can extract readable strings from `dcd` to find clues:

```bash
# Extract all strings from dcd
strings /path/to/sdcard/home/pi/decrypt/dcd > dcd_strings.txt

# Look for encryption-related strings
strings /path/to/sdcard/home/pi/decrypt/dcd | grep -i "aes\|decrypt\|encrypt\|key\|crypto\|fernet"

# Look for file paths
strings /path/to/sdcard/home/pi/decrypt/dcd | grep -E "/media|/var/lib/tfl|dcs|\.py"
```

This might reveal:
- Encryption method used
- Key locations
- File paths
- Library names

### Option 3: Binary Analysis
Advanced reverse engineering:

```bash
# Use objdump to see symbols
aarch64-linux-gnu-objdump -T /path/to/sdcard/home/pi/decrypt/dcd

# Use readelf to see sections
aarch64-linux-gnu-readelf -a /path/to/sdcard/home/pi/decrypt/dcd

# Use strings to find clues
strings /path/to/sdcard/home/pi/decrypt/dcd | head -100
```

### Option 4: Build MSP Controller from Documentation
Since we know:
- Protocol: MSP
- Betaflight version: 4.5.1 (MSP API 1.46)
- Configuration: Serial ports, baud rates, MSP override settings

We can create a Betaflight MSP controller using:
- MSP protocol documentation
- Python MSP libraries (pymsp, multiwii-python)
- Your current MAVLink controller as a template

## Recommendation

**Use Option 1 (Runtime Extraction)** because:
1. ✅ Fastest way to get source code
2. ✅ No reverse engineering needed
3. ✅ Get exact MSP_RC implementation
4. ✅ Can verify against our implementation later

**Then use Option 4** to create Betaflight controller based on:
- Extracted MSP_RC.py (if we get it)
- OR MSP protocol documentation (if extraction fails)

## Next Steps

1. **Try to boot the SD-card image**:
   - Use QEMU/KVM for ARM64
   - Or boot on actual Raspberry Pi hardware
   - Wait for system to fully boot

2. **Check if decryption happened**:
   ```bash
   ls -la /media/src/
   # Should see Python files including MSP_RC.py
   ```

3. **Copy decrypted files**:
   ```bash
   # Create backup
   cp -r /media/src /tmp/src_backup
   
   # Or copy to USB/external storage
   # Or use SCP/SSH to transfer
   ```

4. **Extract MSP_RC module**:
   ```bash
   # Find MSP_RC files
   find /tmp/src_backup -name "*MSP*" -o -name "*msp*"
   
   # Copy to analysis location
   ```

Would you like help setting up a VM to boot the SD-card image, or do you have access to hardware to boot it?

## Troubleshooting: "boot partition no FAT" Error

This error means the Raspberry Pi cannot find a bootable FAT32 partition. Common causes and solutions:

### Issue 1: Image File Format
The image file might be:
- A filesystem image (already extracted)
- A recovery/backup format
- Corrupted or incomplete

**Check your image file**:
```bash
# On Windows, check file properties
# Right-click .img file → Properties → Check file size

# If the path contains "recovery", it might be extracted already
# Your path: ...\Образ флэшки\recovery\...
```

**Solution**: If it's already extracted:
- You might need to create a bootable image from the extracted files
- Or find the actual disk image file (usually larger, several GB)

### Issue 2: Flashing Tool Issue
Some tools don't handle certain image formats correctly.

**Try different tools**:
1. **Raspberry Pi Imager** (most reliable)
2. **balenaEtcher** (good compatibility)
3. **Win32DiskImager** (older but reliable)
4. **Rufus** (alternative option)

### Issue 3: Image File Corruption
The image might be corrupted or incomplete.

**Verify image**:
- Check file size (should be several GB for full Raspberry Pi image)
- Try re-downloading/copying the image file
- Check if image file has checksum/MD5 to verify integrity

### Issue 4: SD Card Issues
- SD card might be incompatible
- Card might be corrupted
- Card might be too small

**Try**:
- Use a different SD card (Class 10, 16GB+ recommended)
- Format SD card first (FAT32) before flashing
- Use a different card reader

### Issue 5: Image Needs Extraction
If the image is in a container format (zip, tar.gz, etc.):

**Extract first**:
```bash
# If it's a .zip file
# Extract to get .img file

# If it's a .tar.gz file  
# Extract to get .img file

# Then flash the extracted .img file
```

### Issue 6: Wrong Image Type
The image might be:
- A filesystem backup (not bootable)
- A recovery image (needs special handling)
- An incomplete backup

**Check image structure**:
- Bootable Raspberry Pi images should have:
  - Boot partition (FAT32, ~256MB)
  - Root filesystem partition (ext4, rest of space)

### Solution Steps

1. **Verify Image File**:
   - Check file size (should be GB, not MB)
   - Check file extension (.img)
   - Check if it's actually a disk image or extracted files

2. **Try Raspberry Pi Imager**:
   - Download from official site
   - Use "Use custom image" option
   - Let it verify the image before flashing

3. **Check Image File Location**:
   - Your path shows "recovery" folder
   - Make sure you're using the actual .img file, not extracted files
   - Look for a file like `raspberry-pi-image.img` or similar

4. **Re-flash with Verification**:
   - Use Raspberry Pi Imager
   - Enable "Verify after write" option
   - This will check if flash was successful

5. **Check SD Card After Flashing**:
   - After flashing, Windows might ask to format - **DON'T format**
   - You should see two partitions:
     - Boot partition (FAT32, ~256MB)
     - Linux partition (not readable in Windows)
   - If you only see one partition or it's empty, flashing failed

### Alternative: Mount Image to Extract Files

If booting fails, you can try to mount the image directly to extract files:

**On Linux/WSL**:
```bash
# Mount the image file
sudo losetup -P /dev/loop0 raspberry-pi-image.img

# Mount partitions
sudo mkdir -p /mnt/boot /mnt/root
sudo mount /dev/loop0p1 /mnt/boot  # Boot partition
sudo mount /dev/loop0p2 /mnt/root  # Root filesystem

# Look for decrypted files
ls -la /mnt/root/media/src/

# Copy files
cp -r /mnt/root/media/src /tmp/extracted_src

# Unmount
sudo umount /mnt/boot /mnt/root
sudo losetup -d /dev/loop0
```

**On Windows (using WSL or Linux VM)**:
- Use WSL2 with Ubuntu
- Follow Linux instructions above

### Quick Diagnostic Questions

1. **What is the file size of your .img file?**
   - Should be several GB (e.g., 4GB, 8GB, 16GB)

2. **What tool did you use to flash?**
   - Try Raspberry Pi Imager if you haven't

3. **After flashing, can you see partitions in Windows Disk Management?**
   - Should see boot partition (FAT32)
   - Should see Linux partition (unreadable in Windows)

4. **Is the image file in a "recovery" folder?**
   - Might need to find the actual disk image file
   - Recovery folder might contain extracted files, not the image

5. **Did you verify the image file integrity?**
   - Check if there's a checksum file
   - Try re-copying the image file

### Next Steps

1. **Verify image file** - Check size, format, location
2. **Try Raspberry Pi Imager** - Most reliable tool
3. **Check SD card** - Try different card if available
4. **Mount image directly** - Extract files without booting (if image is valid)

Share:
- Image file size
- Tool used for flashing
- What you see in Windows Disk Management after flashing
- Whether the image is in a "recovery" folder or is it the actual .img file

This will help diagnose the exact issue!

## Image Size Issue: 28.6 GB on 32 GB SD Card

**Problem**: Your image is 28.6 GB and SD card is 32 GB - this leaves very little margin.

**Issues this can cause**:
- Flashing tools need extra space for verification
- Some tools fail if there's less than 10% free space
- Write errors can occur near the end

### Solutions

#### Solution 1: Disable Verification (Temporary)
Some tools verify after writing, which needs extra space:

**Raspberry Pi Imager**:
- Flash without "Verify after write" option
- Or use a different tool that doesn't verify

**balenaEtcher**:
- Usually doesn't verify by default
- Try this tool instead

#### Solution 2: Use Larger SD Card
- Get a 64 GB SD card (recommended)
- Gives plenty of space for flashing and verification

#### Solution 3: Mount Image Directly (BEST OPTION)
Since booting is problematic, mount the image directly to extract files:

**Using WSL2 (Recommended)**:
```bash
# 1. Install WSL2 if needed
wsl --install

# 2. In WSL2 Ubuntu terminal, navigate to image location
cd /mnt/c/Users/Ilya/Desktop/university/fpv/ext_projects/Автонаведение/Образ\ флэшки/

# 3. Install required tools
sudo apt update
sudo apt install -y kpartx

# 4. Mount the image file
sudo losetup -P /dev/loop0 recovery.img
# (Replace recovery.img with your actual .img filename)

# 5. Wait a moment for partitions to be detected
sleep 2

# 6. Check what partitions were created
ls -la /dev/loop0*

# 7. Mount the root filesystem (usually loop0p2)
sudo mkdir -p /mnt/pi_root
sudo mount /dev/loop0p2 /mnt/pi_root

# 8. Check if /media/src exists (decrypted files)
ls -la /mnt/pi_root/media/src/

# 9. If decrypted files exist, copy them
mkdir -p ~/extracted_src
cp -r /mnt/pi_root/media/src/* ~/extracted_src/

# 10. If decrypted files don't exist, check for encrypted source
ls -la /mnt/pi_root/home/pi/decrypt/

# 11. Copy important files
cp -r /mnt/pi_root/home/pi/decrypt/* ~/extracted_src/decrypt/
cp -r /mnt/pi_root/home/pi/env ~/extracted_src/env/ 2>/dev/null || true

# 12. Find Python files
find /mnt/pi_root -name "*.py" -type f > ~/extracted_src/python_files.txt

# 13. Copy Python files (if any readable ones exist)
find /mnt/pi_root -name "*.py" -type f -exec cp {} ~/extracted_src/python/ \; 2>/dev/null || true

# 14. Unmount when done
sudo umount /mnt/pi_root
sudo losetup -d /dev/loop0

# 15. Copy extracted files to Windows
cp -r ~/extracted_src /mnt/c/Users/Ilya/Desktop/extracted_src
```

**Using Linux VM**:
- Use VirtualBox/VMware with Ubuntu
- Mount the image file as a disk
- Follow similar steps

#### Solution 4: Check Image Integrity
The image might be corrupted:

```bash
# In WSL2 or Linux
file /path/to/image.img
# Should show: "DOS/MBR boot sector" or "Linux filesystem"

# Check if it's actually a disk image
fdisk -l /path/to/image.img
# Should show partition table
```

### Why Mounting is Better

**Advantages of mounting image directly**:
1. ✅ No need to boot Raspberry Pi
2. ✅ No SD card space issues
3. ✅ Can access all files immediately
4. ✅ Can extract encrypted files even if they're not decrypted
5. ✅ Faster than booting and waiting

**What you can extract**:
- Encrypted `dcd` and `dcs` files
- Configuration files (`device_info.json`, etc.)
- Python virtual environment (might have MSP libraries)
- Systemd service files
- Any readable Python scripts

### Step-by-Step: Mount and Extract

1. **Install WSL2** (if not already installed):
   ```powershell
   # In PowerShell as Administrator
   wsl --install
   # Restart computer when prompted
   ```

2. **Open WSL2 Ubuntu**:
   - Search for "Ubuntu" in Start menu
   - First time: create username/password

3. **Navigate to image location**:
   ```bash
   cd /mnt/c/Users/Ilya/Desktop/university/fpv/ext_projects/Автонаведение/Образ\ флэшки/
   ls -lh *.img
   ```

4. **Mount and extract** (use commands from Solution 3 above)

5. **Check extracted files**:
   ```bash
   ls -lah ~/extracted_src/
   ```

### If Image Won't Mount

If mounting fails, the image might be:
- Corrupted
- In a special format
- Encrypted at the disk level

**Try**:
```bash
# Check file type
file your-image.img

# Try different mounting methods
sudo fdisk -l your-image.img
sudo partprobe /dev/loop0
```

### Next Steps

1. **Try mounting the image** (fastest way to get files)
2. **If mounting works**: Extract all files, especially:
   - `/home/pi/decrypt/` directory
   - `/home/pi/env/` (Python virtual environment)
   - Any `.py` files you can find
   - `/var/lib/tfl/atg` file

3. **If mounting doesn't work**: 
   - Try re-flashing with Raspberry Pi Imager (disable verification)
   - Or get a larger SD card (64 GB)

**Share what you find**:
- Can you mount the image?
- What files do you see in `/home/pi/decrypt/`?
- Are there any readable Python files?
- What's in the Python virtual environment?

This will help us get the MSP code even without booting!

## CRITICAL: Split Partition Backup Format

**Your backup format**:
- `0.fat` - Boot partition (FAT32, ~256MB)
- `1.img` - Root filesystem partition (ext4, ~28GB)

This is a **partition-level backup**, not a full disk image. We need to either:
1. Create a bootable disk image from these partitions
2. Flash partitions directly to SD card
3. **Mount partitions directly to extract files** (FASTEST - no booting needed!)

### Option 1: Mount Partitions Directly (RECOMMENDED - No Booting!)

**Using WSL2**:
```bash
# 1. Navigate to backup location
cd "/mnt/c/Users/Ilya/Desktop/university/fpv/ext_projects/Автонаведение/Образ флэшки/"

# 2. Mount the root filesystem (1.img) - THIS HAS ALL THE FILES!
sudo mkdir -p /mnt/pi_root
sudo mount -o loop,ro 1.img /mnt/pi_root

# 3. Check if it mounted successfully
ls -la /mnt/pi_root/

# 4. Look for decrypted files (if system was running when backed up)
ls -la /mnt/pi_root/media/src/ 2>/dev/null || echo "No /media/src (expected - only exists at runtime)"

# 5. Extract important files
mkdir -p ~/extracted_src

# Copy decrypt directory
cp -r /mnt/pi_root/home/pi/decrypt ~/extracted_src/decrypt/

# Copy Python virtual environment
cp -r /mnt/pi_root/home/pi/env ~/extracted_src/env/ 2>/dev/null || true

# Copy device_info.json
cp /mnt/pi_root/home/pi/device_info.json ~/extracted_src/ 2>/dev/null || true

# Copy atg file
cp /mnt/pi_root/var/lib/tfl/atg ~/extracted_src/ 2>/dev/null || true

# Find all Python files
find /mnt/pi_root -name "*.py" -type f > ~/extracted_src/python_files.txt

# Copy any readable Python files
mkdir -p ~/extracted_src/python
find /mnt/pi_root -name "*.py" -type f -exec cp {} ~/extracted_src/python/ \; 2>/dev/null || true

# Check Python virtual environment for MSP libraries
find /mnt/pi_root/home/pi/env -name "*msp*" -o -name "*betaflight*" > ~/extracted_src/msp_libraries.txt 2>/dev/null || true

# 6. Copy to Windows
cp -r ~/extracted_src /mnt/c/Users/Ilya/Desktop/extracted_src

# 7. Unmount
sudo umount /mnt/pi_root
```

**This gives you**:
- ✅ All encrypted files (`dcd`, `dcs`, `atg`)
- ✅ Python virtual environment (with MSP libraries!)
- ✅ Configuration files
- ✅ Any readable Python scripts
- ✅ No need to boot or decrypt!

### Option 2: Create Bootable Disk Image

**Using WSL2/Linux**:
```bash
# 1. Create a disk image file (32GB)
dd if=/dev/zero of=raspberry-pi-full.img bs=1M count=32768

# 2. Create partition table
parted raspberry-pi-full.img <<EOF
mklabel msdos
mkpart primary fat32 8192s 532479s    # Boot partition (~256MB)
mkpart primary ext4 532480s 100%     # Root partition (rest)
quit
EOF

# 3. Set up loop device
sudo losetup -P /dev/loop0 raspberry-pi-full.img

# 4. Format partitions
sudo mkfs.vfat -F 32 /dev/loop0p1
sudo mkfs.ext4 /dev/loop0p2

# 5. Mount partitions
sudo mkdir -p /mnt/boot /mnt/root
sudo mount /dev/loop0p1 /mnt/boot
sudo mount /dev/loop0p2 /mnt/root

# 6. Copy boot partition
sudo dd if=0.fat of=/dev/loop0p1 bs=4M

# 7. Copy root partition
sudo dd if=1.img of=/dev/loop0p2 bs=4M

# 8. Unmount
sudo umount /mnt/boot /mnt/root
sudo losetup -d /dev/loop0

# 9. Now flash raspberry-pi-full.img to SD card
```

### Option 3: Flash Partitions Directly to SD Card

**Using WSL2/Linux** (SD card must be inserted):
```bash
# 1. Identify SD card device (BE CAREFUL - wrong device will destroy data!)
lsblk
# Look for your SD card (usually /dev/sdX or /dev/mmcblk0)

# 2. Unmount any mounted partitions
sudo umount /dev/sdX* 2>/dev/null || true

# 3. Create partition table on SD card
sudo parted /dev/sdX <<EOF
mklabel msdos
mkpart primary fat32 8192s 532479s
mkpart primary ext4 532480s 100%
quit
EOF

# 4. Format partitions
sudo mkfs.vfat -F 32 /dev/sdX1
sudo mkfs.ext4 /dev/sdX2

# 5. Copy boot partition
sudo dd if=0.fat of=/dev/sdX1 bs=4M status=progress

# 6. Copy root partition
sudo dd if=1.img of=/dev/sdX2 bs=4M status=progress

# 7. Sync to ensure writes complete
sync
```

**⚠️ WARNING**: Replace `/dev/sdX` with your actual SD card device! Wrong device = data loss!

### Option 4: Use Raspberry Pi Imager with Custom Script

Raspberry Pi Imager doesn't directly support split partitions, but you can:
1. Use Option 2 to create full image first
2. Then flash with Raspberry Pi Imager

## Recommendation: Use Option 1 (Mount Directly)

**Why Option 1 is best**:
1. ✅ **Fastest** - No need to create image or flash SD card
2. ✅ **No booting needed** - Extract files immediately
3. ✅ **No decryption needed** - Get encrypted files directly
4. ✅ **Can analyze Python virtual environment** - Find MSP libraries
5. ✅ **Can extract all important files** - decrypt/, env/, configs

**What you'll get**:
- Encrypted `dcd`, `dcs`, `atg` files (for analysis)
- Python virtual environment (check for MSP libraries!)
- Configuration files (`device_info.json`, etc.)
- Any readable Python scripts
- Systemd service files

## Quick Start: Mount and Extract

**In WSL2**:
```bash
# Mount root partition
sudo mount -o loop,ro 1.img /mnt/pi_root

# Extract everything important
mkdir -p ~/extracted_src
cp -r /mnt/pi_root/home/pi/decrypt ~/extracted_src/
cp -r /mnt/pi_root/home/pi/env ~/extracted_src/ 2>/dev/null || true
find /mnt/pi_root -name "*.py" -type f > ~/extracted_src/python_files.txt

# Copy to Windows
cp -r ~/extracted_src /mnt/c/Users/Ilya/Desktop/extracted_src

# Unmount
sudo umount /mnt/pi_root
```

**Then check**:
- `~/extracted_src/env/lib/python*/site-packages/` - Look for MSP libraries!
- `~/extracted_src/decrypt/` - Encrypted files for analysis
- `~/extracted_src/python_files.txt` - List of all Python files

This is the fastest way to get the MSP code!

## Creating Bootable SD Card on Windows

### Method 1: Using WSL2 + Raspberry Pi Imager (RECOMMENDED)

**Step 1: Combine partitions into full disk image using WSL2**

```bash
# Open WSL2 Ubuntu terminal

# Navigate to backup location
cd "/mnt/c/Users/Ilya/Desktop/university/fpv/ext_projects/Автонаведение/Образ флэшки/"

# Create a 32GB disk image file
dd if=/dev/zero of=raspberry-pi-full.img bs=1M count=32768

# Create partition table (MBR/MSDOS)
parted raspberry-pi-full.img <<EOF
mklabel msdos
mkpart primary fat32 8192s 532479s
mkpart primary ext4 532480s 100%
quit
EOF

# Set up loop device
sudo losetup -P /dev/loop0 raspberry-pi-full.img

# Format partitions
sudo mkfs.vfat -F 32 /dev/loop0p1
sudo mkfs.ext4 /dev/loop0p2

# Mount partitions
sudo mkdir -p /mnt/boot /mnt/root
sudo mount /dev/loop0p1 /mnt/boot
sudo mount /dev/loop0p2 /mnt/root

# Copy boot partition (0.fat)
sudo dd if=0.fat of=/dev/loop0p1 bs=4M status=progress

# Copy root partition (1.img)
sudo dd if=1.img of=/dev/loop0p2 bs=4M status=progress

# Sync to ensure writes complete
sync

# Unmount
sudo umount /mnt/boot /mnt/root
sudo losetup -d /dev/loop0

# The full image is now at: raspberry-pi-full.img
# Copy it to Windows-accessible location
cp raspberry-pi-full.img /mnt/c/Users/Ilya/Desktop/raspberry-pi-full.img
```

**Step 2: Flash with Raspberry Pi Imager**

1. Download Raspberry Pi Imager: https://www.raspberrypi.com/software/
2. Install and open Raspberry Pi Imager
3. Click "Choose OS" → Scroll down → "Use custom image"
4. Select `raspberry-pi-full.img` from your Desktop
5. Click "Choose Storage" → Select your SD card
6. Click "Write" (disable "Verify after write" if you get space errors)
7. Wait for completion

### Method 2: Direct Flash Using WSL2 (Advanced)

**If you're comfortable with command line**:

```bash
# In WSL2 Ubuntu terminal

# 1. Insert SD card into Windows
# 2. In WSL2, find SD card device
lsblk
# Look for your SD card (might show as /dev/sdb, /dev/sdc, etc.)
# Check size to identify it correctly!

# 3. IMPORTANT: Unmount any Windows-mounted partitions first
# In PowerShell (as Administrator):
# Get-Disk | Where-Object {$_.Size -lt 50GB -and $_.Size -gt 20GB} | Get-Partition | Remove-PartitionAccessPath -AccessPath <drive-letter>

# 4. Create partition table on SD card
sudo parted /dev/sdX <<EOF
mklabel msdos
mkpart primary fat32 8192s 532479s
mkpart primary ext4 532480s 100%
quit
EOF

# 5. Format partitions
sudo mkfs.vfat -F 32 /dev/sdX1
sudo mkfs.ext4 /dev/sdX2

# 6. Copy boot partition
sudo dd if=0.fat of=/dev/sdX1 bs=4M status=progress

# 7. Copy root partition
sudo dd if=1.img of=/dev/sdX2 bs=4M status=progress

# 8. Sync
sync
```

**⚠️ CRITICAL WARNING**: Replace `/dev/sdX` with your actual SD card device! Wrong device = data loss!

### Method 3: Using Win32DiskImager (Alternative)

**Step 1: Create full image using WSL2** (same as Method 1, Step 1)

**Step 2: Flash with Win32DiskImager**:
1. Download Win32DiskImager: https://sourceforge.net/projects/win32diskimager/
2. Run as Administrator
3. Select `raspberry-pi-full.img`
4. Select SD card drive
5. Click "Write"
6. Wait for completion

### Method 4: Using balenaEtcher (Easiest)

**Step 1: Create full image using WSL2** (same as Method 1, Step 1)

**Step 2: Flash with balenaEtcher**:
1. Download balenaEtcher: https://www.balena.io/etcher/
2. Install and open
3. Click "Flash from file"
4. Select `raspberry-pi-full.img`
5. Click "Select target" → Choose SD card
6. Click "Flash!"
7. Wait for completion

## Quick Reference: WSL2 Commands

**If WSL2 is not installed**:
```powershell
# In PowerShell as Administrator
wsl --install
# Restart computer when prompted
```

**To open WSL2**:
- Search for "Ubuntu" in Start menu
- Or type `wsl` in PowerShell/Command Prompt

**To navigate to Windows files in WSL2**:
```bash
cd /mnt/c/Users/Ilya/Desktop/university/fpv/ext_projects/Автонаведение/Образ\ флэшки/
```

## Troubleshooting

### Issue: "Permission denied" in WSL2
**Solution**: Use `sudo` for mount/dd commands

### Issue: SD card not detected in WSL2
**Solution**: 
- Make sure SD card is inserted
- Check in Windows Disk Management first
- In WSL2, run `lsblk` to see all block devices

### Issue: "Device busy" error
**Solution**: 
- Unmount partitions first: `sudo umount /dev/sdX*`
- Or close any Windows Explorer windows accessing the SD card

### Issue: Wrong device selected
**Solution**: 
- Always double-check device with `lsblk`
- Check device size matches your SD card
- When in doubt, don't proceed!

## Recommended Workflow

1. **First**: Mount `1.img` to extract files (see Option 1 above) - Get MSP code immediately!
2. **Then**: Create bootable SD card if you want to boot the system later

**Why this order?**
- Extracting files is faster and gives you what you need
- Creating bootable SD card takes time and may have issues
- You can analyze MSP code while SD card is being prepared

## Next Steps

1. **Extract files first** (5 minutes):
   ```bash
   sudo mount -o loop,ro 1.img /mnt/pi_root
   cp -r /mnt/pi_root/home/pi/env ~/extracted_src/env/
   cp -r /mnt/pi_root/home/pi/decrypt ~/extracted_src/decrypt/
   ```

2. **Then create bootable SD card** (30+ minutes):
   - Use Method 1 (WSL2 + Raspberry Pi Imager)
   - Or Method 4 (WSL2 + balenaEtcher) - easiest

Want me to walk you through the extraction process first? That's the fastest way to get the MSP code!

