# Connection Working Configuration

## ✅ Successfully Working Configuration

Based on testing, the following configuration works:

- **Port:** `/dev/ttyAMA10` (or `/dev/serial0` which symlinks to it)
- **Baud Rate:** 256000
- **Status:** Heartbeat received ✓

## Findings

### Serial Port Discovery
Your Raspberry Pi uses `/dev/ttyAMA10` instead of the more common `/dev/ttyAMA0`. This is normal for certain Raspberry Pi models or configurations.

The symlink `/dev/serial0` points to `/dev/ttyAMA10`:
```bash
$ ls -l /dev/serial0
lrwxrwxrwx 1 root root 8 Oct 29 22:33 /dev/serial0 -> ttyAMA10
```

### System/Component ID Extraction
The initial heartbeat reception was successful, but the System ID and Component ID were showing as 0 because:
- `master.target_system` and `master.target_component` may not be populated immediately
- We need to read the actual heartbeat message to extract the source IDs

**Fixed:** Updated scripts now read the heartbeat message directly and extract IDs using:
- `fresh_hb.get_srcSystem()` - Gets the source system ID
- `fresh_hb.get_srcComponent()` - Gets the source component ID

## Updated Test Scripts

Both test scripts have been updated:
1. ✅ `simple_heartbeat_test.py` - Now includes `/dev/ttyAMA10` and properly extracts IDs
2. ✅ `test_connection.py` - Now includes `/dev/ttyAMA10` and properly extracts IDs

## Recommended Configuration

For production use, consider using `/dev/serial0` instead of `/dev/ttyAMA10`:
- More portable (works regardless of which actual device is used)
- Automatically points to the correct serial port
- Standard Linux convention

### Update Production Config (Optional)

You can update `drone_client/config/production.yaml`:

```yaml
mavlink:
  connection: "/dev/serial0"  # or "/dev/ttyAMA10"
  baud: 256000
```

Both will work since `/dev/serial0` → `/dev/ttyAMA10`.

## Next Steps

1. ✅ Connection verified - Heartbeat is working
2. Run `simple_heartbeat_test.py` again to see proper System/Component IDs
3. Run `test_connection.py` for comprehensive diagnostics
4. Test with `basic_config_check.py` to see full system information
5. Configure production system to use `/dev/serial0` or `/dev/ttyAMA10`

## Testing

Run the updated test:
```bash
python3 simple_heartbeat_test.py
```

Expected output should now show:
- ✓ System ID: [actual number, not 0]
- ✓ Component ID: [actual number, not 0]
- ✓ Vehicle Type: [Quadrotor/Hexarotor/etc]
- ✓ Autopilot: [ArduPilot/PX4]

