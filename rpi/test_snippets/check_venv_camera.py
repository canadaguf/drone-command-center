#!/usr/bin/env python3
"""
Check if current virtual environment is properly configured for picamera2
"""

import sys
import os

print("=" * 60)
print("Virtual Environment Camera Compatibility Check")
print("=" * 60)

# Check if we're in a venv
in_venv = hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)

if not in_venv:
    print("⚠️  Not running in a virtual environment")
    print("   This check is for venv compatibility")
    sys.exit(1)

print(f"\n✓ Running in virtual environment")
print(f"  Python: {sys.executable}")
print(f"  Prefix: {sys.prefix}")

# Check if venv was created with --system-site-packages
# This is stored in pyvenv.cfg
pyvenv_cfg = os.path.join(sys.prefix, 'pyvenv.cfg')
if os.path.exists(pyvenv_cfg):
    with open(pyvenv_cfg, 'r') as f:
        content = f.read()
        if 'include-system-site-packages = true' in content:
            print(f"\n✓ Venv created with --system-site-packages")
            print(f"  This is GOOD for picamera2!")
        else:
            print(f"\n⚠️  Venv created WITHOUT --system-site-packages")
            print(f"  picamera2 may not work properly")
            print(f"\n  Fix options:")
            print(f"  1. Test if it works anyway (system packages might be accessible)")
            print(f"  2. Recreate venv with system-site-packages:")
            print(f"     deactivate")
            print(f"     rm -rf {sys.prefix}")
            print(f"     python3 -m venv --system-site-packages {sys.prefix}")
            print(f"     source {sys.prefix}/bin/activate")
else:
    print(f"\n⚠️  Could not find pyvenv.cfg")

# Check if system picamera2 is accessible
print(f"\n[1/3] Checking system picamera2 package...")
try:
    import importlib.util
    # Try to find python3-picamera2 system package
    system_site_packages = []
    for path in sys.path:
        if 'site-packages' in path and 'dist-packages' in path:
            system_site_packages.append(path)
    
    if system_site_packages:
        print(f"  System site-packages paths found")
        # Check if picamera2 is in system packages
        picamera2_found = False
        for sp_path in system_site_packages:
            picamera2_path = os.path.join(sp_path, 'picamera2')
            if os.path.exists(picamera2_path):
                print(f"  ✓ Found picamera2 in system packages: {picamera2_path}")
                picamera2_found = True
                break
        
        if not picamera2_found:
            print(f"  ⚠️  picamera2 not found in system packages")
            print(f"     Install with: sudo apt install -y python3-picamera2")
    else:
        print(f"  ⚠️  Could not determine system site-packages")
except Exception as e:
    print(f"  ⚠️  Error checking system packages: {e}")

# Test import
print(f"\n[2/3] Testing picamera2 import...")
try:
    from picamera2 import Picamera2
    print(f"  ✓ picamera2 imported successfully!")
    
    # Try to get version if available
    try:
        import picamera2
        if hasattr(picamera2, '__version__'):
            print(f"  ✓ Version: {picamera2.__version__}")
    except:
        pass
    
except ImportError as e:
    print(f"  ❌ Failed to import picamera2: {e}")
    print(f"\n  Troubleshooting:")
    print(f"  1. Install system package:")
    print(f"     sudo apt install -y python3-picamera2 libcamera-dev libcap-dev")
    print(f"  2. Install Python package:")
    print(f"     pip install picamera2")
    print(f"  3. If import still fails, recreate venv with --system-site-packages")
except Exception as e:
    print(f"  ❌ Unexpected error: {e}")

# Test numpy (required dependency)
print(f"\n[3/3] Testing numpy import...")
try:
    import numpy as np
    print(f"  ✓ numpy {np.__version__} imported successfully")
except ImportError:
    print(f"  ❌ numpy not found")
    print(f"     Install with: pip install numpy")

print(f"\n" + "=" * 60)
print("Summary")
print("=" * 60)

# Final check
try:
    from picamera2 import Picamera2
    import numpy as np
    print("✓ All imports successful - camera should work!")
    print("\nNext step: Run simple_camera_test.py to test camera hardware")
except ImportError as e:
    print(f"❌ Import failed: {e}")
    print("\nFix the import issues above before testing camera hardware")

