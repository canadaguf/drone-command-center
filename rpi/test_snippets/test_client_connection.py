#!/usr/bin/env python3
"""
Test drone client connection and initialization
Tests MAVLink, WebSocket, and basic client setup
"""

import sys
import asyncio
import logging
from pathlib import Path

# Add parent directory to path to import drone_client
sys.path.insert(0, str(Path(__file__).parent.parent))

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger(__name__)

async def test_mavlink_connection():
    """Test MAVLink connection."""
    print("\n" + "=" * 60)
    print("[1/3] Testing MAVLink Connection")
    print("=" * 60)
    
    try:
        from drone_client.config import Config
        from drone_client.controllers.mavlink_controller import MAVLinkController
        
        config = Config()
        mavlink_config = config.get_mavlink_config()
        
        print(f"  Connection: {mavlink_config.get('connection')}")
        print(f"  Baud Rate: {mavlink_config.get('baud')}")
        
        mavlink = MAVLinkController(mavlink_config)
        
        if mavlink.connect():
            print("  ✓ MAVLink connection successful")
            
            # Get telemetry
            telemetry = mavlink.get_telemetry()
            if telemetry:
                print(f"  ✓ Telemetry received")
                if 'armed' in telemetry:
                    print(f"     Armed: {telemetry['armed']}")
                if 'mode' in telemetry:
                    print(f"     Mode: {telemetry['mode']}")
            
            mavlink.disconnect()
            return True
        else:
            print("  ❌ MAVLink connection failed")
            return False
            
    except Exception as e:
        print(f"  ❌ MAVLink test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_websocket_connection():
    """Test WebSocket connection to backend."""
    print("\n" + "=" * 60)
    print("[2/3] Testing WebSocket Connection")
    print("=" * 60)
    
    try:
        from drone_client.config import Config
        from drone_client.communication.websocket_client import WebSocketClient
        
        config = Config()
        backend_config = config.get_backend_config()
        
        ws_url = backend_config.get('ws_url', 'wss://drone-command-center.onrender.com/ws?client=drone')
        print(f"  Backend URL: {ws_url}")
        
        websocket = WebSocketClient(backend_config)
        
        print("  Attempting connection...")
        connected = await websocket.connect()
        
        if connected:
            print("  ✓ WebSocket connection successful")
            
            # Wait a moment to ensure connection is stable
            await asyncio.sleep(1)
            
            # Check status
            status = websocket.get_status()
            print(f"  ✓ Connection status: {status['connected']}")
            print(f"  ✓ Registered handlers: {len(status['registered_handlers'])}")
            
            await websocket.disconnect()
            return True
        else:
            print("  ❌ WebSocket connection failed")
            print("  Troubleshooting:")
            print("    1. Check internet connection")
            print("    2. Verify backend URL is correct")
            print("    3. Check if backend is running")
            return False
            
    except Exception as e:
        print(f"  ❌ WebSocket test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

async def test_client_initialization():
    """Test basic client initialization (without starting all loops)."""
    print("\n" + "=" * 60)
    print("[3/3] Testing Client Initialization")
    print("=" * 60)
    
    try:
        from drone_client.config import Config
        from drone_client.controllers.mavlink_controller import MAVLinkController
        from drone_client.sensors.camera import CameraManager
        
        config = Config()
        
        # Test MAVLink initialization
        print("  Initializing MAVLink...")
        mavlink_config = config.get_mavlink_config()
        mavlink = MAVLinkController(mavlink_config)
        
        if mavlink.connect():
            print("  ✓ MAVLink initialized")
            mavlink.disconnect()
        else:
            print("  ⚠️  MAVLink initialization failed (may be OK if FC not connected)")
        
        # Test Camera initialization
        print("  Initializing Camera...")
        camera_config = config.get_camera_config()
        camera = CameraManager(camera_config)
        
        if camera.initialize():
            print("  ✓ Camera initialized")
            camera.cleanup()
        else:
            print("  ⚠️  Camera initialization failed")
        
        print("  ✓ Basic initialization test complete")
        return True
        
    except Exception as e:
        print(f"  ❌ Initialization test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

async def main():
    """Main test function."""
    print("=" * 60)
    print("Drone Client Connection Test")
    print("=" * 60)
    print("\nThis script tests:")
    print("  1. MAVLink connection to flight controller")
    print("  2. WebSocket connection to backend")
    print("  3. Basic client component initialization")
    
    results = {
        'mavlink': False,
        'websocket': False,
        'initialization': False
    }
    
    # Test MAVLink
    results['mavlink'] = await test_mavlink_connection()
    
    # Test WebSocket
    results['websocket'] = await test_websocket_connection()
    
    # Test initialization
    results['initialization'] = await test_client_initialization()
    
    # Summary
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print(f"  MAVLink:        {'✓ PASS' if results['mavlink'] else '✗ FAIL'}")
    print(f"  WebSocket:      {'✓ PASS' if results['websocket'] else '✗ FAIL'}")
    print(f"  Initialization: {'✓ PASS' if results['initialization'] else '✗ FAIL'}")
    
    if all(results.values()):
        print("\n✓ All tests passed! Client should be ready to run.")
        print("\nNext steps:")
        print("  1. Start the client: python3 -m drone_client.main")
        print("  2. Open frontend and test arm command")
        return 0
    else:
        print("\n⚠️  Some tests failed. Fix issues above before running full client.")
        return 1

if __name__ == "__main__":
    try:
        exit_code = asyncio.run(main())
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nFatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

