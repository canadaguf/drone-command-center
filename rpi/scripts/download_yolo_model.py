#!/usr/bin/env python3
"""
Download and setup YOLO11n ONNX model for person detection.
"""

import os
import sys
import urllib.request
from pathlib import Path

# Model URLs (YOLO11n ONNX format)
MODEL_URLS = {
    'onnx': 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.onnx',
    'pt': 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt',
}

def download_file(url: str, destination: Path, description: str) -> bool:
    """Download a file from URL.
    
    Args:
        url: URL to download from
        destination: Destination file path
        description: Description of what's being downloaded
        
    Returns:
        True if successful, False otherwise
    """
    try:
        print(f"\nDownloading {description}...")
        print(f"  URL: {url}")
        print(f"  Destination: {destination}")
        
        # Create parent directory if needed
        destination.parent.mkdir(parents=True, exist_ok=True)
        
        # Download with progress
        def show_progress(block_num, block_size, total_size):
            downloaded = block_num * block_size
            percent = min(100, (downloaded * 100) / total_size) if total_size > 0 else 0
            bar_length = 40
            filled = int(bar_length * percent / 100)
            bar = '=' * filled + '-' * (bar_length - filled)
            print(f"\r  [{bar}] {percent:.1f}%", end='', flush=True)
        
        urllib.request.urlretrieve(url, destination, show_progress)
        print()  # New line after progress
        
        # Verify file exists and has content
        if destination.exists() and destination.stat().st_size > 0:
            size_mb = destination.stat().st_size / (1024 * 1024)
            print(f"  ✓ Downloaded successfully ({size_mb:.2f} MB)")
            return True
        else:
            print(f"  ✗ Download failed - file is empty or missing")
            return False
            
    except Exception as e:
        print(f"  ✗ Download failed: {e}")
        return False

def convert_pt_to_onnx(pt_path: Path, onnx_path: Path) -> bool:
    """Convert PyTorch model to ONNX format.
    
    Args:
        pt_path: Path to .pt file
        onnx_path: Path to save .onnx file
        
    Returns:
        True if successful, False otherwise
    """
    try:
        print(f"\nConverting PyTorch model to ONNX...")
        
        # Try to import ultralytics
        try:
            from ultralytics import YOLO
        except ImportError:
            print("  ✗ ultralytics not installed")
            print("  Install with: pip install ultralytics")
            return False
        
        # Load model
        print(f"  Loading model from {pt_path}...")
        model = YOLO(str(pt_path))
        
        # Export to ONNX
        print(f"  Exporting to ONNX...")
        model.export(format='onnx', imgsz=320, simplify=True)
        
        # Find the exported file (usually same name with .onnx extension)
        exported_path = pt_path.with_suffix('.onnx')
        if exported_path.exists():
            # Move to desired location
            exported_path.rename(onnx_path)
            print(f"  ✓ Converted successfully")
            return True
        else:
            print(f"  ✗ Conversion failed - output file not found")
            return False
            
    except Exception as e:
        print(f"  ✗ Conversion failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main function."""
    print("=" * 60)
    print("YOLO11n Model Setup")
    print("=" * 60)
    
    # Determine model directory
    models_dir = Path.home() / 'models'
    if 'MODELS_DIR' in os.environ:
        models_dir = Path(os.environ['MODELS_DIR'])
    
    print(f"\nModel directory: {models_dir}")
    
    # Create directory
    models_dir.mkdir(parents=True, exist_ok=True)
    
    # Check what's already there
    onnx_path = models_dir / 'yolo11n.onnx'
    pt_path = models_dir / 'yolo11n.pt'
    
    if onnx_path.exists():
        size_mb = onnx_path.stat().st_size / (1024 * 1024)
        print(f"\n✓ ONNX model already exists: {onnx_path} ({size_mb:.2f} MB)")
        print("  Model is ready to use!")
        return 0
    
    # Try to download ONNX directly
    print("\n[Option 1] Downloading ONNX model directly...")
    if download_file(MODEL_URLS['onnx'], onnx_path, 'YOLO11n ONNX model'):
        print("\n✓ Model setup complete!")
        print(f"  Model location: {onnx_path}")
        print(f"  Update config to use: {onnx_path}")
        return 0
    
    # If ONNX download fails, try PyTorch + conversion
    print("\n[Option 2] Downloading PyTorch model and converting...")
    if not pt_path.exists():
        if not download_file(MODEL_URLS['pt'], pt_path, 'YOLO11n PyTorch model'):
            print("\n✗ Failed to download model")
            return 1
    
    # Convert to ONNX
    if convert_pt_to_onnx(pt_path, onnx_path):
        print("\n✓ Model setup complete!")
        print(f"  Model location: {onnx_path}")
        print(f"  Update config to use: {onnx_path}")
        
        # Optionally remove .pt file to save space
        try:
            pt_path.unlink()
            print(f"  Removed temporary .pt file")
        except:
            pass
        
        return 0
    else:
        print("\n✗ Failed to convert model")
        print(f"  You can manually convert using:")
        print(f"    from ultralytics import YOLO")
        print(f"    model = YOLO('{pt_path}')")
        print(f"    model.export(format='onnx', imgsz=320)")
        return 1

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n\nDownload interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nFatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

