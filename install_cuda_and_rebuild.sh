#!/bin/bash
# Script to install CUDA toolkit and rebuild darknet_ros with GPU support

set -e

echo "=========================================="
echo "Installing CUDA Toolkit for GPU Support"
echo "=========================================="

# Check if CUDA is already installed
if command -v nvcc &> /dev/null; then
    echo "CUDA toolkit is already installed!"
    nvcc --version
else
    echo "Installing CUDA toolkit..."
    sudo apt update
    sudo apt install -y nvidia-cuda-toolkit
    
    # Verify installation
    if command -v nvcc &> /dev/null; then
        echo "CUDA toolkit installed successfully!"
        nvcc --version
    else
        echo "ERROR: CUDA toolkit installation failed!"
        exit 1
    fi
fi

echo ""
echo "=========================================="
echo "Rebuilding darknet_ros with GPU support"
echo "=========================================="

# Navigate to workspace
cd "$(dirname "$0")"

# Clean build to ensure fresh GPU build
echo "Cleaning previous build..."
rm -rf build/darknet_ros install/darknet_ros

# Rebuild with GPU support
echo "Building darknet_ros with GPU support..."
colcon build --packages-select darknet_ros

echo ""
echo "=========================================="
echo "Verifying GPU support"
echo "=========================================="

# Check if CUDA files were compiled
CUDA_OBJ_COUNT=$(find build/darknet_ros -name "*.cu.o" 2>/dev/null | wc -l)
if [ "$CUDA_OBJ_COUNT" -gt 0 ]; then
    echo "✓ GPU support enabled! Found $CUDA_OBJ_COUNT CUDA object files."
else
    echo "✗ WARNING: No CUDA object files found. GPU support may not be enabled."
fi

# Check build log for GPU messages
if grep -qi "Building with GPU support" build/darknet_ros/CMakeCache.txt 2>/dev/null || \
   grep -qi "CUDA Version" log/latest_build/darknet_ros/stdout_stderr.log 2>/dev/null; then
    echo "✓ Build log confirms GPU support is enabled."
else
    echo "⚠ Could not verify GPU support from build logs."
fi

echo ""
echo "=========================================="
echo "Done! You can now run darknet_ros with GPU acceleration."
echo "=========================================="
