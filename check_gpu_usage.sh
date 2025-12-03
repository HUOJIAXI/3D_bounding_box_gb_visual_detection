#!/bin/bash
# Script to monitor GPU usage while YOLO is running
# Usage: ./check_gpu_usage.sh

echo "Monitoring GPU usage (Press Ctrl+C to stop)..."
echo "================================================"
watch -n 1 'nvidia-smi --query-gpu=index,name,utilization.gpu,memory.used,memory.total --format=csv'
