# Ultralytics ROS2 - YOLOv8 & YOLOv11 Integration

ROS2 wrapper for Ultralytics YOLO (YOLOv8 and YOLOv11) object detection, providing improved detection performance over YOLOv2/v3 in darknet_ros.

## Features

- **YOLOv8 Support**: Nano, Small, Medium, Large, and Extra Large models
- **YOLOv11 Support**: Latest generation YOLO models with improved performance
- **ROS2 Native**: Built for ROS2 with modern Python support
- **Compatible Topics**: Publishes to same topics as darknet_ros for easy integration
- **GPU Accelerated**: CUDA support for fast inference
- **Flexible Configuration**: Easy-to-use YAML config files
- **Real-time Visualization**: Optional detection image publishing

## Performance Comparison

| Model | Size | mAP | Speed (FPS) | Best For |
|-------|------|-----|-------------|----------|
| YOLOv2 | 194MB | 44.0 | 40 | Legacy |
| YOLOv3 | 235MB | 55.3 | 30 | Baseline |
| **YOLOv8n** | 6.2MB | 37.3 | 80+ | Speed |
| **YOLOv8s** | 22MB | 44.9 | 60+ | Balance |
| **YOLOv8m** | 52MB | 50.2 | 40+ | Recommended |
| **YOLOv8l** | 87MB | 52.9 | 25+ | Accuracy |
| **YOLOv8x** | 136MB | 53.9 | 20+ | Best Accuracy |
| **YOLOv11m** | 40MB | 51.5 | 45+ | Latest Gen |

## Installation

### 1. Install Python Dependencies

```bash
cd /home/nvidia/gb_ws/src/ultralytics_ros
pip3 install -r requirements.txt
```

Or for Jetson devices (with PyTorch pre-installed):

```bash
pip3 install ultralytics opencv-python
```

### 2. Build ROS2 Package

```bash
cd /home/nvidia/gb_ws
colcon build --packages-select ultralytics_ros
source install/setup.bash
```

## Usage

### Quick Start - YOLOv8 Nano (Fastest)

```bash
ros2 launch ultralytics_ros yolov8n.launch.py
```

### YOLOv8 Medium (Recommended)

```bash
ros2 launch ultralytics_ros yolov8m.launch.py
```

### YOLOv11 Medium (Latest Generation)

```bash
ros2 launch ultralytics_ros yolov11m.launch.py
```

### Custom Configuration

```bash
ros2 launch ultralytics_ros yolo_detector.launch.py \
  config_file:=yolov8m.yaml \
  image_topic:=/camera/color/image_raw \
  device:=cuda:0
```

### Run on CPU (No GPU)

```bash
ros2 launch ultralytics_ros yolov8n.launch.py device:=cpu
```

## Published Topics

Compatible with darknet_ros message types:

| Topic | Type | Description |
|-------|------|-------------|
| `/darknet_ros/bounding_boxes` | `darknet_ros_msgs/BoundingBoxes` | Detection bounding boxes |
| `/darknet_ros/found_object` | `darknet_ros_msgs/ObjectCount` | Object count |
| `/darknet_ros/detection_image` | `sensor_msgs/Image` | Annotated image |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Input camera image |

## Configuration

### Available Models

- **YOLOv8**: `yolov8n.yaml`, `yolov8s.yaml`, `yolov8m.yaml`, `yolov8l.yaml`, `yolov8x.yaml`
- **YOLOv11**: `yolov11n.yaml`, `yolov11s.yaml`, `yolov11m.yaml`, `yolov11l.yaml`, `yolov11x.yaml`

### Configuration Parameters

Edit YAML files in `config/` directory:

```yaml
yolo_detector_node:
  ros__parameters:
    model_path: 'yolov8m.pt'           # Model file (.pt format)
    model_type: 'yolov8'               # yolov8 or yolov11
    device: 'cuda:0'                   # cuda:0, cuda:1, or cpu
    confidence_threshold: 0.25         # Detection confidence (0.0-1.0)
    iou_threshold: 0.45                # NMS IoU threshold
    input_topic: '/camera/color/image_raw'
    max_det: 300                       # Maximum detections per image
    enable_visualization: true         # Draw bounding boxes
    publish_image: true                # Publish annotated image
    image_size: 640                    # Input size for YOLO
```

## Model Download

Models are automatically downloaded on first run:
- YOLOv8 models: ~6MB to 136MB
- YOLOv11 models: ~5MB to 100MB

Models are cached in `~/.cache/ultralytics/`

## Performance Tips

### For Jetson Devices

1. **Use smaller models** on Jetson Nano/TX2 (yolov8n, yolov8s)
2. **Use medium models** on Jetson Orin (yolov8m, yolov11m)
3. **Enable TensorRT** for faster inference:

```python
# In yolo_detector_node.py, add after model loading:
self.model.export(format='engine')  # Export to TensorRT
```

### For Desktop GPUs

- Use yolov8l or yolov8x for best accuracy
- Increase `image_size` to 1280 for better small object detection

### CPU-Only Systems

- Use yolov8n for real-time performance
- Reduce `image_size` to 416 or 320

## Comparison with darknet_ros

### Advantages

✅ **Better Performance**: 10-20% higher mAP compared to YOLOv3
✅ **Faster Inference**: 2-3x faster on same hardware
✅ **Smaller Models**: YOLOv8n is 30x smaller than YOLOv3
✅ **Active Development**: Regular updates from Ultralytics
✅ **Modern Architecture**: PyTorch-based, easier to customize
✅ **Multiple Variants**: 5 model sizes for different use cases

### Compatibility

✅ **Same Topics**: Drop-in replacement for darknet_ros
✅ **Same Messages**: Uses darknet_ros_msgs
✅ **Same Classes**: COCO 80 classes compatible

## Troubleshooting

### Model Download Fails

```bash
# Manually download models
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8m.pt
mv yolov8m.pt ~/.cache/ultralytics/
```

### CUDA Out of Memory

- Use smaller model (yolov8n or yolov8s)
- Reduce `image_size` parameter
- Reduce `max_det` parameter

### Low FPS

- Check GPU usage: `nvidia-smi`
- Use smaller model
- Reduce input resolution
- Disable visualization: `enable_visualization: false`

## Development

### Project Structure

```
ultralytics_ros/
├── config/                    # YAML configuration files
├── launch/                    # Launch files
├── ultralytics_ros/          # Python package
│   ├── __init__.py
│   └── yolo_detector_node.py # Main detector node
├── CMakeLists.txt
├── package.xml
├── setup.py
├── requirements.txt
└── README.md
```

### Dependencies

- ROS2 (Humble or newer)
- Python 3.8+
- PyTorch 2.0+
- Ultralytics 8.0+
- OpenCV 4.5+
- CUDA 11.0+ (optional, for GPU)

## License

MIT License

## Credits

- **Ultralytics**: https://github.com/ultralytics/ultralytics
- **darknet_ros**: https://github.com/leggedrobotics/darknet_ros
- **YOLO**: Joseph Redmon & Ali Farhadi

## References

- YOLOv8 Paper: https://docs.ultralytics.com
- YOLOv11 Release: https://github.com/ultralytics/ultralytics
- darknet_ros: https://github.com/leggedrobotics/darknet_ros
