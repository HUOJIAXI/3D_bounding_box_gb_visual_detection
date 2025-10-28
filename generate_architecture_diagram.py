#!/usr/bin/env python3
"""
Generate System Architecture Diagram for 3D Bounding Box Detection
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import matplotlib.patches as patches

# Create figure
fig, ax = plt.subplots(1, 1, figsize=(12, 10))
ax.set_xlim(0, 10)
ax.set_ylim(0, 12)
ax.axis('off')

# Define colors
color_input = '#E3F2FD'      # Light blue
color_2d = '#C8E6C9'          # Light green
color_3d = '#FFE082'          # Light yellow
color_output = '#FFCCBC'      # Light orange
color_arrow = '#424242'       # Dark gray

# Title
ax.text(5, 11.5, '3D Bounding Box Detection System Architecture',
        fontsize=20, fontweight='bold', ha='center')

# ============ Layer 1: Input (Camera) ============
camera_box = FancyBboxPatch((3.5, 9.5), 3, 1,
                            boxstyle="round,pad=0.1",
                            edgecolor='#1976D2', facecolor=color_input, linewidth=2)
ax.add_patch(camera_box)
ax.text(5, 10, 'RGB-D Camera\n(Orbbec Astra Pro)',
        fontsize=11, ha='center', va='center', fontweight='bold')

# Arrow from camera splits
ax.annotate('', xy=(4, 8.8), xytext=(4, 9.5),
            arrowprops=dict(arrowstyle='->', lw=2, color=color_arrow))
ax.annotate('', xy=(6, 8.8), xytext=(6, 9.5),
            arrowprops=dict(arrowstyle='->', lw=2, color=color_arrow))

# ============ Layer 2: Split Data ============
# RGB Image
rgb_box = FancyBboxPatch((0.5, 8), 3, 0.8,
                         boxstyle="round,pad=0.05",
                         edgecolor='#1976D2', facecolor=color_input, linewidth=1.5)
ax.add_patch(rgb_box)
ax.text(2, 8.4, 'RGB Image\n/camera/color/image_raw',
        fontsize=9, ha='center', va='center')

# Point Cloud
pc_box = FancyBboxPatch((6.5, 8), 3, 0.8,
                        boxstyle="round,pad=0.05",
                        edgecolor='#1976D2', facecolor=color_input, linewidth=1.5)
ax.add_patch(pc_box)
ax.text(8, 8.4, 'Point Cloud (Depth)\n/camera/depth/points',
        fontsize=9, ha='center', va='center')

# Arrow from RGB to darknet_ros
ax.annotate('', xy=(2, 6.7), xytext=(2, 8),
            arrowprops=dict(arrowstyle='->', lw=2, color=color_arrow))

# ============ Layer 3: 2D Detection ============
darknet_box = FancyBboxPatch((0.5, 5.5), 3, 1.2,
                             boxstyle="round,pad=0.1",
                             edgecolor='#388E3C', facecolor=color_2d, linewidth=2)
ax.add_patch(darknet_box)
ax.text(2, 6.4, 'darknet_ros', fontsize=12, ha='center', va='center', fontweight='bold')
ax.text(2, 6.0, 'YOLO Object Detection', fontsize=9, ha='center', va='center')
ax.text(2, 5.7, '(2D Bounding Boxes)', fontsize=8, ha='center', va='center', style='italic')

# Arrow from darknet_ros to 2D boxes
ax.annotate('', xy=(2, 4.6), xytext=(2, 5.5),
            arrowprops=dict(arrowstyle='->', lw=2, color=color_arrow))

# ============ Layer 4: 2D Bounding Boxes ============
bbox2d_box = FancyBboxPatch((0.5, 4), 3, 0.6,
                            boxstyle="round,pad=0.05",
                            edgecolor='#388E3C', facecolor=color_2d, linewidth=1.5)
ax.add_patch(bbox2d_box)
ax.text(2, 4.3, '2D Bounding Boxes\n(xmin, ymin, xmax, ymax)',
        fontsize=9, ha='center', va='center')

# Arrows converging to darknet_ros_3d
# From 2D boxes
ax.annotate('', xy=(4, 2.4), xytext=(2.5, 4),
            arrowprops=dict(arrowstyle='->', lw=2, color=color_arrow))

# From Point Cloud (stored, then used)
ax.annotate('', xy=(6, 2.4), xytext=(8, 8),
            arrowprops=dict(arrowstyle='->', lw=2, color=color_arrow, linestyle='dashed'))
ax.text(7.3, 5.2, 'Depth\nData', fontsize=8, ha='center',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=color_arrow))

# ============ Layer 5: 3D Processing ============
darknet3d_box = FancyBboxPatch((3, 1.2), 4, 1.2,
                               boxstyle="round,pad=0.1",
                               edgecolor='#F57C00', facecolor=color_3d, linewidth=2)
ax.add_patch(darknet3d_box)
ax.text(5, 2.1, 'darknet_ros_3d', fontsize=12, ha='center', va='center', fontweight='bold')
ax.text(5, 1.7, '3D Projection Algorithm', fontsize=9, ha='center', va='center')
ax.text(5, 1.4, 'Map 2D pixels → 3D coordinates', fontsize=8, ha='center', va='center', style='italic')

# Arrow from darknet_ros_3d to output
ax.annotate('', xy=(5, 0.6), xytext=(5, 1.2),
            arrowprops=dict(arrowstyle='->', lw=2, color=color_arrow))

# ============ Layer 6: 3D Bounding Boxes Output ============
bbox3d_box = FancyBboxPatch((3, 0), 4, 0.6,
                            boxstyle="round,pad=0.05",
                            edgecolor='#D84315', facecolor=color_output, linewidth=1.5)
ax.add_patch(bbox3d_box)
ax.text(5, 0.3, '3D Bounding Boxes\n(xmin, xmax, ymin, ymax, zmin, zmax) in meters',
        fontsize=9, ha='center', va='center', fontweight='bold')

# ============ Add annotations for key steps ============
# Step 1 - Top right of camera
ax.text(6.8, 10.2, '①', fontsize=14, ha='center', va='center',
        bbox=dict(boxstyle='circle', facecolor='yellow', edgecolor='red', linewidth=2))

# Step 2 - Left of darknet_ros
ax.text(-0.2, 6.1, '②', fontsize=14, ha='center', va='center',
        bbox=dict(boxstyle='circle', facecolor='yellow', edgecolor='red', linewidth=2))

# Step 3 - Left of darknet_ros_3d
ax.text(2.3, 1.8, '③', fontsize=14, ha='center', va='center',
        bbox=dict(boxstyle='circle', facecolor='yellow', edgecolor='red', linewidth=2))

# Step 4 - Right of output
ax.text(7.5, 0.3, '④', fontsize=14, ha='center', va='center',
        bbox=dict(boxstyle='circle', facecolor='yellow', edgecolor='red', linewidth=2))

# ============ Add legend/key information ============
legend_y = 10.8
ax.text(0.3, legend_y, 'Pipeline Steps:', fontsize=10, fontweight='bold')
ax.text(0.3, legend_y-0.3, '① Capture RGB-D data', fontsize=8)
ax.text(0.3, legend_y-0.6, '② Detect objects in 2D (YOLO)', fontsize=8)
ax.text(0.3, legend_y-0.9, '③ Project 2D boxes to 3D space', fontsize=8)
ax.text(0.3, legend_y-1.2, '④ Output 3D bounding boxes', fontsize=8)

# ============ Add ROS topics information ============
topic_x = 9.7
ax.text(topic_x, 10.8, 'ROS 2 Topics:', fontsize=10, fontweight='bold', ha='right')
ax.text(topic_x, 10.5, '/camera/color/image_raw', fontsize=7, ha='right', family='monospace',
        bbox=dict(boxstyle='round,pad=0.2', facecolor=color_input, alpha=0.7))
ax.text(topic_x, 10.2, '/camera/depth/points', fontsize=7, ha='right', family='monospace',
        bbox=dict(boxstyle='round,pad=0.2', facecolor=color_input, alpha=0.7))
ax.text(topic_x, 9.9, '/darknet_ros/bounding_boxes', fontsize=7, ha='right', family='monospace',
        bbox=dict(boxstyle='round,pad=0.2', facecolor=color_2d, alpha=0.7))
ax.text(topic_x, 9.6, '/darknet_ros_3d/bounding_boxes', fontsize=7, ha='right', family='monospace',
        bbox=dict(boxstyle='round,pad=0.2', facecolor=color_output, alpha=0.7))

# Add frame border
border = mpatches.Rectangle((0.1, 0.1), 9.8, 11.7, fill=False,
                            edgecolor='black', linewidth=1, linestyle='-')
ax.add_patch(border)

# Add footer
ax.text(5, -0.3, '© HUOJIAXI - 3D Bounding Box Detection System',
        fontsize=8, ha='center', style='italic', color='gray')

plt.tight_layout()
plt.savefig('system_architecture.png', dpi=300, bbox_inches='tight', facecolor='white')
print("Architecture diagram saved as: system_architecture.png")
plt.close()

# ============ Generate a second diagram showing coordinate system ============
fig2, ax2 = plt.subplots(1, 1, figsize=(10, 8))
ax2.set_xlim(-2, 8)
ax2.set_ylim(-1, 7)
ax2.axis('off')

# Title
ax2.text(3, 6.5, '3D Coordinate System and Detection Example',
         fontsize=16, fontweight='bold', ha='center')

# Draw camera
camera_patch = mpatches.FancyBboxPatch((2.5, 5), 1, 0.5,
                                       boxstyle="round,pad=0.05",
                                       edgecolor='blue', facecolor='lightblue', linewidth=2)
ax2.add_patch(camera_patch)
ax2.text(3, 5.25, 'Camera', fontsize=9, ha='center', va='center', fontweight='bold')

# Draw coordinate axes
origin = [3, 5]
# X-axis (forward/depth) - Red
ax2.arrow(origin[0], origin[1], 3, 0, head_width=0.15, head_length=0.2,
          fc='red', ec='red', linewidth=2)
ax2.text(6.5, 5, 'X (Depth)', fontsize=11, va='center', fontweight='bold', color='red')

# Y-axis (horizontal/left-right) - Green
ax2.arrow(origin[0], origin[1], 0, -2, head_width=0.15, head_length=0.2,
          fc='green', ec='green', linewidth=2)
ax2.text(3, 2.5, 'Y (Left-Right)', fontsize=11, ha='center', fontweight='bold', color='green')

# Z-axis (vertical/up-down) - Blue (using perspective)
ax2.arrow(origin[0], origin[1], -1, 1.5, head_width=0.15, head_length=0.2,
          fc='blue', ec='blue', linewidth=2)
ax2.text(1.5, 6.7, 'Z (Height)', fontsize=11, fontweight='bold', color='blue')

# Draw detected person (3D bounding box)
# Front face
person_x = 4.5
person_y = 3.5
person_w = 0.8
person_h = 1.5

person_front = mpatches.Rectangle((person_x, person_y), person_w, person_h,
                                  linewidth=2, edgecolor='orange', facecolor='orange', alpha=0.3)
ax2.add_patch(person_front)

# Back face (perspective)
offset_x = -0.3
offset_y = 0.3
person_back = mpatches.Rectangle((person_x+offset_x, person_y+offset_y), person_w, person_h,
                                 linewidth=2, edgecolor='orange', facecolor='orange', alpha=0.15, linestyle='--')
ax2.add_patch(person_back)

# Connect corners
ax2.plot([person_x, person_x+offset_x], [person_y, person_y+offset_y], 'orange', linewidth=1.5)
ax2.plot([person_x+person_w, person_x+person_w+offset_x], [person_y, person_y+offset_y], 'orange', linewidth=1.5)
ax2.plot([person_x, person_x+offset_x], [person_y+person_h, person_y+person_h+offset_y], 'orange', linewidth=1.5)
ax2.plot([person_x+person_w, person_x+person_w+offset_x], [person_y+person_h, person_y+person_h+offset_y], 'orange', linewidth=1.5)

ax2.text(person_x+person_w/2, person_y+person_h/2, 'Person', fontsize=10,
         ha='center', va='center', fontweight='bold', color='darkred')

# Add dimension labels
ax2.text(person_x+person_w+0.3, person_y+person_h/2, f'ΔZ = {person_h:.1f}m\n(Height)',
         fontsize=8, va='center', bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow'))

ax2.text(person_x+person_w/2, person_y-0.3, f'ΔY = {person_w:.1f}m\n(Width)',
         fontsize=8, ha='center', bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow'))

# Distance from camera
ax2.plot([origin[0], person_x+person_w/2], [origin[1], person_y+person_h/2],
         'k--', linewidth=1, alpha=0.5)
ax2.text(3.8, 4.5, '1.8m', fontsize=9, fontweight='bold',
         bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='black'))

# Add output box
output_text = "3D Output:\nObject: person\nProbability: 0.94\nX: 1.5 - 2.1m\nY: -0.4 - 0.4m\nZ: 0.0 - 1.5m"
ax2.text(0.5, 1.5, output_text, fontsize=9, family='monospace',
         bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', edgecolor='darkgreen', linewidth=2),
         verticalalignment='top')

# Add grid for reference
ax2.grid(True, alpha=0.2, linestyle=':', linewidth=0.5)

plt.tight_layout()
plt.savefig('coordinate_system.png', dpi=300, bbox_inches='tight', facecolor='white')
print("Coordinate system diagram saved as: coordinate_system.png")
plt.close()

print("\nDiagrams generated successfully!")
print("- system_architecture.png: Complete pipeline architecture")
print("- coordinate_system.png: 3D coordinate system explanation")
