# RGB Dataset Collector Design

## Goal

Provide a minimal ROS/PyQt panel for collecting RealSense RGB source frames for carton segmentation fine-tuning.

## Scope

- Start only the existing camera node and the collector panel.
- Do not start robot arm, serial, MoveIt, tactile, grasp, or torque nodes.
- Subscribe to `/supervisor/camera/color/image_raw` by default.
- Display the live RGB image stream.
- Save the latest received RGB frame when the user clicks one of three buttons.

## Categories

- `positive`: button text `有纸盒`, path `~/carton_dataset/raw_rgb/positive`
- `negative`: button text `无纸盒`, path `~/carton_dataset/raw_rgb/negative`
- `low_sample`: button text `低样本`, path `~/carton_dataset/raw_rgb/low_sample`

## File Naming

Each category uses its own six-digit sequence number. On startup, the collector scans existing PNG files in each category directory and continues from the largest existing numeric stem.

Examples:

- `~/carton_dataset/raw_rgb/positive/000001.png`
- `~/carton_dataset/raw_rgb/negative/000001.png`
- `~/carton_dataset/raw_rgb/low_sample/000001.png`

## Architecture

Add an executable script `src/alicia_flexible_grasp_supervisor/scripts/rgb_dataset_collector_gui.py`. It initializes a ROS node, subscribes to the RGB image topic through `cv_bridge`, keeps a thread-safe copy of the latest BGR frame, renders an RGB preview in PyQt, and saves the BGR frame with `cv2.imwrite`.

Add `src/alicia_flexible_grasp_supervisor/launch/rgb_dataset_collector.launch`. It loads `config/camera.yaml`, starts `camera_node.py`, and starts the collector GUI with configurable `color_topic` and `output_root` parameters.

## Error Handling

- If no frame has arrived, button clicks show `尚未收到 RGB 图像`.
- If image conversion fails, the GUI status shows the conversion error and the ROS node keeps running.
- If saving fails, the GUI status shows the target path and exception.
- Directories are created automatically.

## Validation

- Unit tests cover category path mapping and sequence-number continuation from existing files.
- Syntax compilation covers the collector script.
- Runtime launch is validated by starting the launch file and checking that ROS nodes stay alive long enough for the panel to appear.
