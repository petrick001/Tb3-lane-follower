# tb3_lane_follower — Robotics Project, Part I

Camera-only autonomous lane following for a TurtleBot3 (ROS2 Humble, Ubuntu 22.04).
Yellow line on the LEFT, white line on the RIGHT; the robot drives between them.
All image processing runs on the remote PC (not the Pi).

```
Pi camera --/camera/image_raw/compressed--> [lane_follower node on PC] --/cmd_vel--> TurtleBot3
```

## How it works
1. `lane_detector.py` – brightness normalisation, HSV thresholds for yellow/white below the horizon,
   glare-blob removal, largest connected piece per colour, lane centre in a band near the bottom.
2. `controller.py` – PD steering on the lane-centre error; slows down in corrections.
3. `lane_follower_node.py` – ROS2 wiring, watchdog (stops if no images / lane lost), debug image.

## Run on the school PC
Robot side (Pi, **one** bringup only, then the camera node as usual, format BGR888):
```bash
ros2 launch turtlebot3_bringup robot.launch.py
```
PC side:
```bash
mkdir -p ~/robotics_ws/src && cd ~/robotics_ws/src
git clone <YOUR_REPO_URL> tb3_lane_follower
cd ~/robotics_ws && colcon build --packages-select tb3_lane_follower
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=30
```
1. Detect only (robot cannot move) and look at the result:
   `ros2 launch tb3_lane_follower lane_follower.launch.py enabled:=false`
   then in another terminal `rqt_image_view` -> `/lane_follower/debug_image`
2. Wheels off the ground, then drive: `ros2 launch tb3_lane_follower lane_follower.launch.py`
3. On the track at low speed. Stop any time with Ctrl+C (the node sends a zero command on exit).

If your camera has no compressed topic: set `use_compressed: false` and `image_topic: /camera/image_raw`
in `config/lane_follower.yaml`.

## Live tuning (no rebuild)
```bash
ros2 param set /lane_follower kp 1.5        # steering gain
ros2 param set /lane_follower v_max 0.15    # top speed (m/s)
ros2 param set /lane_follower white_v_min 150
```
| Symptom | Change |
|---|---|
| Turns the wrong way | check `yellow_on_left`; sign of the error is tested in `test/test_logic.py` |
| Oscillates left-right | lower `kp`, raise `kd`, raise `smooth` toward 0.3 |
| Reacts too slowly in curves | raise `kp`, lower `v_max` |
| Misses the white line | lower `white_v_min`; check the debug image |
| Sees white where there is none (windows, tiles) | raise `white_v_min`, raise `roi_top` |
| Drives off when only one line is visible | tune `half_lane` (half lane width / image width) |

## Test without a robot
```bash
python3 tools/test_on_dataset.py /path/to/turtlebot3_autorace_dataset .   # stats + montage.jpg
python3 -m pytest test/ -q
```
On the course dataset (334 images, lights on/off, floor/table, both directions) the detector finds
both lines in 278 images, one line in 53, and none in 3.

## Acknowledgements
Written with the help of an AI assistant (Claude, Anthropic). Detector thresholds were checked on the
course dataset provided by J. Rodriguez.
