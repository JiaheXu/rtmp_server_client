# MKV and telemetry publisher and mcap recorder

## Dependencies

```bash
sudo apt install ros-humble-ros2bag ros-humble-rosbag2-storage-mcap ros-humble-cv-bridge
```

## How to Run

* Modify config.yaml with the video file path and subtitles path (optional) and time delay if the video feed is delayed

### Option 1
  
* Run `launch.sh` which handles publishing the telemetry and video feed and mcap recording automatically.
  * This option may hide problems and errors...

### Option 2

* Run `video_telemetry_publisher.py` in one terminal window
* Run `ros2_bag_recorder.py` in another terminal window

* When you see: "End of video, shutting down.". Ctrl+c to finish.

### Option 3

* Publish an mkv video file only to a ros2 topic. Nothing else.
`mkv_publisher_standalone.py`
