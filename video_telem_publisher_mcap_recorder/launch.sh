#!/bin/bash

# Activate ROS environment
source /opt/ros/humble/setup.bash

# Start ros2 bag recording in background
python3 ros2_bag_recorder.py &
BAG_RECORDER_PID=$!

# Start video and telemetry publisher
python3 video_telemetry_publisher.py
PUBLISHER_EXIT_CODE=$?

# After publisher finishes, terminate bag recorder
echo "Terminating ros2 bag recorder..."
kill $BAG_RECORDER_PID

# Wait for bag recorder to finish terminating
wait $BAG_RECORDER_PID

echo "Both processes have completed."

# Exit with publisher's exit code
exit $PUBLISHER_EXIT_CODE
