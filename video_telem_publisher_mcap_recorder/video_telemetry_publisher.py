import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import os
import yaml
import sys

class VideoAndTelemetryPublisher(Node):
    def __init__(self, config):
        super().__init__('video_and_telemetry_publisher')

        # Load configuration parameters
        self.video_path = config['video_file_path']
        self.subtitle_path = config.get('subtitle_file_path', None)
        self.telemetry_offset = config.get('telemetry_offset_seconds', 0)
        self.drone_id = config.get('drone_id', 'drone1')

        # Create publishers for video frames and telemetry data
        self.video_pub = self.create_publisher(Image, f'{self.drone_id}/camera', 10)
        self.bridge = CvBridge()

        # Only create telemetry publishers if telemetry data is available
        self.telemetry_available = False
        if self.subtitle_path and os.path.isfile(self.subtitle_path):
            self.telemetry_available = True
            self.gps_pub = self.create_publisher(NavSatFix, f'{self.drone_id}/gps', 10)
            self.groundspeed_pub = self.create_publisher(Float64, f'{self.drone_id}/groundspeed', 10)
            self.climb_rate_pub = self.create_publisher(Float64, f'{self.drone_id}/climb_rate', 10)
        else:
            self.get_logger().warn("Telemetry subtitle file not found or not provided. Continuing without telemetry data.")
            self.telemetry_data = []
            self.telemetry_index = 0

        # Open the MKV video file using OpenCV
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open the video file: {self.video_path}")
            sys.exit(1)  # Exit with an error code
            return

        # Retrieve video properties: frame rate (FPS) and total number of frames
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.frame_count = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        self.get_logger().info(f"Video Frame Rate: {self.fps} FPS")
        self.get_logger().info(f"Total Frames in Video: {self.frame_count}")

        # Parse .ass subtitle file and store telemetry data if available
        if self.telemetry_available:
            self.telemetry_data = self.parse_ass_file(self.subtitle_path, self.telemetry_offset)
            self.telemetry_index = 0

        # Timer to publish video frames and telemetry data based on video FPS
        frame_interval = 1.0 / self.fps if self.fps > 0 else 0.033  # Default to 30 FPS if FPS is zero
        self.timer = self.create_timer(frame_interval, self.publish_data)

        self.frame_index = 0

    def parse_ass_file(self, subtitle_path, telemetry_offset):
        import re
        from collections import defaultdict

        # Read .ass file and extract Dialogue lines
        try:
            with open(subtitle_path, 'r') as file:
                lines = file.readlines()
        except FileNotFoundError:
            self.get_logger().warn(f"Subtitle file not found: {subtitle_path}. Telemetry data will not be available.")
            return []

        dialogue_entries = []
        for line in lines:
            if line.startswith("Dialogue:"):
                # Extract fields
                fields = line.strip().split(',', 9)  # Split into at most 10 fields to preserve text
                if len(fields) >= 10:
                    start_time = fields[1].strip()
                    text = fields[9].strip()
                    dialogue_entries.append((start_time, text))

        # Group dialogues by start_time
        grouped_dialogues = defaultdict(list)
        for start_time, text in dialogue_entries:
            grouped_dialogues[start_time].append(text)

        # Process grouped dialogues
        telemetry_data = []
        for start_time, texts in grouped_dialogues.items():
            # Parse start time to seconds
            time_parts = start_time.split(':')
            start_seconds = float(time_parts[0]) * 3600 + float(time_parts[1]) * 60 + float(time_parts[2])
            # Apply offset to telemetry time
            adjusted_start_time = start_seconds + telemetry_offset

            # Init vars for labels and values
            labels = []
            values = []

            # Process texts to separate labels and values
            for text in texts:
                # Remove any styling commands (e.g., {\an3\pos(295,1075)})
                text_clean = re.sub(r'{.*?}', '', text).strip()
                # Split text into lines
                lines = text_clean.split('\\N')

                # Determine if this is a label line or a value line
                if all(':' in line for line in lines):
                    # It's a label line
                    labels.extend([line.strip().strip(':') for line in lines])
                else:
                    # It's a value line
                    values.extend([line.strip() for line in lines])

            # Create a dictionary mapping labels to values
            telemetry_dict = {}
            for label, value in zip(labels, values):
                telemetry_dict[label] = value

            # Add to telemetry_data
            telemetry_data.append((adjusted_start_time, telemetry_dict))

        # Sort telemetry data by adjusted_start_time
        telemetry_data.sort(key=lambda x: x[0])

        return telemetry_data

    def publish_data(self):
        # Check if we've reached the end of the video based on frame count
        if self.frame_index >= self.frame_count:
            self.get_logger().info("Reached the end of the video based on frame count. Shutting down.")
            self.cap.release()
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)
            return

        # Publish video frame
        ret, frame = self.cap.read()

        # If frame reading fails, log a warning and continue
        if not ret:
            self.get_logger().warn(f"Failed to read frame {self.frame_index}. Skipping.")
            self.frame_index += 1  # Skip this frame and try the next one
            return

        # Convert frame to a ROS2 Image message
        try:
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.video_pub.publish(ros_image)
            self.get_logger().info(f"Published video frame {self.frame_index + 1} / {self.frame_count}")
        except Exception as e:
            self.get_logger().error(f"Error publishing video frame: {e}")
            return

        self.frame_index += 1

        # Publish telemetry data if available
        if self.telemetry_available and self.telemetry_data:
            # Get current timestamp in seconds since start of playback
            current_time_seconds = self.frame_index / self.fps

            # Publish telemetry data if it's time
            while self.telemetry_index < len(self.telemetry_data) and \
                    current_time_seconds >= self.telemetry_data[self.telemetry_index][0]:
                telemetry_dict = self.telemetry_data[self.telemetry_index][1]
                self.publish_telemetry(telemetry_dict)
                self.telemetry_index += 1


    def publish_telemetry(self, telemetry_dict):
        gps_msg = NavSatFix()

        # Extract GPS data
        lat_str = telemetry_dict.get('Latitude')
        lon_str = telemetry_dict.get('Longitude')
        alt_str = telemetry_dict.get('Alt (Rel)')

        if lat_str:
            try:
                gps_msg.latitude = float(lat_str)
            except ValueError:
                self.get_logger().error(f"Invalid latitude value: {lat_str}")
                gps_msg.latitude = 0.0
        else:
            gps_msg.latitude = 0.0

        if lon_str:
            try:
                gps_msg.longitude = float(lon_str)
            except ValueError:
                self.get_logger().error(f"Invalid longitude value: {lon_str}")
                gps_msg.longitude = 0.0
        else:
            gps_msg.longitude = 0.0

        if alt_str:
            try:
                # Remove unit if present (e.g., '13.3 m')
                alt_value = alt_str.split()[0]
                gps_msg.altitude = float(alt_value)
            except ValueError:
                self.get_logger().error(f"Invalid altitude value: {alt_str}")
                gps_msg.altitude = 0.0
        else:
            gps_msg.altitude = 0.0

        # Publish GPS data
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        self.gps_pub.publish(gps_msg)
        self.get_logger().info(f"Published GPS: Lat={gps_msg.latitude}, Lon={gps_msg.longitude}, Alt={gps_msg.altitude}")

        # Extract Ground Speed data
        groundspeed_str = telemetry_dict.get('Ground Speed')
        if groundspeed_str:
            try:
                # Remove unit if present (e.g., '1.9 m/s')
                groundspeed_value = groundspeed_str.split()[0]
                groundspeed = float(groundspeed_value)
            except ValueError:
                self.get_logger().error(f"Invalid Ground Speed value: {groundspeed_str}")
                groundspeed = None
        else:
            groundspeed = None

        # Publish Ground Speed data
        if groundspeed is not None:
            groundspeed_msg = Float64()
            groundspeed_msg.data = groundspeed
            self.groundspeed_pub.publish(groundspeed_msg)
            self.get_logger().info(f"Published Ground Speed: {groundspeed} m/s")

        # Extract Climb Rate data
        climb_rate_str = telemetry_dict.get('Climb Rate')
        if climb_rate_str:
            try:
                # Remove unit if present (e.g., '0.0 m/s')
                climb_rate_value = climb_rate_str.split()[0]
                climb_rate = float(climb_rate_value)
            except ValueError:
                self.get_logger().error(f"Invalid Climb Rate value: {climb_rate_str}")
                climb_rate = None
        else:
            climb_rate = None

        # Publish Climb Rate data
        if climb_rate is not None:
            climb_rate_msg = Float64()
            climb_rate_msg.data = climb_rate
            self.climb_rate_pub.publish(climb_rate_msg)
            self.get_logger().info(f"Published Climb Rate: {climb_rate} m/s")

    def destroy_node(self):
        # Release video capture when node is destroyed
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    # Load configuration from YAML file
    config_file = 'config.yaml'
    if not os.path.isfile(config_file):
        print(f"Error: Configuration file '{config_file}' not found.")
        sys.exit(1)

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    # Are configuration parameters are present
    required_params = ['video_file_path']
    for param in required_params:
        if param not in config:
            print(f"Error: Missing required configuration parameter '{param}' in '{config_file}'.")
            sys.exit(1)

    publisher = VideoAndTelemetryPublisher(config)

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            publisher.destroy_node()
            rclpy.shutdown()
        sys.exit(0)

if __name__ == '__main__':
    main()
