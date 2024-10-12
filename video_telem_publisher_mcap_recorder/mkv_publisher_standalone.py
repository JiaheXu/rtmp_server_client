import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

# Set path to MKV file
mkv_file_path = "/home/starfish_admin/projects/TRIAGE2/logs/9-29-24/COURSE_3/2024-09-29_15.14.52_COURSE_3.mkv"

class VideoPublisher(Node):
    def __init__(self, video_path):
        super().__init__('video_publisher')
        
        # Create a publisher that will publish video frames as ROS Image messages
        self.publisher_ = self.create_publisher(Image, 'drone_camera', 10)
        self.bridge = CvBridge()
        
        # Open MKV video file using OpenCV
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open the video file: {video_path}")
            return

        # Get video frame rate
        self.frame_rate = self.cap.get(cv2.CAP_PROP_FPS)
        if self.frame_rate <= 0:
            self.get_logger().warn(f"Could not determine frame rate, defaulting to 10 FPS.")
            self.frame_rate = 10.0
        
        self.frame_interval = 1.0 / self.frame_rate  # Interval between frames

        # Get video width and height
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        self.get_logger().info(f"Video properties - Frame Rate: {self.frame_rate} FPS, Width: {self.frame_width}, Height: {self.frame_height}")
        
        # Create a timer that matches video frame rate
        self.timer = self.create_timer(self.frame_interval, self.publish_frame)
        
    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("End of video, shutting down.")
            rclpy.shutdown()
            return

        # Convert frame to a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Publish image
        self.publisher_.publish(ros_image)
        self.get_logger().info(f"Published frame at time: {self.get_clock().now().to_msg()}")

def main(args=None):
    rclpy.init(args=args)

    if not os.path.exists(mkv_file_path):
        print(f"Error: The specified MKV file does not exist: {mkv_file_path}")
    else:
        video_publisher = VideoPublisher(mkv_file_path)
        rclpy.spin(video_publisher)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
