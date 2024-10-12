import subprocess
import os
import signal
import sys

def get_incremented_bag_name(base_path, base_name):
    """
    Increment the bag file name if it already exists.
    Example: If "drone_data_bag_0" exists, it will create "drone_data_bag_1".
    """
    index = 0
    while True:
        bag_name = f"{base_name}_{index}"
        output_path = os.path.join(base_path, bag_name)
        if not os.path.exists(output_path):
            return output_path
        index += 1

def start_ros2_bag_recording(base_path, base_name="drone_data_bag", drone_id="drone1"):
    """
    Starts ros2 bag recording process with an incremented filename if it already exists.
    This creates an uncompressed MCAP file
    """
    # Ensure base path exists
    if not os.path.exists(base_path):
        os.makedirs(base_path)
        print(f"Created directory: {base_path}")

    # Get incremented bag file path
    output_path = get_incremented_bag_name(base_path, base_name)
    print(f"Recording to: {output_path}")

    # Build ros2 bag record command for uncompressed MCAP
    topics = f"/{drone_id}/camera /{drone_id}/gps /{drone_id}/groundspeed /{drone_id}/climb_rate"
    command = f"source /opt/ros/humble/setup.bash && ros2 bag record {topics} " \
          f"--output {output_path} --storage mcap"


    # Run ros2 bag recording command
    try:
        # Start process and return subprocess.Popen object
        process = subprocess.Popen(['bash', '-c', command], preexec_fn=os.setsid)
        print(f"Recording started successfully at {output_path}")
        return process, output_path
    except Exception as e:
        print(f"Error starting ros2 bag recording: {e}")
        sys.exit(1)

def compress_mcap_file(output_path):
    """
    Compresses the .mcap file using zstd to create a .mcap.zstd file.
    """
    mcap_file = f"{output_path}.mcap"
    compressed_file = f"{mcap_file}.zstd"

    if os.path.exists(mcap_file):
        print(f"Compressing {mcap_file} to {compressed_file}...")
        try:
            subprocess.run(['zstd', mcap_file, '-o', compressed_file], check=True)
            print(f"Compression successful: {compressed_file}")
        except subprocess.CalledProcessError as e:
            print(f"Error during compression: {e}")
    else:
        print(f"Error: {mcap_file} does not exist. Cannot compress.")

if __name__ == "__main__":
    # Base directory and name for bag files
    base_directory = "/home/starfish_admin/projects/TRIAGE2/converted_videos_to_mcap"
    base_bag_name = "drone_data_bag"

    drone_id = "drone1"

    # Start recording and get process object and file path
    recorder_process, output_path = start_ros2_bag_recording(base_directory, base_bag_name, drone_id)

    try:
        # Wait for the recording process to complete
        recorder_process.wait()
    except KeyboardInterrupt:
        # Ctrl+C graceful stopping
        print("Interrupt received, stopping recording...")
        os.killpg(os.getpgid(recorder_process.pid), signal.SIGINT)
        recorder_process.wait()
    finally:
        print("Recording process has terminated.")
        
        # Compress the .mcap file after recording finishes
        compress_mcap_file(output_path)
