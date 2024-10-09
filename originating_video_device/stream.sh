#!/bin/bash


FRAMERATE=10
RESOLUTION="1920x1080"
BITRATE="3M"

# Fixed parameters for local stream
RTSP_URL="rtsp://10.223.61.63:8554/ghadron"
LOCAL_UDP="udp://127.0.0.1:1234"
#REMOTE_RTMP="rtmp://10.223.132.96:1937/live/stream"
REMOTE_RTMP="rtmp://10.223.61.67:1937/live/stream"
#LOCAL_CODEC="copy"
REMOTE_CODEC="libx264"
PRESET="ultrafast" # Default ultrafast
BUF_SIZE="6.0M"  # Reduce buffer size to lower latency
		 # Buffer size should be at least equal to or larger than -maxrate or bitrate. A common practice is to set it to double the -maxrate.
GOP_SIZE=20	 # Group of Pictures. controls how frequently keyframes (I-frames) are inserted
		 # Keyframes take up more bandwidth but are essential for ensuring that viewers can start playback or seek into the video stream quickly. A lower g value can reduce latency but increase bandwidth usage, while a higher value increases compression efficiency but introduces more delay.
		 # Advisable to set the GOP size to a multiple of the framerate, such as 30 (for a 15 fps stream).
# Command to run ffmpeg local and remote forwarding
#ffmpeg -fflags nobuffer \
#    -flags low_delay \
#    -rtsp_transport tcp \
#    -i "$RTSP_URL" \
#    -c:v "$LOCAL_CODEC" \
#    -f mpegts "$LOCAL_UDP" \
#    -c:v "$REMOTE_CODEC" \
#    -preset "$PRESET" \
#    -tune zerolatency \
#    -g $GOP_SIZE \
#    -maxrate "$BITRATE" \
#    -bufsize "$BUF_SIZE" \
#    -vf "fps=$FRAMERATE,scale=$RESOLUTION" \
#    -f flv "$REMOTE_RTMP"
#
#exit 0

# Command to run ffmpeg remote forwarding
ffmpeg -fflags nobuffer \
    -flags low_delay \
    -rtsp_transport tcp \
    -i "$RTSP_URL" \
    -c:v "$REMOTE_CODEC" \
    -preset "$PRESET" \
    -tune zerolatency \
    -g $GOP_SIZE \
    -b:v "$BITRATE" \
    -maxrate "$BITRATE" \
    -bufsize "$BUF_SIZE" \
    -x264-params "nal-hrd=cbr:force-cfr=1" \
    -vf "fps=$FRAMERATE,scale=$RESOLUTION" \
    -fflags +discardcorrupt \
    -err_detect ignore_err \
    -f flv "$REMOTE_RTMP"

