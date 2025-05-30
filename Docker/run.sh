# bin/bash
CAMERA_DEV="/dev/video4"
ARDUINO_DEV="/dev/ttyACM1"
# Check if Arduino and CAM device exists
if [ ! -e "$CAMERA_DEV" ]; then
  echo "Error: $CAMERA_DEV not found on host."
  exit 1
fi

if [ ! -e "$ARDUINO_DEV" ]; then
  echo "Error: $ARDUINO_DEV not found on host."
  exit 1
fi


docker run -it --rm \
  --net=host \
  --privileged \
  --device=$CAMERA_DEV \
  --device=$ARDUINO_DEV \
  --group-add video \
  -v /home/hagi/Downloads/ggg/workspace/:/ros2_ws \
  test:latest
# v4l2-ctl --list-devices
