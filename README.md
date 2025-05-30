# Laser Tracking system with Ros2 (PX4)

This repository contains a complete laser targeting and tracking system that integrates hardware (Arduino, USB camera, servo motor with laser) and software components (YOLO, MiDaS, and ROS 2 packages) to accurately pinpoint and track a target in real-time.

# Architecture

## Hardware
- ### USB Camera

  Captures live video stream.
- ### Arduino

  Controls a servo motor to point a laser at the target.
- ### Servo Motor + Laser

  Points to the detected target in real-time.

## Software (ROS 2 + Docker)
- ### my_camera_pkg:

  Launches the USB camera and publishes image data.
- ### px2:

  Subscribes to the camera image. Runs YOLO for object detection and MiDaS for depth estimation. Publishes the detected absolute distance and x-coordinate of the bounding box center to the next node.
- ### hw_px3:

  Subscribes to absolute distance and x-coordinate. Computes the required servo angle based on a mathematical formula. Sends the angle to the Arduino to control the servo motor for precise laser targeting.
- ### custom_msg:

  Defines custom ROS 2 message types for data exchange between nodes.

- ### system_launch:

  Launches the entire system (camera, AI modules, servo control) with a single command.
