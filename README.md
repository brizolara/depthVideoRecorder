# depthVideoRecorder
Tool to facilitate depth video recordings for point cloud footage

## Overview
1. Record  
  1.1. This application decodes depth streams from Kinect v2 (TO-DO) or Intel RealSense into RGB  
  1.2. Then forwards to a video recorder via Spout (we use the [OBS - Open Broadcaster Software](https://obsproject.com), with the [Spout plugin](https://github.com/Off-World-Live/obs-spout2-plugin))
2. Playback  
  2.1. This application reads a recorded video and plays back the corresponding point cloud on-the-fly
  2.2. If the playback is ok, we record the point cloud animation as a .obj containg the frames in sequence, to be imported in Blender, e.g.

First commit:
- Converts RealSense 16-bit depth stream to 8-bit/channel RGB and forwards via Spout...
	- Green channel gets multiplied by a factor, because its range is too small
- ... so a third-party application records the depth video
- Creates point cloud from depth

TODO:
  - Save somewhere the value of the green channel multiplier (GMult) of a recording
  - Read recorded depth videos and recover the depth values correctly
  - From these videos, generate the point cloud for each frame and write the frames sequentially in a vertex-only .OBJ file
    - And, of course, perform tests rendering point clouds from our OBJ files...
  - Port this application to Kinect v2 (a.k.a. Kinect ONE)

## Usage	instructions
Check out ofApp::keyReleased...
