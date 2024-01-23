# depthVideoRecorder
Tool to facilitate depth video recordings for point cloud footage

This is a [LabPresenca](https://www.labpresenca.com.br)'s tool for making Toré Virtual, a VR movie that documents an indigenous ritual in a transfigured way, in co-creation with Fulni-Ô people.  
Tore Virtual is on the making thanks to the Funarte Retomada public prize, funded by Fundação Nacional das Artes, from the brazilian Ministry of Culture.

<img src="https://github.com/brizolara/depthVideoRecorder/blob/main/depth-video-recorder.png?raw=true">

## Overview

0. Inspection - Show color depth frame, color frame and point cloud  
2. Recording  
  1.1. Depth frames from Intel RealSense (and soon Kinect v2) are decoded into RGB textures and shared via Spout  
  1.2. The concurrent color frames are also shared via Spout  
  1.3. Both frames are recorded side-by-side to a video file via a video recorder (we use the [OBS - Open Broadcaster Software](https://obsproject.com) with the [Spout plugin](https://github.com/Off-World-Live/obs-spout2-plugin))  
    1.3.1. Set up OBS for lossless, RGB, and with same resolution of two depth images side-by-side (1696x960 for our RealSense configuration)! Up until now, the point cloud calculated from the depth video didn't survive the slightest encoding of the depth stream - A well-designed smooth 4096-level color map _may_ give soom room here, so we can have smaller files.
3. Playback  
  2.1. Recorded videos can be loaded along the with the reconstructed point cloud on-the-fly  
  2.2. The point cloud animation (sequence of point clouds) can be saved as a sequence of vertex-only a .ply files, to be imported in Blender, e.g.

## Setup
  OpenFrameworks 0.10.0. We are using Visual Studio 2017.  
  OpenFrameworks addons: ofxGui, ofxRealSense2, ofxSpout.
