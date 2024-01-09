#include "ofApp.h"
//#include "example.hpp"

//	[Brizo] 09-01-2024
//	- Converts RealSense 16-bit depth stream to 8-bit/channel RGB and forwards via Spout...
//		- Green channel gets multiplied by a factor, because its range is too small
//	- ... so a third-party application records the depth video
//	- Creates point cloud from depth
//	TODO:
//	- Save somewhere the value of the green channel multiplier (GMult) of a recording
//	- Read recorded depth videos and recover the depth values correctly
//	- From these videos, generate the point cloud for each frame and write the frames sequentially in a vertex-only .OBJ file
//		- And, of course, perform tests rendering point clouds from our OBJ files...
//	- Port this application to Kinect v2 (a.k.a. Kinect ONE)
//
//	Instructions: Check ofApp::keyReleased below

void ofApp::setup(){
    
	device = new rs2::device();

    ofSetFrameRate(30);
    ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_NOTICE);

	mesh.setMode(OF_PRIMITIVE_POINTS);

	glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
	glPointSize(renderingDefs.pointSize); // make the points bigger
    
	//
	//	RealSense camera

	// if you see app crash at runtime, please check,
	// 1. Copy Intel.Realsense.dll and realsense2.dll in to /bin? 
	// 2. Unplug and re-connect Realsense camera and restart app.
	pipe.start();

	//	FIXME - hard-coded, may change per configuration or device
	int w = 848;
	int h = 480;

	//	Spout - Initialise the Spout sender
	spoutSender.init("DepthVideo", w, h);// , OF_PIXELS_RGB);	//	848x480: RealSense depth width and height
	
	//depthFrameMono = new unsigned short[w*h];
	//depthFrameRG   = new unsigned char [w*h*2];
	depthFrameRGB  = new unsigned char [w*h*3];
	
	//	According to ofGraphicsConstants.h, OF_IMAGE_GRAYSCALE is expected to use OF_PIXELS_GRAY, which are 1 byte.
	//	In fact, allocate calls ofPixels::ofPixelFormatFromImageType(OF_IMAGE_GRAYSCALE), which returns OF_PIXELS_GRAY(which is 1 byte).
	//If we could use instead OF_PIXELS_GRAY_ALPHA we would be expected to use 2 channels(see ofImage::channelsFromPixelFormat)
	//	We decided to use a RGB image:
	depthOFImage.allocate(w, h, OF_IMAGE_COLOR);
	//depthPixels.allocate(848, 480, 2);

}

void ofApp::convert16BitTo2Channel8bit(unsigned short* in, int in_size, unsigned char* out)
{	
	for (int i = 0; i < in_size; i++) {
		out[i*2] = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
		out[i*2 + 1] = static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
	}
}
void ofApp::convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out)
{
	for (int i = 0; i < in_size; i++) {
		out[i*3]     = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
		out[i*3 + 1] = GMult * static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
		out[i*3 + 2] = 0;
	}
}

void ofApp::realsenseUpdate()
{
	// Get depth data from camera
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	if (!depth) {
		return;
	}
	points = pc.calculate(depth);

	//std::cout << depth.get_bytes_per_pixel() << std::endl;
	//std::cout << depth.get_stride_in_bytes() << std::endl;
	//std::cout << "-----" << std::endl;
	//depth.get_profile().format()

	// Create oF mesh
	mesh.clear();
	int n = points.size();
	if (n != 0) {
		const rs2::vertex * vs = points.get_vertices();
		for (int i = 0; i < n; i++) {
			if (vs[i].z) {
				//std::cout << vs[i].z << " ";
				const rs2::vertex v = vs[i];
				glm::vec3 v3(10.0*v.x, 10.0*v.y, 10.0*v.z);
				mesh.addVertex(v3);
				mesh.addColor(ofFloatColor(0, 0, ofMap(v.z, 2, 6, 1, 0), 0.8));
			}
		}
	}

	int w = depth.get_width();
	int h = depth.get_height();

	//	NOTE - Investigate why the parameters below aren't correct (or if there's a bug):
	//depthOFTexture.allocate(848, 480, GL_INTENSITY16);
	//depthOFTexture.loadData(depth.get_data(), w, h, OF_PIXELS_GRAY_ALPHA, GL_LUMINANCE_ALPHA);
	//	Not working either...
	//ofPixels pix;
	//pix.allocate(848, 480, OF_PIXELS_GRAY_ALPHA);
	//depthOFTexture.readToPixels(pix);
	//depthOFImage.setFromPixels(pix);
	//	Not working either?
	//depthPixels.setFromAlignedPixels(reinterpret_cast<unsigned char*>(depthPixels.getData()), 848, 480, OF_PIXELS_GRAY_ALPHA, 848*2);

	this->convert16BitTo3Channel8bit((unsigned short*)(depth.get_data()), w * h, depthFrameRGB);
	depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);
}

void ofApp::update()
{
	realsenseUpdate();

	// Send the camera's texture once per frame
	spoutSender.send(depthOFImage.getTexture());
}

void ofApp::draw(){
    
    ofBackground(200);

	//	TODO - Texture a pkane with our ofImage so we can render both it and the point cloud
	if (this->drawOption == DRAW_OPT::DEPTH)
	{
		depthOFImage.draw(0, 0);
	}
	else {	//	Point cloud
		cam.begin();
		float s = 200;
		ofScale(s, -s, -s);
		ofDrawAxis(1);

		ofPushMatrix();
		ofTranslate(0, 1, 0);
		ofRotateZDeg(90);
		ofSetColor(0, 200);
		ofDrawGridPlane(1, 5, true);
		ofPopMatrix();

		mesh.drawVertices();

		cam.end();
	}
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
	//
	//	Some tests on the point cloud visualization:

	if (key == ofKey::OF_KEY_RIGHT) {
		mesh.setMode(OF_PRIMITIVE_POINTS);
	}
	else if (key == ofKey::OF_KEY_LEFT) {
		mesh.setMode(OF_PRIMITIVE_LINES);
	}
	else if ( ((key-48)>=0) && ((key-48)<10) ){
		mesh.setMode(ofPrimitiveMode(key - 48));
	}
	else if (key == '-') {
		if ((renderingDefs.pointSize - 0.1) > 0) {
			renderingDefs.pointSize -= 0.1;
			glPointSize(renderingDefs.pointSize);
		}
	}
	else if (key == '+') {
		renderingDefs.pointSize += 0.1;
		glPointSize(renderingDefs.pointSize);
	}
	
	//
	//	Multiplier of the green channel for the depth video:

	else if (key == ofKey::OF_KEY_UP) {
		GMult++;
		std::cout << "Green channel multiplier: " << GMult << std::endl;
	}
	else if (key == ofKey::OF_KEY_DOWN) {
		if (GMult > 1) {
			GMult--;
			std::cout << "Green channel multiplier: " << GMult << std::endl;
		}
	}

	//
	//	Draw option (FIXME - Remove this option and draw everything in 3d)
	else if (key == ' ') {
		if (drawOption == DRAW_OPT::DEPTH) {
			drawOption = DRAW_OPT::POINTCLOUD;
		}
		else {
			drawOption = DRAW_OPT::DEPTH;
		}
	}
}
