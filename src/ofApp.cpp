#include "ofApp.h"
//#include "ofJson.h"	// actually uses nlohmann's json
#include "librealsense2/rsutil.h"
#include "librealsense2/h/rs_option.h"
#include "librealsense2/h/rs_sensor.h"
#include "json.hpp"
#include "ofUtils.h"
#include "ofMath.h"

//	[Brizo] 04-01-2024 - 16-01-2024
//	This is a LabPresenca's tool for making Tore Virtual, a VR movie that documents a indigenous ritual in a transfigured way,
//in co-creation with Fulni-O people
//	Tore Virtual is on the making thanks to a Funarte public prize, from the brazilian federal government.
//	
//	What this does:
//	- Applies Near/Far clipping in depth video from RealSense D455
//	- Converts the RealSense depth video (it's a 16-bit depth stream) to 8-bit/channel RGB and forwards via Spout...
//		- Green channel gets multiplied by a factor, because its range is too small - TODO: review the GMult feature
//	- ... so a third-party application records the depth video (lossless, RGB! Up until now, point cloud didn't survive the slightest encoding)
//	- Creates point cloud from depth
//	- Reads recorded depth videos and recovers the depth values correctly (and hence, point cloud)[OK, but test for big depth spans]
//
//	Coming Soon:
//	- Save somewhere the value of the green channel multiplier (GMult) of a recording - TODO: review the GMult feature
//	- Write to file the point clouds generated from depth video file, in a vertex-only .OBJ file per frame
//	- And, of course, perform tests rendering point clouds from our OBJ files...
//	- Support for Kinect v2 (a.k.a. Kinect ONE)
//	- Add a clipping cuboid (=rectangular prism) - frustrum would be nice, but let's keep it simple...
//
//	Instructions:
//	- Press 'l' to load a video file instead of using the sensor.
//	- Check ofApp::keyReleased below

void ofApp::setup()
{
	gui.setup("Near/Far clipping");
	gui.add(near_US.set("Near", 417, 0, 512));
	gui.add(far_US.set("Far", 793, 256, 4096));

	//	TESTS
	//	* Testing conversions used to transcode depth to video and vice-versa
	bool test_success = testConversions_US_UC3Channels(/*logToConsole*/ false);
	assert(test_success == true);
	
	ofSetFrameRate(30);
    ofSetVerticalSync(true);
    ofSetLogLevel(OF_LOG_NOTICE);

	mesh.setMode(OF_PRIMITIVE_POINTS);

	glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
	glPointSize(renderingDefs.pointSize); // make the points bigger
    
	//
	//	RealSense camera

	//	As the optimal depth resolution of the D455 device is 848x480, we don't change the default configuration
	/*rs2::config cfg;
	cfg.enable_device("231122302600"); // NOTE: This HARD-CODED number is the serial number of your device
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);	*/

	pipelineProfile = pipe.start();	//	if we use custom configuration: pipe.start(cfg);

	realSenseDepthIntrinsics = nullptr;
}

//--------------------------------------------------------------
///	Set up everything that demanded knowledge about the stream
void ofApp::finishSetup()
{
	//	Spout - It shares the video memory with other Spout-enabled applications. We use for recording the depth stream
	spoutSender.init("DepthVideo", w, h);// , OF_PIXELS_RGB);

	depthFrameRGB = new unsigned char[w*h * 3];

	//	NOTE - We would prefer to work with 16-bit monochrome depth images and depth video, but...
	//	According to ofGraphicsConstants.h, OF_IMAGE_GRAYSCALE is expected to use OF_PIXELS_GRAY, which are 1 byte.
	//	In fact, allocate calls ofPixels::ofPixelFormatFromImageType(OF_IMAGE_GRAYSCALE), which returns OF_PIXELS_GRAY(which is 1 byte).
	//If we could use instead OF_PIXELS_GRAY_ALPHA we would be expected to use 2 channels(see ofImage::channelsFromPixelFormat)
	//	For now, we decided to use a RGB image:
	depthOFImage.allocate(w, h, OF_IMAGE_COLOR);
	//depthPixels.allocate(848, 480, 2);

	reconstructed_depthOFImage.allocate(w, h, OF_IMAGE_COLOR);
	reconstructedDepth = new unsigned short[w*h];

	//	For tests
	differenceDepthRGB = new unsigned char[w*h * 3];

	xyz_pointCloud = new rs2::vertex[w*h];
}

//--------------------------------------------------------------
void ofApp::convert16BitTo2Channel8bit(unsigned short* in, int in_size, unsigned char* out)
{	
	for (int i = 0; i < in_size; i++) {
		out[i*2] = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
		out[i*2 + 1] = static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
	}
}

//--------------------------------------------------------------
void ofApp::convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out)
{
	for (int i = 0; i < in_size; i++) {
		out[i*3 + 0] = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
		out[i*3 + 1] = GMult * static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
		out[i*3 + 2] = 0;
	}
}

//--------------------------------------------------------------
void ofApp::convert3Channel8bitTo16Bit(unsigned char* in, int out_size, unsigned short* out)
{
	for (int i = 0; i < out_size; i++) {
		out[i] = (unsigned short)(in[3*i + 1])*256/GMult + unsigned short(in[3*i]);
	}
}

//--------------------------------------------------------------
void ofApp::convert16BitTo3Channel8bit_mapped(unsigned short* in, int in_size, unsigned char* out, unsigned char** map)
{
	for (int i = 0; i < in_size; i++) {
		out[i * 3 + 0] = map[min(4095, (int)in[i])][0];
		out[i * 3 + 1] = map[min(4095, (int)in[i])][1];
		out[i * 3 + 2] = map[min(4095, (int)in[i])][2];
	}
}

//--------------------------------------------------------------
void ofApp::convert_mapped3Channel8bitTo16Bit(unsigned char* in, int out_size, unsigned short* out, unsigned char** map)
{
	unsigned short i, j, k, n; // 3 indices of the LUT (color map) and the sequential (or flat) index
	unsigned short r, g, b;
	for (int i = 0; i < out_size; i++) {
		r = (unsigned short)in[3 * i];
		g = (unsigned short)in[3 * i + 1];
		b = (unsigned short)in[3 * i + 2];
		k = (255 - r) / 17;
		i = ((1 - (k % 2)) * 15) + (g * ((k % 2) * 2 - 1) / 17);
		//j = (255 - b) / 17;	this is for the html color map (the original "1004")
		j = ((1 - (i % 2)) * 15) + (b * ((i % 2) * 2 - 1) / 17); // and this is for our version
		out[i] = (256*k) + (16*j) + i;
	}
}

//--------------------------------------------------------------
//	TODO - Should we do this in the configuration of the device? Or use this to write the device configuration?
void ofApp::applyNearAndFar(unsigned short* data, int size)
{
	for (int i = 0; i < size; i++) {
		if (data[i] < near_US.get()) {
			data[i] = 0;
		}
		else if (data[i] > far_US.get()) {
			data[i] = 0;
		}
	}
}

//--------------------------------------------------------------
void ofApp::writeRealSenseIntrinsics(std::string p_filepath)
{
	nlohmann::json params;

	params["depth_intrin"]["width"] = realSenseDepthIntrinsics->width;
	params["depth_intrin"]["height"] = realSenseDepthIntrinsics->height;
	params["depth_intrin"]["ppx"] = realSenseDepthIntrinsics->ppx;
	params["depth_intrin"]["ppy"] = realSenseDepthIntrinsics->ppy;
	params["depth_intrin"]["fx"] = realSenseDepthIntrinsics->fx;
	params["depth_intrin"]["fy"] = realSenseDepthIntrinsics->fy;
	params["depth_intrin"]["model"] = realSenseDepthIntrinsics->model;
	params["depth_intrin"]["coeffs0"] = realSenseDepthIntrinsics->coeffs[0];
	params["depth_intrin"]["coeffs1"] = realSenseDepthIntrinsics->coeffs[1];
	params["depth_intrin"]["coeffs2"] = realSenseDepthIntrinsics->coeffs[2];
	params["depth_intrin"]["coeffs3"] = realSenseDepthIntrinsics->coeffs[3];
	params["depth_intrin"]["coeffs4"] = realSenseDepthIntrinsics->coeffs[4];

	/* Other parameters can be retrieved form :
	depth_intrin = depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
	color_intrin = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
	color_extrin_to_depth = color_profile.as<rs2::video_stream_profile>().get_extrinsics_to(depth_profile);
	depth_extrin_to_color = depth_profile.as<rs2::video_stream_profile>().get_extrinsics_to(color_profile);*/
	
	std::ofstream f(std::string(p_filepath) + this->FNAME_INTRINSIC_PARAMS);
	f << params;
	f.flush();
}

//--------------------------------------------------------------
void ofApp::readRealSenseIntrinsics(std::string p_filepath)
{
	std::ifstream f(p_filepath);
	nlohmann::json params = nlohmann::json::parse(f);

	realSenseDepthIntrinsics->width = params["depth_intrin"]["width"];
	realSenseDepthIntrinsics->height = params["depth_intrin"]["height"];
	realSenseDepthIntrinsics->ppx = params["depth_intrin"]["ppx"];
	realSenseDepthIntrinsics->ppy = params["depth_intrin"]["ppy"];
	realSenseDepthIntrinsics->fx = params["depth_intrin"]["fx"];
	realSenseDepthIntrinsics->fy = params["depth_intrin"]["fy"];
	realSenseDepthIntrinsics->model = params["depth_intrin"]["model"];
	realSenseDepthIntrinsics->coeffs[0] = params["depth_intrin"]["coeffs0"];
	realSenseDepthIntrinsics->coeffs[1] = params["depth_intrin"]["coeffs1"];
	realSenseDepthIntrinsics->coeffs[2] = params["depth_intrin"]["coeffs2"];
	realSenseDepthIntrinsics->coeffs[3] = params["depth_intrin"]["coeffs3"];
	realSenseDepthIntrinsics->coeffs[4] = params["depth_intrin"]["coeffs4"];
}

//--------------------------------------------------------------
void ofApp::writeDeviceInfo(std::string p_filepath)
{
	float fov[2];
	rs2_fov(realSenseDepthIntrinsics, &fov[0]);

	nlohmann::json params;

	char s[256];
	sprintf(s, "%f %f", fov[0], fov[1]);
	params["info"]["fov"] = std::string(s);

	params["info"]["advanced_mode"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_ADVANCED_MODE));
	params["info"]["locked"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_CAMERA_LOCKED));
	params["info"]["op_code"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_DEBUG_OP_CODE));
	params["info"]["firmware_version"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION));
	params["info"]["camera_name"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME));
	params["info"]["product_id"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_PRODUCT_ID));
	params["info"]["serial"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER));
	params["info"]["usb_compatibility"] = std::string(device.get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));
	
	std::ofstream f(std::string(p_filepath) + this->FNAME_DEVICE_INFO);
	f << params;
	f.flush();
}

//--------------------------------------------------------------
//	NOTE - Includes the zero-depth elements (i.e., points where depth couldn't be estimated). We left to you to get rid of them...
//	Adapted from https://github.com/IntelRealSense/librealsense/blob/v2.24.0/wrappers/python/examples/box_dimensioner_multicam/helper_functions.py#L121-L147
void ofApp::pointCloudFromDepth(unsigned short* p_depth, rs2_intrinsics* p_intrinsics, rs2::vertex* xyz_out)
{

//	Convert the depthmap to a 3D point cloud
//
//	Parameters :
//---------- -
//depth_frame : rs.frame()
//	The depth_frame containing the depth map
//	camera_intrinsics : The intrinsic values of the imager in whose coordinate system the depth_frame is computed
//
//	Return :
//----------
//	x : array
//	The x values of the pointcloud in meters
//	y : array
//	The y values of the pointcloud in meters
//	z : array
//	The z values of the pointcloud in meters

/*static void rs2_deproject_pixel_to_point(float point[3], const struct rs2_intrinsics * intrin, const float pixel[2], float depth)
{
	assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
	assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
	//assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

	float x = (pixel[0] - intrin->ppx) / intrin->fx;
	float y = (pixel[1] - intrin->ppy) / intrin->fy;
	if (intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
	{
		float r2 = x * x + y * y;
		float f = 1 + intrin->coeffs[0] * r2 + intrin->coeffs[1] * r2*r2 + intrin->coeffs[4] * r2*r2*r2;
		float ux = x * f + 2 * intrin->coeffs[2] * x*y + intrin->coeffs[3] * (r2 + 2 * x*x);
		float uy = y * f + 2 * intrin->coeffs[3] * x*y + intrin->coeffs[2] * (r2 + 2 * y*y);
		x = ux;
		y = uy;
	}
	point[0] = depth * x;
	point[1] = depth * y;
	point[2] = depth;
}*/
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			xyz_out[i*w + j].z = p_depth[i*w + j] / 1000.0;
			xyz_out[i*w + j].x = xyz_out[i*w + j].z * (i - p_intrinsics->ppx) / p_intrinsics->fx;
			xyz_out[i*w + j].y = xyz_out[i*w + j].z * (j - p_intrinsics->ppy) / p_intrinsics->fy;
		}
	}
}

//--------------------------------------------------------------
bool ofApp::testConversions_US_UC3Channels(bool logToConsole)
{
	unsigned short us_check[2] = { 256, 513 };
	unsigned short us[2] = { us_check[0], us_check[1] };
	unsigned char uc[6] = { 0, 0, 0, 0, 0, 0 };

	if (logToConsole) {
		std::cout << "testConversions_US_UC3Channels" << std::endl;
		std::cout << "Assuming Big Endianess." << std::endl << std::endl;
		std::cout << "* Input array (2 unsigned shorts): " << std::endl;
		std::cout << us_check[0] << " " << us_check[1] << std::endl;
	}

	this->convert16BitTo3Channel8bit(&us[0], 2, &uc[0]);

	if (logToConsole) {
		std::cout << std::endl << "Now, note that US format is 2 bytes, but we are going to allocate each US value as a sequence of" << std::endl;
		std::cout << "3 bytes, where only 2 are used (this is for compatibility with RGB images). As we assume Big Endian," << std::endl;
		std::cout << "for every 3 bytes, the third will be zero." << std::endl << std::endl;
		std::cout << "* Input array converted to 6 bytes, Big Endian:" << std::endl;
		std::cout << (int)uc[0] << " " << (int)uc[1] << " " << (int)uc[2] << " | " << (int)uc[3] << " " << (int)uc[4] << " " << (int)uc[5] << std::endl;
	}

	this->convert3Channel8bitTo16Bit(&uc[0], 2, &us[0]);

	if (logToConsole) {
		std::cout << "* Reconstructed Input array:" << std::endl;
		std::cout << us[0] << " " << us[1] << std::endl;
	}

	return (us_check[0] == us[0]) && (us_check[1] == us[1]);
}

//--------------------------------------------------------------
void ofApp::fillVboMesh(int p_npts, rs2::vertex* p_vertices)
{
	// Create oF mesh
	mesh.clear();

	if (p_npts != 0) {
		for (int i = 0; i < p_npts; i++) {
			if (p_vertices[i].z > 0.01f) {
				//std::cout << vs[i].z << " ";
				const rs2::vertex v = p_vertices[i];
				glm::vec3 v3(10.0*p_vertices[i].x, 10.0*p_vertices[i].y, 10.0*p_vertices[i].z);
				mesh.addVertex(v3);
				mesh.addColor(ofFloatColor(0, 0, ofMap(p_vertices[i].z, 2, 6, 1, 0), 0.8));
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::updateFromRealSense()
{
	// Get depth data from camera
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	if (!depth) {
		return;
	}

	this->applyNearAndFar((unsigned short*)(depth.get_data()), w*h);

	points = pc.calculate(depth);

	bool deviceInfoLogged = (realSenseDepthIntrinsics != nullptr);
	if (!deviceInfoLogged)
	{
		realSenseDepthIntrinsics = new rs2_intrinsics();
		*realSenseDepthIntrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

		std::cout << "RealSense device intrinsic parameters: " << std::endl;
		std::cout << "k1, k2, p1, p2, k3: " << realSenseDepthIntrinsics->coeffs[0] << " " << realSenseDepthIntrinsics->coeffs[1]
			<< " " << realSenseDepthIntrinsics->coeffs[2] << " " << realSenseDepthIntrinsics->coeffs[3] << " "
			<< realSenseDepthIntrinsics->coeffs[4] << std::endl;
		std::cout << "fx, fy: " << realSenseDepthIntrinsics->fx << " " << realSenseDepthIntrinsics->fy << std::endl;
		std::cout << "width, height: " << realSenseDepthIntrinsics->width << " " << realSenseDepthIntrinsics->height << std::endl;
		std::cout << "distortion model: " << realSenseDepthIntrinsics->model << std::endl;
		std::cout << "ppx, ppy: " << realSenseDepthIntrinsics->ppx << " " << realSenseDepthIntrinsics->ppy << std::endl;

		std::cout << "Writing json file to " << ofToDataPath("", true) << std::endl;

		writeRealSenseIntrinsics(ofToDataPath("",true));

		device = pipelineProfile.get_device();

		writeDeviceInfo(ofToDataPath("", true));

		//	The option below doesn't work. My RS Viewer (v2.54.1) doesn't save the resolution I set there in the Custom preset
		//	Use the custom preset you should have saved with the SDK's Intel RealSense Viewer
		if (device.query_sensors()[0].supports(rs2_option::RS2_OPTION_VISUAL_PRESET)) {
			device.query_sensors()[0].set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_CUSTOM);
			std::cout << "RS2_RS400_VISUAL_PRESET_CUSTOM should be set now..." << std::endl;
			/*std::cout << device.query_sensors()[0].get_option(rs2_option::RS2_OPTION_AUTO_EXPOSURE_MODE);
			std::cout << device.query_sensors()[0].get_option(rs2_option::RS2_OPTION_GAIN);
			std::cout << device.query_sensors()[0].get_option(rs2_option::RS2_OPTION_LASER_POWER);*/
		}
		w = depth.get_width();
		h = depth.get_height();

		finishSetup();
	}
	
	fillVboMesh(points.size(), (rs2::vertex*) points.get_vertices());

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
	//this->convert16BitTo3Channel8bit_mapped((unsigned short*)(depth.get_data()), w * h, depthFrameRGB, colormap);
	depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);
}

//--------------------------------------------------------------
void ofApp::updateFromVideoFile()
{
	if (!realSenseDepthIntrinsics)
	{
		realSenseDepthIntrinsics = new rs2_intrinsics();

		readRealSenseIntrinsics(ofToDataPath("", true) + this->FNAME_DEVICE_INFO);

		w = realSenseDepthIntrinsics->width;
		h = realSenseDepthIntrinsics->height;

		finishSetup();
	}

	unsigned char* videoFrameRGB = depthVidPlayer.getPixelsRef().getPixels();
	depthOFImage.setFromPixels(videoFrameRGB, w, h, OF_IMAGE_COLOR);

	this->convert3Channel8bitTo16Bit(depthOFImage.getPixelsRef().getData(), w*h, reconstructedDepth);

	/*
	//	Converting rgb read from video to unsigned short and then back to RGB to test the RGB->US conversion
	this->convert16BitTo3Channel8bit(reconstructedDepth, w*h, depthFrameRGB);
	//reconstructed_depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);

	for (int i = 0; i < w*h*3; i++) {
		differenceDepthRGB[i] =
			abs(videoFrameRGB[i] - depthFrameRGB[i]);
	}
	reconstructed_depthOFImage.setFromPixels(differenceDepthRGB, w, h, OF_IMAGE_COLOR);*/

	this->applyNearAndFar(reconstructedDepth, w*h);

	pointCloudFromDepth(reconstructedDepth, realSenseDepthIntrinsics, xyz_pointCloud);

	fillVboMesh(w*h, xyz_pointCloud);
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (mode == MODE::DEVICE)
	{
		updateFromRealSense();

		spoutSender.send(depthOFImage.getTexture());
	}
	else 
	{
		updateFromVideoFile();
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    //ofBackground(200);

	//	TODO - Texture a pkane with our ofImage so we can render both it and the point cloud
	if (this->drawOption == DRAW_OPT::DEPTH)
	{
		depthOFImage.draw(0, 0);
		if (mode == MODE::VIDEOPLAYBACK) {
			reconstructed_depthOFImage.draw(w, 0);
		}
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

	//if (!hideGUI) {
		gui.draw();
		ofSetColor(255);
		ofDrawBitmapString("(Apply only with live sensor data,\nnot on loaded depth video files)", 12, 78);
	//}
    
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

	//
	//	Load depth video and seek to the first frame
	//	Now we are in PLAYBACK mode
	else if (key == 'l')
	{
		ofFileDialogResult dialogResult = ofSystemLoadDialog("Load a depth video file");
		if (dialogResult.bSuccess) {
			std::cout << depthVidPlayer.load(dialogResult.getPath()) << std::endl;
			depthVidPlayer.play();
			//depthVidPlayer.setPaused(true);
			depthVidPlayer.firstFrame();
			std::cout << depthVidPlayer.getWidth() << " " << depthVidPlayer.getHeight() << std::endl;
			std::cout << depthVidPlayer.getPixelsRef().getPixels() << std::endl;
			depthVidPlayer.draw(0, 0);
			mode = MODE::VIDEOPLAYBACK;
		}
	}
}
