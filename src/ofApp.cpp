#include "ofApp.h"
//#include "ofJson.h"	// actually uses nlohmann's json
#include "librealsense2/rsutil.h"
#include "librealsense2/h/rs_option.h"
#include "librealsense2/h/rs_sensor.h"
#include "librealsense2/h/rs_processing.h"
#include "json.hpp"
#include "ofUtils.h"
#include "ofMath.h"

#include "Device.h"

#include <stdarg.h>

//	[Brizo] 04-01-2024 - 26-01-2024
//	This is a LabPresenca's tool for making Tore Virtual, a VR movie that documents an indigenous ritual in a transfigured way,
//in co-creation with Fulni-O people
//	Tore Virtual is on the making thanks to a Funarte (Fundacao Nacional das Artes, brazilian Ministry of Culture) public prize.
//	
//	What this does:
//	- Applies Near/Far clipping in depth video from RealSense D455
//	- Converts the RealSense depth video (it's a 16-bit depth stream) to 8-bit/channel RGB and put the color stream alongside it - making a 2x wide video -,
//and forwards this video via Spout...
//	- ... so a third-party application records this video (lossless, RGB! Up until now, point cloud didn't survive the slightest encoding of the depth stream)
//	- Creates point cloud from depth (either device depth stream or video), colored from the color stream (if live) or the color portion of the video file
//	- Allows writing to file the point cloud generated from the video file, in a vertex-only .PLY file per frame 
//		- With a nice clipping cuboid (=rectangular prism) - frustrum would be nice, but let's keep it simple...
//	
//	Coming Soon:
//	- Support for Kinect v2 (a.k.a. Kinect ONE), by Anthony Bet (@AnthonyAposta)

//
//	Instructions are on screen (or check ofApp::keyReleased below)

void ofApp::setup()
{


	mode = MODE::DEVICE;

	nearFar_Gui.setup("Near/Far clipping");
	nearFar_Gui.add(near_US.set("Near", 417, 0, 512));
	nearFar_Gui.add(far_US.set("Far", 793, 256, 4096));

	cropPointCloud_Gui.setup("Crop points to export", "crop_settings.xml");

#ifdef LP_KINECTV2
	cropPointCloud_Gui.add(nearMeters.set("Near", 0.5, 0.5, 4.5));
	cropPointCloud_Gui.add(farMeters.set("Far", 4.5, 0.5, 4.5));

	cropPointCloud_Gui.add(lowMeters.set("Low", 2.5, -2.5, 2.5));
	cropPointCloud_Gui.add(highMeters.set("High", -2.5, -2.5, 2.5));

	cropPointCloud_Gui.add(leftMeters.set("Left", -2.5, -2.5, 2.5));
	cropPointCloud_Gui.add(rightMeters.set("Right", 2.5, -2.5, 2.5));
#else 
#ifdef LP_REALSENSE_D455
	cropPointCloud_Gui.add(lowMeters.set("Down", 2.0, -4.0, 4.0));
	cropPointCloud_Gui.add(highMeters.set("Up", -2.0, -4.0, 4.0));

	cropPointCloud_Gui.add(farMeters.set("Far", 3.0, -6.0, 6.0));
	cropPointCloud_Gui.add(nearMeters.set("Near", -3.0, -6.0, 6.0));

	cropPointCloud_Gui.add(rightMeters.set("Right", -2.0, -4.0, 4.0));
	cropPointCloud_Gui.add(leftMeters.set("Left", 2.0, -4.0, 4.0));
#endif // LP_REALSENSE_D455
#endif // LP_KINECTV2

	//	TESTS
	//	* Testing conversions used to transcode depth to video and vice-versa
	bool test_success = testConversions_US_UC3Channels(/*logToConsole*/ false);
	assert(test_success == true);

	ofSetFrameRate(30);
	ofSetVerticalSync(true);
	ofSetLogLevel(OF_LOG_NOTICE);

	cam.setGlobalPosition(glm::vec3(4, 0, 0));
	cam.lookAt(glm::vec3(0, 0, 0), glm::vec3(0, -1, 0));

	pointCloud.setMode(OF_PRIMITIVE_POINTS);

	glEnable(GL_POINT_SMOOTH); // use circular points instead of square points
	glPointSize(renderingDefs.pointSize); // make the points bigger

	//
	//	RealSense camera
	
	realSenseDepthIntrinsics = nullptr;
	alignToDepth = new rs2::align(rs2_stream::RS2_STREAM_DEPTH);
#ifndef LP_LIBREALSENSE_JOHN
	rs2::config cfg;
	cfg.enable_device("231122302600"); // NOTE: This HARD-CODED number is the serial number of your device
	cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);	//	higher resolution because we'll align it to the depth frame and generate coloured point cloud
	cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 30);	//	The optimal depth resolution of the D455 device is 848x480
	
	pipelineProfile = pipe.start(cfg);	//	if we use custom configuration: pipe.start(cfg);
#else
	ofDisableArbTex();
	
	//	TODO - Passar para o init
	//spoutSenderDepth.init("DepthVideo", 848, 480);// , OF_PIXELS_RGB);
	//spoutSenderRGB.init("RGBVideoRegistered", 848, 480);

	//this->guiPanel.setup("settings.xml");

	device = nullptr;

	this->eventListeners.push(this->context.deviceAddedEvent.newListener([&](std::string serialNumber)
	{
		ofLogNotice(__FUNCTION__) << "Starting device " << serialNumber;
		std::cout << "Starting device " << serialNumber << std::endl;
		device = this->context.getDevice(serialNumber);
		device->enableDepth(848, 480, 30);
		device->enableColor(1280, 720, 30);
		device->enablePoints();

		device->startPipeline();	//	Here the default parameters are set

		device->alignMode.set("Align", ofxRealSense2::Device::Align::Depth);
		//this->guiPanel.add(device->params);
	}));

	try
	{
		this->context.setup(false);
	}
	catch (std::exception& e)
	{
		ofLogFatalError(__FUNCTION__) << e.what();
	}

	std::cout << "3" << std::endl;

#endif 
}

//--------------------------------------------------------------
///	Set up everything that demanded knowledge about the stream
void ofApp::finishSetup()
{
	//	Spout - It shares the video memory with other Spout-enabled applications. We use for recording the depth stream
	spoutSenderDepth.init("DepthVideo", w, h);// , OF_PIXELS_RGB);
	spoutSenderRGB.init("RGBVideoRegistered", w, h);

	//RGB_OFPixels.allocate(w, h, OF_IMAGE_COLOR);
	colorOFImage.allocate(w, h, OF_IMAGE_COLOR);

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
		out[i * 2] = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
		out[i * 2 + 1] = static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
	}
}

//--------------------------------------------------------------
void ofApp::convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out)
{
	for (int i = 0; i < in_size; i++) {
		out[i * 3 + 0] = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
		out[i * 3 + 1] = /*GMult **/ static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
		out[i * 3 + 2] = 0;
	}
}

//--------------------------------------------------------------
void ofApp::convert3Channel8bitTo16Bit(unsigned char* in, int out_size, unsigned short* out)
{
	for (int i = 0; i < out_size; i++) {
		out[i] = (unsigned short)(in[3 * i + 1]) * 256 + unsigned short(in[3 * i]);
		//if (out[i] != 0)std::cout << out[i] << std::endl;
		//out[i] = (unsigned short)(in[3 * i + 2]) * 256 + unsigned short(in[3 * i + 1]);
		//out[i] = (unsigned short)(in[3 * i]) * 256 + unsigned short(in[3 * i + 1]);
		//out[i] = (unsigned short)(in[3 * i + 1]) * 256 + unsigned short(in[3 * i + 2]);
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
		out[i] = (256 * k) + (16 * j) + i;
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
	std::string fullpath = std::string(p_filepath) + this->FNAME_INTRINSIC_PARAMS;

	std::cout << "Writing RealSense device depth-related intrinsic parameters to " << fullpath << std::endl;

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

	std::ofstream f(fullpath);
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
//#ifndef LP_LIBREALSENSE_JOHN
void ofApp::writeDeviceInfo(std::string p_filepath)
{
	float fov[2];
	rs2_fov(realSenseDepthIntrinsics, &fov[0]);

	nlohmann::json params;

	char s[256];
	sprintf(s, "%f %f", fov[0], fov[1]);
	params["info"]["fov"] = std::string(s);

	params["info"]["advanced_mode"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_ADVANCED_MODE));
	params["info"]["locked"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_CAMERA_LOCKED));
	params["info"]["op_code"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_DEBUG_OP_CODE));
	params["info"]["firmware_version"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_FIRMWARE_VERSION));
	params["info"]["camera_name"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME));
	params["info"]["product_id"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_PRODUCT_ID));
	params["info"]["serial"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_SERIAL_NUMBER));
	params["info"]["usb_compatibility"] = std::string(device->getNativeDevice().get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR));

	std::ofstream f(std::string(p_filepath) + this->FNAME_DEVICE_INFO);
	f << params;
	f.flush();
}
//#endif

//--------------------------------------------------------------
///	Slight adaptation of rsutil.h's static void rs2_deproject_pixel_to_point(float point[3], const struct rs2_intrinsics * intrin, const float pixel[2], float depth)
///	NOTE - it's different from https://github.com/IntelRealSense/librealsense/blob/v2.24.0/wrappers/python/examples/box_dimensioner_multicam/helper_functions.py#L121-L147
///	NOTE - Includes the zero-depth elements (i.e., points where depth couldn't be estimated). We left to you to get rid of them...
void ofApp::pointCloudFromDepth(unsigned short* p_depth, rs2_intrinsics* p_intrinsics, rs2::vertex* xyz_out)
{
	assert(p_intrinsics->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
	assert(p_intrinsics->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
	//assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

	float f, x, y, r2 = 0;
	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			x = (i - p_intrinsics->ppx) / p_intrinsics->fx;
			y = (j - p_intrinsics->ppy) / p_intrinsics->fy;
			r2 = x * x + y * y;
			f = 1 + p_intrinsics->coeffs[0] * r2 + p_intrinsics->coeffs[1] * r2*r2 + p_intrinsics->coeffs[4] * r2*r2*r2;
			xyz_out[i*w + j].x = x * f + 2 * p_intrinsics->coeffs[2] * x*y + p_intrinsics->coeffs[3] * (r2 + 2 * x*x);
			xyz_out[i*w + j].y = y * f + 2 * p_intrinsics->coeffs[3] * x*y + p_intrinsics->coeffs[2] * (r2 + 2 * y*y);
			xyz_out[i*w + j].z = p_depth[i*w + j] / 1000.0;
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
void ofApp::fillVboMesh(int p_npts, rs2::vertex* p_vertices, const unsigned char* p_colors)
{
	// Create oF mesh
	pointCloud.clear();

	glm::vec3 v3;

	if (p_npts != 0)
	{
		for (int i = 0; i < p_npts; i++)
		{
			v3.x = p_vertices[i].x;
			v3.y = p_vertices[i].y;
			v3.z = p_vertices[i].z;

			if ((v3.z > nearMeters.get()) && (v3.z < farMeters.get())) {
				if ((v3.x > rightMeters.get()) && (v3.x < leftMeters.get())) {
					if ((v3.y > highMeters.get()) && (v3.y < lowMeters.get())) {
						pointCloud.addVertex(v3);
						pointCloud.addColor(ofFloatColor(p_colors[i * 3] / 256.0, p_colors[i * 3 + 1] / 256.0, p_colors[i * 3 + 1] / 256.0, 0.8));
					}
				}
			}
		}
	}
}

//--------------------------------------------------------------
#ifdef LP_REALSENSE_D455
///	NOTE - The point cloud vertices calculated live (i.e., mode == MODE::DEVICE) at rs2::pointcloud::calculate(depth) are different from the ones
///we calculate from the depth stream in ofApp::pointCloudFromDepth. TODO - Discover WHY
void ofApp::fillVboMesh_transformed(int p_npts, rs2::vertex* p_vertices, const unsigned char* p_colors)
{
	// Create oF mesh
	pointCloud.clear();

	glm::vec3 v3;

	if (p_npts != 0)
	{
		for (int i = 0; i < p_npts; i++)
		{
			//	Read NOTE above
			v3.x = p_vertices[i].y;
			v3.y = p_vertices[i].x;
			v3.z = p_vertices[i].z;

			if ((v3.z > nearMeters.get()) && (v3.z < farMeters.get())) {
				if ((v3.x > rightMeters.get()) && (v3.x < leftMeters.get())) {
					if ((v3.y > highMeters.get()) && (v3.y < lowMeters.get())) {
						pointCloud.addVertex(v3);
						pointCloud.addColor(ofFloatColor(p_colors[i * 3] / 256.0, p_colors[i * 3 + 1] / 256.0, p_colors[i * 3 + 1] / 256.0, 0.8));
					}
				}
			}
		}
	}
}
#endif // LP_REALSENSE_D455

//--------------------------------------------------------------
void ofApp::updateFromRealSense()
{
#ifdef LP_LIBREALSENSE_JOHN
	if (!device) {
		return;
	}
	this->context.update();

	frameset = device->frameset;
	rs2::depth_frame depth = frameset.get_depth_frame();
#else
	// Get depth data from camera
	frameset = pipe.wait_for_frames();
	auto depth = frameset.get_depth_frame();
#endif

	if (!depth) {
		return;
	}

	
#ifndef LP_LIBREALSENSE_JOHN // already aligned in context.update()
	//	Here the color frame is aligned to the depth frame, and, hence, resized to w x h
	frameset = alignToDepth->process(frameset);
#endif

	frameset = alignToDepth->process(frameset);

	bool deviceInfoLogged = (realSenseDepthIntrinsics != nullptr);
	if (!deviceInfoLogged)
	{
		realSenseDepthIntrinsics = new rs2_intrinsics();
		*realSenseDepthIntrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

		writeRealSenseIntrinsics(ofToDataPath("", true));

#ifndef LP_LIBREALSENSE_JOHN
		device = pipelineProfile.get_device();
#endif

		writeDeviceInfo(ofToDataPath("", true));

		//	The option below doesn't work. My RS Viewer (v2.54.1) doesn't save the resolution I set there in the Custom preset
		//	Use the custom preset you should have saved with the SDK's Intel RealSense Viewer
		/*if (device.query_sensors()[0].supports(rs2_option::RS2_OPTION_VISUAL_PRESET)) {
			device.query_sensors()[0].set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_CUSTOM);
			std::cout << "RS2_RS400_VISUAL_PRESET_CUSTOM should be set now..." << std::endl;
			//std::cout << device.query_sensors()[0].get_option(rs2_option::RS2_OPTION_AUTO_EXPOSURE_MODE);
			//std::cout << device.query_sensors()[0].get_option(rs2_option::RS2_OPTION_GAIN);
			//std::cout << device.query_sensors()[0].get_option(rs2_option::RS2_OPTION_LASER_POWER);
		}*/
		w = depth.get_width();
		h = depth.get_height();

		finishSetup();
	}

	this->applyNearAndFar((unsigned short*)(depth.get_data()), w*h);

	points = pc.calculate(depth);

	const unsigned char* colorStream = static_cast<const unsigned char*> (frameset.get_color_frame().get_data());
	colorOFImage.setFromPixels(colorStream, w, h, OF_IMAGE_COLOR);

	fillVboMesh/*_transformed*/(points.size(), (rs2::vertex*) points.get_vertices(), colorStream);

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

	spoutSenderDepth.send(depthOFImage.getTexture());
	spoutSenderRGB.send(colorOFImage.getTexture());
}

//--------------------------------------------------------------
void ofApp::updateFromVideoFile()
{
	if (!realSenseDepthIntrinsics)
	{
		realSenseDepthIntrinsics = new rs2_intrinsics();

		readRealSenseIntrinsics(ofToDataPath("", true) + this->FNAME_INTRINSIC_PARAMS);

		w = realSenseDepthIntrinsics->width;
		h = realSenseDepthIntrinsics->height;

		finishSetup();
	}

	if (mode == MODE::RECORDING_POINTCLOUD) {
		depthVidPlayer.setPaused(true);
	}
	else {
		depthVidPlayer.update();
	}

	//	Video file has depth and color images sibe by side
	unsigned char* videoFrameRGB = depthVidPlayer.getPixelsRef().getPixels();
	//	This is why both depthOFImage and colorOFImage get the whole frame, with w*2 width...
	depthOFImage.setFromPixels(videoFrameRGB, w * 2, h, OF_IMAGE_COLOR);
	colorOFImage.setFromPixels(videoFrameRGB, w * 2, h, OF_IMAGE_COLOR);
	//	... and then they are cropped according to the desired part:
	depthOFImage.crop(0, 0, w, h);	//	left half: depth
	colorOFImage.crop(w, 0, w, h);	//	right half: color

#ifdef LP_REALSENSE_D455
	convert3Channel8bitTo16Bit(depthOFImage.getPixelsRef().getData(), w*h, reconstructedDepth);

	/*
	//	Converting rgb read from video to unsigned short and then back to RGB to test the RGB->US conversion
	this->convert16BitTo3Channel8bit(reconstructedDepth, w*h, depthFrameRGB);
	//reconstructed_depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);

	for (int i = 0; i < w*h*3; i++) {
		differenceDepthRGB[i] =
			abs(videoFrameRGB[i] - depthFrameRGB[i]);
	}
	reconstructed_depthOFImage.setFromPixels(differenceDepthRGB, w, h, OF_IMAGE_COLOR);*/

	//applyNearAndFar(reconstructedDepth, w*h);

	pointCloudFromDepth(reconstructedDepth, realSenseDepthIntrinsics, xyz_pointCloud);

	fillVboMesh_transformed(w*h, xyz_pointCloud, colorOFImage.getPixelsRef().getData());
#else
#ifdef LP_KINECTV2
	convert3Channel8bitTo32bit(depthOFImage.getPixelsRef().getData(), w * h, reconstructedDepth);
	applyNearAndFar(reconstructedDepth, w*h);
	fillVboMesh(revertedRawPixelsInt, RGBOFImage.getPixelsRef(), h, w);
#endif
#endif

	if (mode == MODE::RECORDING_POINTCLOUD)
	{
		// Save mesh
		//std::string fullPath = savePLYDialog.filePath + "\\PLY_data_" + to_string(depthVidPlayer.getCurrentFrame()) + ".ply";
		std::string fullPath = ofToDataPath("", true) + "\\PLY_data_" + to_string(depthVidPlayer.getCurrentFrame()) + ".ply";
		std::cout << "Saving point cloud in " << fullPath << ". Total frames: " << depthVidPlayer.getTotalNumFrames() << endl;
		pointCloud.save(fullPath);

		if (depthVidPlayer.getCurrentFrame() == (depthVidPlayer.getTotalNumFrames() - 1)) {
			std::cout << "Point cloud sequence saved!" << std::endl;
			mode = MODE::DEVICE;
		}
		else {
			//	And advance solely 1 frame on video
			depthVidPlayer.setPaused(false);
			depthVidPlayer.nextFrame();
			depthVidPlayer.update();
		}
	}

}

//--------------------------------------------------------------
void ofApp::update()
{
	if (mode == MODE::DEVICE)
	{
		updateFromRealSense();
	}
	else if (mode != MODE::POINTCLOUD_PLAYBACK)
	{
		updateFromVideoFile();
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	//ofBackground(200);

	std::string instructions;
	if (mode == MODE::DEVICE) {
		instructions.append("DEVICE (LIVE) MODE\n[Space Bar]: Alternate depth+color / point cloud visualization\n[L]: Load video recording (go to video playback mode)");
		if (!device) {
			int rightMaringGUIOffset = ofGetWidth() - 520;
			ofDrawBitmapStringHighlight(instructions, rightMaringGUIOffset, 20);
			return;
		}
	}
	else if (mode == MODE::VIDEOPLAYBACK) {
		instructions.append("VIDEO PLAYBACK MODE\n[Space Bar]: Alternate depth+color / point cloud visualization\n[S]: Save OBJ files\n[D]: Back to device mode");
	}

	//	TODO - Texture a plane with our ofImage so we can render both it and the point cloud
	if (this->drawOption == DRAW_OPT::DEPTH)
	{
		depthOFImage.draw(0, 0);
		colorOFImage.draw(w, 0);

		ofDrawBitmapString("(Apply only with live sensor data,\nnot on loaded depth video files)", 12, 78);
		nearFar_Gui.draw();
	}
	else
	{
		//	Point cloud
		cam.begin();
		float s = 100;
		ofScale(s, s, s);
		ofDrawAxis(1);

		ofSetColor(255, 128);

		ofPushMatrix();
		ofRotateY(90);
		ofDrawGridPlane(1, 5, true);
		ofPopMatrix();

		ofPushMatrix();
		//ofRotateX(-90);
		pointCloud.drawVertices();
		pointCloud2.drawVertices();
		ofPopMatrix();

		// Referece cube

		box.depth = abs(farMeters.get() - nearMeters.get());
		box.width = abs(leftMeters.get() - rightMeters.get());
		box.height = abs(highMeters.get() - lowMeters.get());

		box.posZ = (box.depth / 2) + nearMeters.get();
		box.posX = (box.width / 2) + rightMeters.get();
		box.posY = (box.height / 2) + highMeters.get();

		ofNoFill();
		ofSetLineWidth(4.0);	//	Not supported, may not work
		ofSetColor(255, 0, 255);
		ofDrawBox(box.posX, box.posY, box.posZ, box.width, box.height, box.depth);
		ofFill();

		cam.end();

		ofSetColor(255);

		cropPointCloud_Gui.draw();

		if (mode != MODE::RECORDING_POINTCLOUD) {
			instructions.append("\n\nNAVIGATION:\n[left click+drag]: Rotate\n[mouse wheel]: Zoom\n[left click+drag]+[m]: Move");
		}
	}

	if (mode == MODE::RECORDING_POINTCLOUD) {
		instructions.append("RECORDING TO .PLY FILES. Please wait until finish");
	}

	int rightMaringGUIOffset = ofGetWidth() - 520;
	ofDrawBitmapStringHighlight(instructions, rightMaringGUIOffset, 20);
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
	//
	//	Some tests on the point cloud visualization:

	if (key == ofKey::OF_KEY_RIGHT) {
		pointCloud.setMode(OF_PRIMITIVE_POINTS);
	}
	else if (key == ofKey::OF_KEY_LEFT) {
		pointCloud.setMode(OF_PRIMITIVE_LINES);
	}
	else if (((key - 48) >= 0) && ((key - 48) < 10)) {
		pointCloud.setMode(ofPrimitiveMode(key - 48));
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

	/*else if (key == ofKey::OF_KEY_UP) {
		GMult++;
		std::cout << "Green channel multiplier: " << GMult << std::endl;
	}
	else if (key == ofKey::OF_KEY_DOWN) {
		if (GMult > 1) {
			GMult--;
			std::cout << "Green channel multiplier: " << GMult << std::endl;
		}
	}*/

	//
	//	Draw option (FIXME - Remove this option and draw everything in 3d)
	else if (key == 'v') {
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
	else if (key == 'd') {
		mode = MODE::DEVICE;
	}
	else if (key == 's') {
		if (mode == MODE::VIDEOPLAYBACK && depthVidPlayer.isLoaded())
		{
			mode = MODE::RECORDING_POINTCLOUD;

			std::cout << "Started recording point cloud animation to sequence of .PLY files..." << std::endl;

			//savePLYDialog = ofSystemLoadDialog("Save point cloud animation to sequence of .PLY files", true);	Crashes on some Win10 tests!
			//if (savePLYDialog.bSuccess) {
			depthVidPlayer.firstFrame();
			//}
		}
	}
	else if (key == 'f') {
		if (mode == MODE::DEVICE) {
			mode = MODE::POINTCLOUD_PLAYBACK;
			ofFileDialogResult dialogResult = ofSystemLoadDialog("Test mesh 1");
			if (dialogResult.bSuccess) {
				pointCloud.load(dialogResult.getPath());
			}
			dialogResult = ofSystemLoadDialog("Test mesh 2");
			if (dialogResult.bSuccess) {
				pointCloud2.load(dialogResult.getPath());
			}
		}
	}

	std::cout << ofGetFrameRate() << "FPS" << std::endl;
}

void ofApp::exit()
{
	this->context.clear();
}