#include "ofApp.h"
//#include "ofJson.h"	// actually uses nlohmann's json
#include "json.hpp"
#include "ofUtils.h"
#include "ofMath.h"


//	[Brizo] 14-01-2024
//	- Converts RealSense 16-bit depth stream to 8-bit/channel RGB and forwards via Spout...
//		- Green channel gets multiplied by a factor, because its range is too small
//	- ... so a third-party application records the depth video
//	- Creates point cloud from depth
//	- Save somewhere the value of the green channel multiplier (GMult) of a recording - review the GMult feature
//	- Read recorded depth videos and recover the depth values correctly [OK, but test for big depth spans] 
//	- From these videos, generate the point cloud for each frame and write the frames sequentially in a vertex-only .OBJ file
//		- And, of course, perform tests rendering point clouds from our OBJ files...
//	- Port this application to Kinect v2 (a.k.a. Kinect ONE)
//
//	Instructions: Check ofApp::keyReleased below

struct RGB {
	int r;
	int g;
	int b;
};

RGB htmlToRGB(std::string htmlColor) {
	if (htmlColor[0] == '#') {
		htmlColor.erase(0, 1); // remove the '#'
	}

	RGB rgb;
	std::stringstream ss;
	ss << std::hex << htmlColor;

	int color;
	ss >> color;

	rgb.r = (color >> 16) & 0xFF;
	rgb.g = (color >> 8) & 0xFF;
	rgb.b = color & 0xFF;

	return rgb;
}

void colormapFromFile(unsigned char** out_map)
{
	std::ifstream inputFile(ofToDataPath("", true) + "/colormap1004-html.txt");
	std::ofstream outputFile(ofToDataPath("", true) + "/colormap1004-rgb.txt");

	if (!inputFile.is_open() || !outputFile.is_open()) {
		std::cerr << "Unable to open file\n";
		std::exit(1);
	}

	int index = 0;
	std::string line;
	while (std::getline(inputFile, line)) {
		RGB rgb = htmlToRGB(line);
		out_map[index][0] = rgb.r;
		out_map[index][1] = rgb.g;
		out_map[index][2] = rgb.b;
		index++;
		outputFile << "R: " << rgb.r << ", G: " << rgb.g << ", B: " << rgb.b << "\n";
	}

	inputFile.close();
	outputFile.close();
}

void initColorMap(unsigned char** out_map)
{
	colormapFromFile(out_map);

	unsigned char** colormap_cpy = new unsigned char*[4096];
	for (int i = 0; i < 4096; i++) {
		colormap_cpy[i] = new unsigned char[3];
		//memcpy(colormap_cpy[i], out_map[i], 3);
		colormap_cpy[i][0] = out_map[i][0];
		colormap_cpy[i][1] = out_map[i][1];
		colormap_cpy[i][2] = out_map[i][2];
	}

	//	Applying a "zig-zag" specifically in this map (the "1004"), to ensure smoothness
	for (int i = 0; i < 4096; i++) {
		if ((i / 16) % 2 == 1) {
			out_map[i][0] = colormap_cpy[(i/16)*16 + 15 - div(i, 16).rem][0];
			out_map[i][1] = colormap_cpy[(i/16)*16 + 15 - div(i, 16).rem][1];
			out_map[i][2] = colormap_cpy[(i/16)*16 + 15 - div(i, 16).rem][2];
		}
	}
}

void createHSVSmoothMap(unsigned char** out_map)
{
	ofColor colors[4096];

	int h = 0, s = 255, v = 255;
	for (int i = 0; i < 4096; i++) {
		colors[i].setHsb(h, s, v);

		h = (h + 1) % 256;
		if (i % 256 == 0) {
			s -= 10;
			v -= 5;
		}
		out_map[i][0] = colors[i].r;
		out_map[i][1] = colors[i].g;
		out_map[i][2] = colors[i].b;
	}
}

void ofApp::setup()
{
	colormap = new unsigned char*[4096];
	for (int i = 0; i < 4096; i++) { colormap[i] = new unsigned char[3]; }
	//initColorMap(colormap);
	//createHSVSmoothMap(colormap);

	//	TESTS
	//	* Testing conversions used to transcode depth to video and vice-versa
	bool test_success = testConversions_US_UC3Channels(/*logToConsole*/ false);
	assert(test_success == true);
	

	device = new rs2::device();

	realSenseDepthIntrinsics = nullptr;

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

	//	FIXME - hard-coded. Please read from depthIntrinsicParams.json
	w = 848;
	h = 480;

	//	Spout - Initialise the Spout sender
	spoutSender.init("DepthVideo", w, h);// , OF_PIXELS_RGB);
	
	//depthFrameMono = new unsigned short[w*h];
	//depthFrameRG   = new unsigned char [w*h*2];
	depthFrameRGB  = new unsigned char [w*h*3];
	
	//	According to ofGraphicsConstants.h, OF_IMAGE_GRAYSCALE is expected to use OF_PIXELS_GRAY, which are 1 byte.
	//	In fact, allocate calls ofPixels::ofPixelFormatFromImageType(OF_IMAGE_GRAYSCALE), which returns OF_PIXELS_GRAY(which is 1 byte).
	//If we could use instead OF_PIXELS_GRAY_ALPHA we would be expected to use 2 channels(see ofImage::channelsFromPixelFormat)
	//	We decided to use a RGB image:
	depthOFImage.allocate(w, h, OF_IMAGE_COLOR);
	//depthPixels.allocate(848, 480, 2);

	reconstructed_depthOFImage.allocate(w, h, OF_IMAGE_COLOR);
	reconstructedDepth = new unsigned short[w*h];

	//	For tests
	differenceDepthRGB = new unsigned char[w*h*3];

	xyz_pointCloud = new rs2::vertex[w*h];
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
		out[i*3 + 0] = static_cast<unsigned char>(in[i] & 0xFF); // lower byte
		out[i*3 + 1] = GMult * static_cast<unsigned char>((in[i] >> 8) & 0xFF); // higher byte
		out[i*3 + 2] = 0;
	}
}

void ofApp::convert3Channel8bitTo16Bit(unsigned char* in, int out_size, unsigned short* out)
{
	for (int i = 0; i < out_size; i++) {
		out[i] = (unsigned short)(in[3*i + 1])*256/GMult + unsigned short(in[3*i]);
	}
}

void ofApp::convert16BitTo3Channel8bit_mapped(unsigned short* in, int in_size, unsigned char* out, unsigned char** map)
{
	for (int i = 0; i < in_size; i++) {
		out[i * 3 + 0] = map[min(4095, (int)in[i])][0];
		out[i * 3 + 1] = map[min(4095, (int)in[i])][1];
		out[i * 3 + 2] = map[min(4095, (int)in[i])][2];
	}
}

//	FIXME - This is for the map before the "zig-zag" (see initColorMap())
void ofApp::convert3Channel8bitTo16Bit_unmapped(unsigned char* in, int out_size, unsigned short* out, unsigned char** map)
{
	unsigned short i, j, k, n; // 3 indices of the LUT (color map) and the sequential (or flat) index
	unsigned short r, g, b;
	for (int i = 0; i < out_size; i++) {
		r = (unsigned short)in[3 * i];
		g = (unsigned short)in[3 * i + 1];
		b = (unsigned short)in[3 * i + 2];
		j = (255 - b) / 17;
		k = (255 - r) / 17;
		i = (1 - (k % 2)) * 15 + g * ((k % 2) * 2 - 1) / 17;
		out[i] = (256*k) + (16*j) + i;
	}
}

void ofApp::writeRealSenseIntrinsics(const char* p_filepath)
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

	/* Other parameters:
	depth_intrin = depth_profile.as<rs2::video_stream_profile>().get_intrinsics();
	color_intrin = color_profile.as<rs2::video_stream_profile>().get_intrinsics();
	color_extrin_to_depth = color_profile.as<rs2::video_stream_profile>().get_extrinsics_to(depth_profile);
	depth_extrin_to_color = depth_profile.as<rs2::video_stream_profile>().get_extrinsics_to(color_profile);
	
	params["color_to_depth_extrin"]["rotation0"] = color_extrin_to_depth.rotation[0];
	params["color_to_depth_extrin"]["rotation1"] = color_extrin_to_depth.rotation[1];
	params["color_to_depth_extrin"]["rotation2"] = color_extrin_to_depth.rotation[2];
	params["color_to_depth_extrin"]["rotation3"] = color_extrin_to_depth.rotation[3];
	params["color_to_depth_extrin"]["rotation4"] = color_extrin_to_depth.rotation[4];
	params["color_to_depth_extrin"]["rotation5"] = color_extrin_to_depth.rotation[5];
	params["color_to_depth_extrin"]["rotation6"] = color_extrin_to_depth.rotation[6];
	params["color_to_depth_extrin"]["rotation7"] = color_extrin_to_depth.rotation[7];
	params["color_to_depth_extrin"]["rotation8"] = color_extrin_to_depth.rotation[8];
	params["color_to_depth_extrin"]["translation0"] = color_extrin_to_depth.translation[0];
	params["color_to_depth_extrin"]["translation1"] = color_extrin_to_depth.translation[1];
	params["color_to_depth_extrin"]["translation2"] = color_extrin_to_depth.translation[2];
	
	params["color_intrin"]["width"] = color_intrin.width;
	params["color_intrin"]["height"] = color_intrin.height;
	params["color_intrin"]["ppx"] = color_intrin.ppx;
	params["color_intrin"]["ppy"] = color_intrin.ppy;
	params["color_intrin"]["fx"] = color_intrin.fx;
	params["color_intrin"]["fy"] = color_intrin.fy;
	params["color_intrin"]["model"] = color_intrin.model;
	params["color_intrin"]["coeffs0"] = color_intrin.coeffs[0];
	params["color_intrin"]["coeffs1"] = color_intrin.coeffs[1];
	params["color_intrin"]["coeffs2"] = color_intrin.coeffs[2];
	params["color_intrin"]["coeffs3"] = color_intrin.coeffs[3];
	params["color_intrin"]["coeffs4"] = color_intrin.coeffs[4];

	std::cout << "color width: " << color_intrin.width << std::endl;
	std::cout << "color height: " << color_intrin.height << std::endl;
	std::cout << "color ppx: " << color_intrin.ppx << std::endl;
	std::cout << "color ppy: " << color_intrin.ppy << std::endl;
	std::cout << "color fx: " << color_intrin.fx << std::endl;
	std::cout << "color fy: " << color_intrin.fy << std::endl;
	std::cout << "color model: " << color_intrin.model << std::endl;
	std::cout << "color coeffs: " << color_intrin.coeffs[0] << ", " << color_intrin.coeffs[1] << ", " << color_intrin.coeffs[2] << ", " << color_intrin.coeffs[3] << ", " << color_intrin.coeffs[4] << ", " << std::endl;

	std::cout << "color to depth extr rotation: " << color_extrin_to_depth.rotation[0] << ", " << color_extrin_to_depth.rotation[1] << ", " << color_extrin_to_depth.rotation[2] << ", " << color_extrin_to_depth.rotation[3] << ", " << color_extrin_to_depth.rotation[4] << ", " << color_extrin_to_depth.rotation[5] << ", " << color_extrin_to_depth.rotation[6] << ", " << color_extrin_to_depth.rotation[7] << ", " << color_extrin_to_depth.rotation[8] << std::endl;
	std::cout << "color to depth extr translation: " << color_extrin_to_depth.translation[0] << ", " << color_extrin_to_depth.translation[1] << ", " << color_extrin_to_depth.translation[2] << std::endl;

	params["depth_to_color_extrin"]["rotation0"] = depth_extrin_to_color.rotation[0];
	params["depth_to_color_extrin"]["rotation1"] = depth_extrin_to_color.rotation[1];
	params["depth_to_color_extrin"]["rotation2"] = depth_extrin_to_color.rotation[2];
	params["depth_to_color_extrin"]["rotation3"] = depth_extrin_to_color.rotation[3];
	params["depth_to_color_extrin"]["rotation4"] = depth_extrin_to_color.rotation[4];
	params["depth_to_color_extrin"]["rotation5"] = depth_extrin_to_color.rotation[5];
	params["depth_to_color_extrin"]["rotation6"] = depth_extrin_to_color.rotation[6];
	params["depth_to_color_extrin"]["rotation7"] = depth_extrin_to_color.rotation[7];
	params["depth_to_color_extrin"]["rotation8"] = depth_extrin_to_color.rotation[8];
	params["depth_to_color_extrin"]["translation0"] = depth_extrin_to_color.translation[0];
	params["depth_to_color_extrin"]["translation1"] = depth_extrin_to_color.translation[1];
	params["depth_to_color_extrin"]["translation2"] = depth_extrin_to_color.translation[2];

	std::cout << "depth to color extr rotation: " << depth_extrin_to_color.rotation[0] << ", " << depth_extrin_to_color.rotation[1] << ", " << depth_extrin_to_color.rotation[2] << ", " << depth_extrin_to_color.rotation[3] << ", " << depth_extrin_to_color.rotation[4] << ", " << depth_extrin_to_color.rotation[5] << ", " << depth_extrin_to_color.rotation[6] << ", " << depth_extrin_to_color.rotation[7] << ", " << depth_extrin_to_color.rotation[8] << std::endl;
	std::cout << "depth to color extr translation: " << depth_extrin_to_color.translation[0] << ", " << depth_extrin_to_color.translation[1] << ", " << depth_extrin_to_color.translation[2] << std::endl;

	std::cout << params << std::endl;

	Json::StreamWriterBuilder builder;
	builder["commentStyle"] = "None";
	builder["indentation"] = "\t";
	//builder["precision"] = 8; // set the precision to 8 digits
	builder["dropNullPlaceholders"] = false;
	*/
	
	std::ofstream f(std::string(p_filepath) + "/depthIntrinsicParams.json");
	f << params;
	f.flush();
}

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

	for (int i = 0; i < h; i++) {
		for (int j = 0; j < w; j++) {
			xyz_out[i*w + j].z = p_depth[i*w + j] / 1000.0;
			xyz_out[i*w + j].x = xyz_out[i*w + j].z * (i - p_intrinsics->ppx) / p_intrinsics->fx;
			xyz_out[i*w + j].y = xyz_out[i*w + j].z * (j - p_intrinsics->ppy) / p_intrinsics->fy;
		}
	}
}

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

void ofApp::realsenseUpdate()
{
	// Get depth data from camera
	auto frames = pipe.wait_for_frames();
	auto depth = frames.get_depth_frame();
	if (!depth) {
		return;
	}
	points = pc.calculate(depth);

	if (!realSenseDepthIntrinsics)
	{
		realSenseDepthIntrinsics = new rs2_intrinsics();
		*realSenseDepthIntrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

		std::cout << "RealSense device intrinsic parameters: " << std::endl;
		std::cout << "k1, k2, p1, p2, k3: " << realSenseDepthIntrinsics->coeffs[0] << " " << realSenseDepthIntrinsics->coeffs[1]
			<< " " << realSenseDepthIntrinsics->coeffs[2] << " " << realSenseDepthIntrinsics->coeffs[3] << " "
			<< realSenseDepthIntrinsics->coeffs[4] << std::endl;
		std::cout << "fx, fy: " << realSenseDepthIntrinsics->fx << " " << realSenseDepthIntrinsics->fy << std::endl;
		std::cout << "width, height: " << realSenseDepthIntrinsics->height << " " << realSenseDepthIntrinsics->height << std::endl;
		std::cout << "distortion model: " << realSenseDepthIntrinsics->model << std::endl;
		std::cout << "ppx, ppy: " << realSenseDepthIntrinsics->ppx << " " << realSenseDepthIntrinsics->ppy << std::endl;

		std::cout << "Writing json file to " << ofToDataPath("", true) << std::endl;

		writeRealSenseIntrinsics(ofToDataPath("",true).c_str());
	}
	//std::cout << depth.get_bytes_per_pixel() << std::endl;
	//std::cout << depth.get_stride_in_bytes() << std::endl;
	//std::cout << "-----" << std::endl;
	//depth.get_profile().format()

	fillVboMesh(points.size(), (rs2::vertex*) points.get_vertices());

	w = depth.get_width();
	h = depth.get_height();

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

void ofApp::update()
{
	if (mode == MODE::DEVICE)
	{
		realsenseUpdate();

		spoutSender.send(depthOFImage.getTexture());
	}
	else {
		unsigned char* videoFrameRGB = depthVidPlayer.getPixelsRef().getPixels();
		depthOFImage.setFromPixels(videoFrameRGB, 1280, 720, OF_IMAGE_COLOR);
		depthOFImage.crop(0, 0, 848, 480);
		
		/*int icropped = 0;
		for (int i = 0; i < 848*480*3; i++) {
			icropped = i + (i / (848 * 3)) * 432 * 3;
			depthFrameRGB[i] =   videoFrameRGB[icropped];
			depthFrameRGB[i+1] = videoFrameRGB[icropped+1];
			depthFrameRGB[i+2] = videoFrameRGB[icropped+2];
		}
		depthOFImage.setFromPixels(depthFrameRGB, w, h, OF_IMAGE_COLOR);*/

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

		int w = realSenseDepthIntrinsics->width;
		int h = realSenseDepthIntrinsics->height;

		pointCloudFromDepth(reconstructedDepth, realSenseDepthIntrinsics, xyz_pointCloud);

		fillVboMesh(w*h, xyz_pointCloud);
	}
}

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

		//	TO-DO - And read camera intrinsics from file
	}
}
