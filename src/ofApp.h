#pragma once

#include "ofMain.h"
#include "ofxSpout.h"
//#include "ofVideoPlayer.h"
#include <librealsense2/rs.hpp>

class ofApp : public ofBaseApp{
public:
    void setup();
    void update();
    void draw();

	void keyReleased(int key);

	void realsenseUpdate();
    
    rs2::pipeline pipe;
    rs2::device* device;
    
    rs2::points points;
    rs2::pointcloud pc;
    
    ofVboMesh mesh;
    ofEasyCam cam;

	typedef struct RenderingDefs {
		GLfloat pointSize = 1;
	};
	RenderingDefs renderingDefs;
  
	ofxSpout::Sender spoutSender;

	ofTexture depthOFTexture;
	ofImage depthOFImage;

	ofImage reconstructed_depthOFImage;

protected:

	int w, h;

	unsigned char* depthFrameRGB;
	void convert16BitTo2Channel8bit(unsigned short* in, int in_size, unsigned char* out); // not used
	void convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out);
	void convert16BitTo3Channel8bit_mapped(unsigned short* in, int in_size, unsigned char* out, unsigned char** map);
	void convert3Channel8bitTo16Bit(unsigned char* in, int in_size, unsigned short* out);
	void convert3Channel8bitTo16Bit_unmapped(unsigned char* in, int out_size, unsigned short* out, unsigned char** map);

	//ofPixels depthPixels;

	typedef enum {
		DEPTH,
		POINTCLOUD
	} DRAW_OPT;
	DRAW_OPT drawOption = DRAW_OPT::DEPTH;

	///	Multiplier for the green channel of the output depth image.
	/// This is desirable because otherwise these always low values can't be retrieved on a recorded video
	int GMult = 1;	

	typedef enum {
		DEVICE,
		VIDEOPLAYBACK
	} MODE;
	MODE mode = MODE::DEVICE;

	ofVideoPlayer depthVidPlayer;

	bool testConversions_US_UC3Channels(bool logToConsole);

	rs2_intrinsics* realSenseDepthIntrinsics;
	void ofApp::writeRealSenseIntrinsics(const char* p_filepath);

	rs2::vertex* xyz_pointCloud;
	void pointCloudFromDepth(unsigned short* p_depth, rs2_intrinsics* p_intrinsics, rs2::vertex* xyz_out);

	void fillVboMesh(int p_npts, rs2::vertex* p_vertices);

	unsigned short* reconstructedDepth;
	unsigned char* differenceDepthRGB;

	//	4096 RGB colors
	unsigned char** colormap;

	ofFpsCounter fpsCounter;
};
