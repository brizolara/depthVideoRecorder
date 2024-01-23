#pragma once

#define LP_REALSENSE_D455

#include "ofMain.h"
#include "ofxSpout.h"
#include "ofxGui.h"
#include <librealsense2/rs.hpp>
#include "colormaps/ColorMap.h"

class ofApp : public ofBaseApp{
public:
    void setup();
	void finishSetup();
    
	void update();
	void updateFromRealSense();
	void updateFromVideoFile();

	void draw();

	void keyReleased(int key);

	
    rs2::pipeline pipe;
	rs2::pipeline_profile pipelineProfile;
    rs2::device device;

	rs2::align* alignToDepth;
    
    rs2::points points;
    rs2::pointcloud pc;
    
    ofVboMesh pointCloud;
	ofVboMesh pointCloud2;
    ofEasyCam cam;

	typedef struct RenderingDefs {
		GLfloat pointSize = 1;
	};
	RenderingDefs renderingDefs;
  
	ofxSpout::Sender spoutSenderDepth;
	ofxSpout::Sender spoutSenderRGB;

	ofImage depthOFImage;
	ofImage colorOFImage;

	ofImage reconstructed_depthOFImage;

protected:

	int w, h;

	unsigned char* depthFrameRGB;
	void convert16BitTo2Channel8bit(unsigned short* in, int in_size, unsigned char* out); // not used
	void convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out);
	void convert16BitTo3Channel8bit_mapped(unsigned short* in, int in_size, unsigned char* out, unsigned char** map);
	void convert3Channel8bitTo16Bit(unsigned char* in, int in_size, unsigned short* out);
	void convert_mapped3Channel8bitTo16Bit(unsigned char* in, int out_size, unsigned short* out, unsigned char** map);
	void applyNearAndFar(unsigned short* data, int size);

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
		VIDEOPLAYBACK,
		RECORDING_POINTCLOUD,
		POINTCLOUD_PLAYBACK
	} MODE;
	MODE mode;

	ofVideoPlayer depthVidPlayer;

	bool testConversions_US_UC3Channels(bool logToConsole);

	const std::string FNAME_INTRINSIC_PARAMS = "/depthIntrinsicParams.json";
	rs2_intrinsics* realSenseDepthIntrinsics;
	void writeRealSenseIntrinsics(std::string p_filepath);
	void readRealSenseIntrinsics(std::string p_filepath);

	const std::string FNAME_DEVICE_INFO = "/device_info.json";
	void writeDeviceInfo(std::string p_filepath);

	rs2::vertex* xyz_pointCloud;
	void pointCloudFromDepth(unsigned short* p_depth, rs2_intrinsics* p_intrinsics, rs2::vertex* xyz_out);

	void fillVboMesh(int p_npts, rs2::vertex* p_vertices, const unsigned char* p_colors);
	#ifdef LP_REALSENSE_D455
	void fillVboMesh_transformed(int p_npts, rs2::vertex* p_vertices, const unsigned char* p_colors);
	#endif

	unsigned short* reconstructedDepth;
	unsigned char* differenceDepthRGB;

	//	4096 RGB colors
	ColorMap colormap;

	ofFpsCounter fpsCounter;

	//	Near and Far planes of distance coded as unsigned short
	ofParameter<unsigned short> far_US;
	ofParameter<unsigned short> near_US;

	typedef struct CropBox
	{
		float depth, width, height, posZ, posX, posY;
	};
	CropBox box;

	//	Point cloud cropping planes
	ofParameter<float> nearMeters;
	ofParameter<float> farMeters;

	ofParameter<float> highMeters;
	ofParameter<float> lowMeters;

	ofParameter<float> leftMeters;
	ofParameter<float> rightMeters;

	ofxPanel nearFar_Gui;
	ofxPanel cropPointCloud_Gui;

	
	ofFileDialogResult savePLYDialog;
};
