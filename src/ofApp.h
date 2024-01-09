#pragma once

#include "ofMain.h"
#include "ofxSpout.h"
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

protected:

	unsigned char* depthFrameRGB;
	void convert16BitTo2Channel8bit(unsigned short* in, int in_size, unsigned char* out); // not used
	void convert16BitTo3Channel8bit(unsigned short* in, int in_size, unsigned char* out);

	//ofPixels depthPixels;

	typedef enum {
		DEPTH,
		POINTCLOUD
	} DRAW_OPT;
	DRAW_OPT drawOption = DRAW_OPT::DEPTH;

	///	Multiplier for the green channel of the output depth image.
	/// This is desirable because otherwise these always low values can't be retrieved on a recorded video
	int GMult;	
};
