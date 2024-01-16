///
///	By now, an array of 4096 RGB colors
///	Objective: create color maps (with a reversion rule - color to depth) that make the generated point cloud
///survive lossy video encodings. No success by now
///
///	15-01-2024 [Brizo] For now, barely used.
/// https://colorspace.r-forge.r-project.org/ (there's a python version) and
///	https://www.ccctool.com/html_v_0_9_0_3/CCC_Tool/cccTool.html may be useful
class ColorMap
{
protected:

	unsigned char** data; // [4096][3];

	struct RGB {
		int r;
		int g;
		int b;
	};

	//
	//	The functions below are very specific for a color map written in a file as a "#rrggbb" (HTML) color per line

	inline RGB htmlToRGB(std::string htmlColor) {
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

	inline void colormapFromFile1(unsigned char** out_map)
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
			outputFile << rgb.r << " " << rgb.g << " " << rgb.b << std::endl;
		}

		inputFile.close();
		outputFile.close();
	}

	inline void initColorMap_unzigzag()
	{
		colormapFromFile1(data);

		unsigned char** colormap_cpy = new unsigned char*[4096];
		for (int i = 0; i < 4096; i++) {
			colormap_cpy[i] = new unsigned char[3];
			//memcpy(colormap_cpy[i], out_map[i], 3);
			colormap_cpy[i][0] = data[i][0];
			colormap_cpy[i][1] = data[i][1];
			colormap_cpy[i][2] = data[i][2];
		}

		//	Applying a "zig-zag" specifically in this map (the "1004"), to ensure smoothness
		for (int i = 0; i < 4096; i++) {
			if ((i / 16) % 2 == 1) {
				data[i][0] = colormap_cpy[(i / 16) * 16 + 15 - div(i, 16).rem][0];
				data[i][1] = colormap_cpy[(i / 16) * 16 + 15 - div(i, 16).rem][1];
				data[i][2] = colormap_cpy[(i / 16) * 16 + 15 - div(i, 16).rem][2];
			}
		}
	}

public:

	ColorMap() {
		data = new unsigned char*[4096];
		for (int i = 0; i < 4096; i++) { 
			data[i] = new unsigned char[3]; 
		}
		initColorMap_unzigzag();
	}

	~ColorMap() {
		for (int i = 0; i < 4096; i++) {
			delete data[i];
		}
		delete[] data;
	}
};
