#include <opencv2/opencv.hpp>
#include <string.h>

using namespace cv;
using namespace std;

class Camera
{
public:
	Camera();
	
	bool open(std::string name);
	void play();
	bool close();
	bool generate_road_contours(int display_construction);
	int generate_car_box();
	
private:
	std::string m_fileName;
	VideoCapture m_cap;
	int m_fps;
		
	Mat m_frame;	
	Mat road_contour_frame;
    vector<vector<Point>> road_contours;
	vector<vector<Point>> car_contours;
};
