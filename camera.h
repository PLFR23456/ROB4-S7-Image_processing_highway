#include <opencv2/opencv.hpp>
#include <string>


using namespace cv;
using namespace std;

class Camera
{
public:
	Camera();
	
	bool open(std::string name);
	void play();
	bool close();
	bool generate_road_contours();
	int generate_car_box(float alpha,Mat* last_ref, Mat* ref);
	bool display_step_highway;
	bool display_step_car;
	
private:
	std::string m_fileName;
	VideoCapture m_cap;
	int m_fps;
	
		
	Mat m_frame;	
	Mat road_contour_frame;
    vector<vector<Point>> road_contours;
	vector<vector<Point>> car_contours;
};
