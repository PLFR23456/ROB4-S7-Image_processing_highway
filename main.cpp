#include "camera.h"

int main(void)
{
	Camera myCam;
		
	myCam.open("cctv.avi");

	bool display_construction = false; // 'true' to show the contours construction // 'false' to skip
	myCam.generate_road_contours(display_construction);

	myCam.play();
	
	
	myCam.close();
	
	return 1;
	
}
