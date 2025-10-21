#include "camera.h"

int main(void)
{
	Camera myCam;
		
	myCam.open("cctv.avi");

	myCam.display_step_highway = false; // 'true' to show the contours construction // 'false' to skip
	myCam.display_step_car = true;

	myCam.generate_road_contours();

	myCam.play();
	
	
	myCam.close();
	
	return 1;
	
}
