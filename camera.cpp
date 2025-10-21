#include "camera.h"
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/imgproc.hpp>
#include <unistd.h>


Camera::Camera()
{
	m_fps = 30;
}

bool Camera::open(std::string filename)
{
	m_fileName = filename;
	
	// Convert filename to number if you want to
	// open webcam stream
	std::istringstream iss(filename.c_str());
	int devid;
	bool isOpen;
	if(!(iss >> devid))
	{
		isOpen = m_cap.open(filename.c_str());
	}
	else 
	{
		isOpen = m_cap.open(devid);
	}
	
	if(!isOpen)
	{
		std::cerr << "Unable to open video file." << std::endl;
		return false;
	}
	
	// set framerate, if unable to read framerate, set it to 30
	m_fps = m_cap.get(cv::CAP_PROP_FPS);
	if(m_fps == 0)
		m_fps = 30;
	return true;
}
bool Camera::generate_road_contours(int display_construction){
	    
	if (!m_cap.isOpened()) {
        std::cerr << "Camera not opened!" << std::endl;
        return false;
    }
	
	namedWindow("Video", cv::WINDOW_AUTOSIZE);
	m_cap.read(m_frame);
	m_cap.read(m_frame);
	m_cap.read(m_frame);
	Mat cloneFrame = m_frame.clone();
	// 0. GaussianBlur (avoir un filtre plutot lisse)
	GaussianBlur(cloneFrame,cloneFrame,Size(5,5),1.5);

	Mat hsv;
	cvtColor(cloneFrame, hsv, COLOR_BGR2HSV);

	// 1. on recupere le canal saturation
	vector<Mat> hsvChannels;
	split(hsv, hsvChannels);
	Mat S = hsvChannels[1];
	if(display_construction)imshow("Saturation",S);	

	Mat grayMask;

	// 2. On garde l'image lorsque la saturation est inférieur à satmax
	int satmax = 50 ;
	threshold(S, grayMask, satmax, 255 , THRESH_BINARY_INV);
	if(display_construction)imshow("Saturation x < 50 ",grayMask);	
	
	// 3. Close + Ouverture avec un rectangle 15 x 15
	Mat kernel = getStructuringElement(MORPH_RECT, Size(15,15));
	morphologyEx(grayMask, grayMask, MORPH_CLOSE, kernel);
	morphologyEx(grayMask, grayMask, MORPH_OPEN, kernel);
	if(display_construction)imshow("Close +> Open with 15x15 rect",grayMask);	

	vector<vector<Point>> contours;
	// 4. Trouve les contours
	findContours(grayMask, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	// 5. Ne garder que les contours à + de 'min_area_value' pixels
	int min_area_value = 10000;
	for (auto& c : contours) {
		double area = contourArea(c);
		if (area > min_area_value) {
			road_contours.push_back(c);
		}
	}
	
	Mat maskColor = Mat::zeros(m_frame.size(), m_frame.type());
	// 6. On redessine les contours larges uniquement, en rouge
	for (auto& c : road_contours) {
		drawContours(maskColor, vector<vector<Point>>{c}, -1, Scalar(0,0,255), 3); // rouge
	}
	if(display_construction)imshow("Only contours > 10k pixels",maskColor);	

	// 7. On trace leur contours
	road_contour_frame = m_frame.clone();
	for (auto& c : road_contours) {
		drawContours(road_contour_frame, vector<vector<Point>>{c}, -1, Scalar(0,0,255), 3); // vert, épaisseur 3
	}
	if(display_construction)imshow("Contours Traced", road_contour_frame);
	return true;
}

int Camera::generate_car_box(){
	// Generate box around the cars
			
	// Image de référence
	ref = alpha * m_frame + (1.0f - alpha) * last_ref;
	last_ref = ref;
	imshow("Ref",ref);
	
	
	// Construction masque de différence
	// si ( |Image_ref(i) - Image(i|>seuil=>1)
	Mat abs_diff;
	abs_diff = abs(ref-m_frame);
	imshow("Image difference",abs_diff);

	// Masque de différence en NDG
	Mat diff_gray;
	cvtColor(abs_diff, diff_gray, COLOR_BGR2GRAY);
	imshow("Image difference en NDG",diff_gray);
	
	// Masque de différence en binaire (threshold)
	Mat diff_bin;
	threshold(diff_gray, diff_bin, 5, 255, THRESH_BINARY);
	imshow("Binaire image diff",diff_bin);

	// On supprime les tous petits bouts
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(15,15));
	morphologyEx(diff_bin, diff_bin, MORPH_CLOSE, kernel);
	morphologyEx(diff_bin, diff_bin, MORPH_OPEN, kernel);
	imshow("Apres fermeture ouverture", diff_bin);

	vector<vector<Point>> contours;
	findContours(diff_bin, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			
}


void Camera::play()
{
	// Create main window
	namedWindow("Video", cv::WINDOW_AUTOSIZE);
	bool isReading = true;
	// Compute time to wait to obtain wanted framerate
	int timeToWait = 1000/m_fps;
	float alpha = 0.65f;  // vitesse de mise à jour
	Mat last_ref = m_frame.clone();
	Mat ref = Mat::zeros(m_frame.size(), m_frame.type());
	while(isReading)
	{
		// Get frame from stream
		isReading = m_cap.read(m_frame);
		
		if(isReading)
		{
			// Show frame in main window
			//imshow("Video",video_with_contours);	

			// Show frame with contours, in main window
			Mat video_with_contours = m_frame.clone();
			for (auto& c : road_contours) {
				drawContours(video_with_contours, vector<vector<Point>>{c}, -1, Scalar(0,0,255), 3); // rouge, épaisseur 3
			}
			imshow("Video",video_with_contours);
			

			/// --------------------------------

			for (auto &c : contours)
			{
				Rect box = boundingRect(c);
				rectangle(m_frame, box, Scalar(0, 255, 0), 2);  // vert, épaisseur 2
			}

			// Afficher le résultat
			imshow("Voiture detectees", m_frame);


			
		}
		else
		{
			std::cerr << "Unable to read device" << std::endl;
		}
		
		// If escape key is pressed, quit the program
		if(waitKey(timeToWait)%256 == 27)
		{
			std::cerr << "Stopped by user" << std::endl;
			isReading = false;
		}	
	}	
}









bool Camera::close()
{
	// Close the stream
	m_cap.release();
	
	// Close all the windows
	destroyAllWindows();
	usleep(100000);
	return true;
}


