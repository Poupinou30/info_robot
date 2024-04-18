//poti fichier pour la cam
//raspberry pi

#ifndef CAM_H
#define CAM_H


#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

#include "opencv2/core.hpp"


#include <iostream>
//#include <fstream>
//using namespace std;

//using namespace cv;
#endif

int main( int argc, char** argv){
		
		std::string image_path = "/home/student/cam_images/img_test_ali.jpg";//ici à modif
		cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
		
		//cv::imshow("Display window", img);
		
		//int k= cv::waitKey(0);
		
		std::cout << "poupon teste son code" << std::endl;
		return 0;
		
}
