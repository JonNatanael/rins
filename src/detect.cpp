/* HOG DETECTOR
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <dlib/svm_threaded.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_processing.h>
#include <dlib/data_io.h>
#include <dlib/image_transforms.h>
#include <dlib/cmd_line_parser.h>
#include <dlib/opencv.h>

#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace std;
using namespace dlib;

struct TrafficSign {
	string name;
	string svm_path;
	rgb_pixel color;
	TrafficSign(string name, string svm_path, rgb_pixel color) :
		name(name), svm_path(svm_path), color(color) {};
};

typedef scan_fhog_pyramid<pyramid_down<6> > image_scanner_type;
std::vector<object_detector<image_scanner_type> > detectors;


void imageCallback(const sensor_msgs::ImageConstPtr& cam_msg) {
	cv_bridge::CvImagePtr cv_ptr;
	cout << "image received" << endl;
	const unsigned long upsample_amount = 0;

	//array2d<unsigned char> image = *cam_msg;

	try {
		cv_ptr = cv_bridge::toCvCopy(cam_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv_image<bgr_pixel> image = cv_image<bgr_pixel>(cv_ptr->image);

	std::vector<rect_detection> rects;

	evaluate_detectors(detectors, image, rects);

	if (rects.size() > 0) {
		cout << rects.at(0).rect.width() << endl;
		cv::Rect r(rects.at(0).rect.left(), rects.at(0).rect.top(), rects.at(0).rect.width(), rects.at(0).rect.height());
		cv::rectangle(cv_ptr->image, r, cv::Scalar(0, 0, 0), 2);
	}


	cv::imshow("detection", cv_ptr->image);
	cv::waitKey(1);

}

int main(int argc, char** argv) {

	/*cout << t << endl;

	string filename = "/home/jon/Desktop/catkin_ws/src/rins/img.png";


	//dlib::array<array2d<unsigned char> > images;
	//images.resize(1);

	array2d<unsigned char> image;

	/*for (unsigned long i = 0; i < images.size(); ++i) {
		load_image(images[i], filename);
	}

	for (unsigned long i = 0; i < upsample_amount; ++i) {
		for (unsigned long j = 0; j < images.size(); ++j) {
			pyramid_up(images[j]);
		}
	}

	load_image(image, filename);

	typedef scan_fhog_pyramid<pyramid_down<6> > image_scanner_type;


	std::vector<TrafficSign> signs;
	signs.push_back(TrafficSign("PARE", "/home/jon/Desktop/catkin_ws/src/rins/svm_detectors/pare_detector.svm",
	                            rgb_pixel(255, 0, 0)));
	signs.push_back(TrafficSign("LOMBADA", "/home/jon/Desktop/catkin_ws/src/rins/svm_detectors/lombada_detector.svm",
	                            rgb_pixel(255, 122, 0)));
	signs.push_back(TrafficSign("PEDESTRE", "/home/jon/Desktop/catkin_ws/src/rins/svm_detectors/pedestre_detector.svm",
	                            rgb_pixel(255, 255, 0)));

	std::vector<object_detector<image_scanner_type> > detectors;

	for (int i = 0; i < signs.size(); i++) {
		object_detector<image_scanner_type> detector;
		deserialize(signs[i].svm_path) >> detector;
		detectors.push_back(detector);
	}

	image_window win;
	std::vector<rect_detection> rects;

	evaluate_detectors(detectors, image, rects);

	// Put the image and detections into the window.
	win.clear_overlay();
	win.set_image(image);

	for (unsigned long j = 0; j < rects.size(); ++j) {
		win.add_overlay(rects[j].rect, signs[rects[j].weight_index].color,
		                signs[rects[j].weight_index].name);
	}

	cout << "Press any key to continue...";
	cin.get();*/

	ros::init (argc, argv, "sign_detector");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud (the name can be remapped in the launch file)
	ros::Subscriber sub = nh.subscribe ("/bebop/image_raw", 1, imageCallback);

	// Create a ROS publisher for the output point cloud (the name can be remapped in the launch file)
	//pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

	cv::namedWindow("detection", CV_WINDOW_AUTOSIZE);


	std::vector<TrafficSign> signs;
	signs.push_back(TrafficSign("PARE", "/home/jon/Desktop/catkin_ws/src/rins/svm_detectors/pare_detector.svm",
	                            rgb_pixel(255, 0, 0)));
	signs.push_back(TrafficSign("LOMBADA", "/home/jon/Desktop/catkin_ws/src/rins/svm_detectors/lombada_detector.svm",
	                            rgb_pixel(255, 122, 0)));
	signs.push_back(TrafficSign("PEDESTRE", "/home/jon/Desktop/catkin_ws/src/rins/svm_detectors/pedestre_detector.svm",
	                            rgb_pixel(255, 255, 0)));

	//std::vector<object_detector<image_scanner_type> > detectors;

	for (int i = 0; i < signs.size(); i++) {
		object_detector<image_scanner_type> detector;
		deserialize(signs[i].svm_path) >> detector;
		detectors.push_back(detector);
	}

	// Spin ...
	ros::spin ();

	return 0;
}