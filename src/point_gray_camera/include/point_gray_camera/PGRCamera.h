
#include <string>
#include <sensor_msgs/Image.h> // ROS message header for Image
#include <sensor_msgs/image_encodings.h> // ROS header for the different supported image encoding types
#include <sensor_msgs/fill_image.h>
#include <sstream>

#include <opencv2/highgui/highgui.hpp>

// FlyCapture SDK from Point Grey
#include "flycapture/FlyCapture2.h"

using namespace cv;

class PGRCamera
{
public:
	PGRCamera(bool enableDebugMode, bool flip);

	// This function returns true if initializationg success; false otherwise
	bool InitializePGRCamera();
	//bool InitializeVideoMode();
	double GetFPS();
	cv::Size GetSize();
	void StopPGRCamera();
	void GetNextMatFrame(sensor_msgs::Image &image, const std::string &frame_id);
	void SetAEROI(cv::Rect ROI, bool reset, bool showROI);
	void SetSerial(uint32_t serial);
	
private:

	void PrintCameraInfo();
	bool m_debug;
	bool m_flip;
	bool m_enableVideo;
	bool m_showOverallROI;
	cv::Size m_size;
	cv::Rect m_overallROI;
	double m_fps;
	double m_width;
	double m_height;
	bool m_connected;
	bool m_started;
	uint32_t m_serial;
	FlyCapture2::Error m_error;
	FlyCapture2::Camera m_camera;
	FlyCapture2::CameraInfo m_camInfo;
	FlyCapture2::BusManager m_busManager;
};
