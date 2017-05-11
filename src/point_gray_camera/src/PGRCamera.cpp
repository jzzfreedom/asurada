#pragma once

#include "point_gray_camera/PGRCamera.h"
#include "flycapture/FlyCapture2.h"
#include <iostream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;
using namespace FlyCapture2;

PGRCamera::PGRCamera(bool enableDebugMode, bool flip) :
		m_debug(enableDebugMode)
		, m_flip(flip)
		, m_enableVideo(false)
		, m_size(Size(0, 0))
		, m_fps(0)
		, m_width(0)
		, m_height(0)
		, m_showOverallROI(false)
		, m_connected(false)
		, m_started(false)
		, m_serial(0)
		, m_busManager()
		, m_camera()
{
}

void PGRCamera::SetSerial(uint32_t serial)
{
	m_serial = serial;
}

bool PGRCamera::InitializePGRCamera()
{
	PGRGuid guid;

	if (m_serial != 0)
	{
		cout<<"Connecting PGR with serial number!"<<endl;
	    m_error = m_busManager.GetCameraFromSerialNumber(m_serial, &guid);
	    if (m_error != PGRERROR_OK)
	    {
			cout << "Failed to connect to camera with serial" << endl;
			return false;
		}

	}
	else
	{
		cout<<"Connecting PGR with index number!"<<endl;
	    m_error = m_busManager.GetCameraFromIndex(0, &guid);
	    if (m_error != PGRERROR_OK)
	    {
			cout << "Failed to connect to camera with index 0" << endl;
			return false;
		}

	}

	m_error = m_camera.Connect(&guid);
	if (m_error != PGRERROR_OK)
	{
		cout << "Failed to connect to camera" << endl;
		return false;
	}

	cout<<"Connection succeed!"<<endl;

	// Get the camera info and print it out
	m_error = m_camera.GetCameraInfo(&m_camInfo);
	if (m_error != PGRERROR_OK)
	{
		std::cout << "Failed to get camera info from camera" << std::endl;
		return false;
	}

	if (m_debug)
	{
		PrintCameraInfo();
	}

	m_error = m_camera.StartCapture();

	if (m_error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
	{
		std::cout << "Bandwidth exceeded" << std::endl;
		return false;
	}
	else if (m_error != PGRERROR_OK)
	{
		std::cout << "Failed to start image capture" << std::endl;
		return false;
	}

    cout<<"PGR camera connected."<<endl;
	m_connected = true;
	return true;
}

/*bool PGRCamera::InitializeVideoMode()
{
	// Get FPS from camera
	Property prop;
	prop.type = FRAME_RATE;
	m_error = m_camera.GetProperty(&prop);
	m_fps = prop.absValue;

	assert(m_enableVideo == false);
	int th = 10;
	Mat frame;
	while (th--)
	{
		bool success = GetNextMatFrame(frame);
		if (success)
		{
			m_width = frame.cols;
			m_height = frame.rows;
			break;
		}
	}

	m_enableVideo = m_width > 0 && m_height > 0 && m_fps > 0;
	return m_enableVideo;
}*/

void PGRCamera::GetNextMatFrame(sensor_msgs::Image &image, const std::string &frame_id)
{
	Image rawImage;
	FlyCapture2::Error error = m_camera.RetrieveBuffer(&rawImage);
	if (error != PGRERROR_OK)
	{
		std::cout << "capture error" << std::endl;
	}

	std_msgs::Header header; // empty header
    TimeStamp embeddedTime = rawImage.GetTimeStamp();
    header.stamp.sec = embeddedTime.seconds;
	header.stamp.nsec = 1000 * embeddedTime.microSeconds;
    header.frame_id = frame_id;

	// convert to rgb
	Image rgbImage;
	rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

	// convert to OpenCV Mat
	unsigned int rowBytes = (double) rgbImage.GetReceivedDataSize()
			/ (double) rgbImage.GetRows();
	Mat matImage = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3,
			rgbImage.GetData(), rowBytes);

	if (m_showOverallROI)
	{
		cv::rectangle(matImage, Point(m_overallROI.x, m_overallROI.y),
				Point(m_overallROI.x + m_overallROI.width - 1,
						m_overallROI.y + m_overallROI.height - 1),
				cv::Scalar(0, 0, 255));

	}
	if (m_flip)
	{
		flip(matImage, matImage, -1); // rotate 180 degree
	}

	cv_bridge::CvImage img_bridge;
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, matImage);
	img_bridge.toImageMsg(image); 
	//matImage.copyTo(frame);
}

void PGRCamera::PrintCameraInfo()
{
	std::cout << m_camInfo.vendorName << " " << m_camInfo.modelName << " "
			<< m_camInfo.serialNumber << std::endl;
}

void PGRCamera::StopPGRCamera()
{
	m_error = m_camera.StopCapture();
	if (m_error != PGRERROR_OK)
	{
		// This may fail when the camera was removed, so don't show
		// an error message
	}

	m_camera.Disconnect();
}

double PGRCamera::GetFPS()
{
	return m_fps;
}

cv::Size PGRCamera::GetSize()
{
	return Size(m_width, m_height);
}

void PGRCamera::SetAEROI(Rect ROI, bool reset, bool showROI)
{
	unsigned int regValue;
	m_error = m_camera.ReadRegister(0x1A70, &regValue);
	unsigned int AE_ctrl = regValue;
	int AE_ROI_enable = regValue >> 31;
	if (AE_ROI_enable == 1)
	{
		cout << "AE_ROI supported." << endl;
		m_error = m_camera.ReadRegister(0x1A74, &regValue);
		regValue = regValue * 4;
		unsigned int AE_ROI_base = regValue & 0xffff;

		m_camera.ReadRegister(AE_ROI_base, &regValue);
		unsigned int Hposunit = regValue >> 16;
		unsigned int Vposunit = regValue & 0xffff;
		cout << AE_ROI_base << "Hposunit: " << Hposunit << "; Vposunit: "
				<< Vposunit << endl;

		m_camera.ReadRegister(AE_ROI_base + 4, &regValue);
		unsigned int Hunit = regValue >> 16;
		unsigned int Vunit = regValue & 0xffff;
		cout << "Hunit: " << Hunit << "; Vunit: " << Vunit << endl;

		if (reset)
		{
			m_camera.WriteRegister(AE_ROI_base + 8, 0, false);
			m_camera.WriteRegister(AE_ROI_base + 12, 0, false);
		}
		else
		{
			m_overallROI = ROI;
			m_showOverallROI = showROI;
			unsigned int leftTop = ROI.x << 16 | ROI.y;
			unsigned int widthHeight = ROI.width << 16 | ROI.height;
			unsigned int temp = 1 << 25 | AE_ctrl;
			//cout << temp <<  " "<<widthHeight<<endl;
			m_camera.WriteRegister(0x1A70, temp, false);
			//m_camera.ReadRegister(0x1A70, &regValue);
			//cout << "After: " << regValue << " "<<temp<<endl;
			m_camera.WriteRegister(AE_ROI_base + 8, leftTop, false);
			m_camera.WriteRegister(AE_ROI_base + 12, widthHeight, false);
		}
	}
	else
	{
		cout << "AE_ROI not supported!" << endl;
	}
	//cout << regValue << " "<<AE_ROI_enable<<endl;
}

