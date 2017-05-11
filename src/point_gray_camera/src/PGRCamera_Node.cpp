#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include "point_gray_camera/PGRCamera.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "point_gray_camera/PGR_AE_ROI.h"

using namespace std;
using namespace cv;

int main(int argc, char**argv)
{
	cout<<"Start PGR camera!"<<endl;

	ros::init(argc, argv, "pgr_camera");
	ros::NodeHandle nh("~");

	ros::Rate rate(30);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("image_raw", 1);

    ros::Publisher roi_publisher = nh.advertise<point_gray_camera::PGR_AE_ROI>("AE_ROI", 1);

    bool flip = false;
	nh.getParam("flip", flip);
		
	std::shared_ptr<PGRCamera> PGR_cam_ptr;
	PGR_cam_ptr = std::make_shared<PGRCamera>(true, flip);

    int serial = 0;
	if (nh.getParam("serial", serial))
	{
		PGR_cam_ptr->SetSerial(uint32_t(serial));
	}

	bool succeed = PGR_cam_ptr->InitializePGRCamera();

	if (!succeed)
	{
		return -1;
	}

    point_gray_camera::PGR_AE_ROI ae_roi_msg;
    ae_roi_msg.flip = flip;
	int32_t expected_width = 0;
	int32_t expected_height = 0; 
	nh.getParam("expected_width", expected_width);
	nh.getParam("expected_height", expected_height);

	ae_roi_msg.width = expected_width;
	ae_roi_msg.height = expected_height;

    int enableAEROI = 0;

    if (nh.getParam("enableAEROI", enableAEROI) && enableAEROI)
	{
        cout<<"AE ROI enabled, reading ROI value"<<endl;
		int32_t AEROI_x_offset = 0;
		int32_t AEROI_y_offset = 0;
		int32_t AEROI_width = 0;
		int32_t AEROI_height = 0;
		if (nh.getParam("AEROI_x_offset", AEROI_x_offset)
			&& nh.getParam("AEROI_y_offset", AEROI_y_offset)
			&& nh.getParam("AEROI_width", AEROI_width)
			&& nh.getParam("AEROI_height", AEROI_height))
		{
			ae_roi_msg.roi_width = AEROI_width;
			ae_roi_msg.roi_height = AEROI_height;
			ae_roi_msg.roi_offset_x = AEROI_x_offset;
			ae_roi_msg.roi_offset_y = AEROI_y_offset;

			cout<<"All AE ROI value available"<<endl;\
			cout<<"  X offset: " << AEROI_x_offset <<endl;
			cout<<"  Y offset: " << AEROI_y_offset <<endl;
			cout<<"  ROI width: " << AEROI_width <<endl;
			cout<<"  ROI height: " << AEROI_height<<endl;
			cv::Rect overall_ROI(Point(AEROI_x_offset, AEROI_y_offset), Size(AEROI_width, AEROI_height));
			PGR_cam_ptr->SetAEROI(overall_ROI, false, false);
		}
	}
    //nh.getParam("frame_id", frame_id);
	//nh.getParam("rate", hz);

    cout<<"Connect succeed."<<endl;

	while(ros::ok())
	{
		sensor_msgs::Image image_msg;

		PGR_cam_ptr->GetNextMatFrame(image_msg, "pgr1");

		pub.publish(image_msg);
		roi_publisher.publish(ae_roi_msg);
		ros::spinOnce();
		rate.sleep();
	}


	return 0;
}
