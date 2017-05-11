#include <iostream>
#include <string>
#include <sstream>

#include <flycapture/FlyCapture2.h>

using namespace std;
using namespace FlyCapture2;

void printCameraInfo(const CameraInfo& cinfo) {
  std::cout << "Serial: " << cinfo.serialNumber
            << ", Model: " << cinfo.modelName
            << ", Vendor: " << cinfo.vendorName
            << ", Sensor: " << cinfo.sensorInfo
            << ", Resolution: " << cinfo.sensorResolution
            << ", Color: " << std::boolalpha << cinfo.isColorCamera
            << ", Firmware Version: " << cinfo.firmwareVersion << std::endl;
}


int main(int argc, char** argv)
{
    cout<<"Hello Point Grey."<<endl;
    
    unsigned int num_devices = 0;
    BusManager bus_manager;
	try
	{
        Error err = bus_manager.GetNumOfCameras(&num_devices);
        cout<<"Point Grey Camera Numbers: " << num_devices <<endl;
		if (num_devices)
		{
			for (unsigned int i = 0; i < num_devices; ++i)
			{
			    PGRGuid guid;
				std::ostringstream ss;
				ss << i;
				bus_manager.GetCameraFromIndex(i, &guid);

				Camera cam;
				cam.Connect(&guid);

				CameraInfo cinfo;
				cam.GetCameraInfo(&cinfo);

				cout <<"Camera: "<< i <<endl;
				printCameraInfo(cinfo);
				cout<<endl;

			}

		}
	}
	catch(const std::runtime_error& e)
	{
		cout<<"Unable to get Point Grey Camera info: " << e.what() << endl;
	}

    return 0;
}
