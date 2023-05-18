#ifndef __RaspiCamCV__
#define __RaspiCamCV__

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//using namespace cv;

typedef struct _RASPIVID_STATE RASPIVID_STATE;

typedef struct
{
	int width;              
	int height;             
	int bitrate;            
	int framerate;          
	int monochrome;
	int buffered;
	int hflip;
	int vflip;
} RASPIVID_CONFIG;

typedef struct {
	RASPIVID_STATE * pState;
} RaspiCamCvCapture;

typedef struct _IplImage IplImage;

RaspiCamCvCapture * raspiCamCvCreateCameraCapture2(int index, RASPIVID_CONFIG* config);
RaspiCamCvCapture * raspiCamCvCreateCameraCapture(int index);
void raspiCamCvReleaseCapture(RaspiCamCvCapture ** capture);
double raspiCamCvGetCaptureProperty(RaspiCamCvCapture * capture, int property_id);
int raspiCamCvSetCaptureProperty(RaspiCamCvCapture * capture, int property_id, double value);
cv::Mat raspiCamCvQueryFrame(RaspiCamCvCapture * capture);

int init_camera(int camnum, int width, int height, bool gray, bool camcv, bool buffered, int hflip, int vflip);

int GetFrameFile(cv::Mat &frame,cv::Mat &framewide);
int GetFrame(int cam);

int GetFrameCam0(cv::Mat &frame, cv::Mat &frameGray, cv::Mat &frameGrayHalf, cv::Mat &frameGrayHalfEdge);
int GetFrameCam1(cv::Mat &frame, cv::Mat &frameGray, cv::Mat &frameGrayHalf);

#endif
