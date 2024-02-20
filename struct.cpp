#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "lib/camera/RaspiCamCV.hpp"

#include <unistd.h>

int camwidth = 640;
int camheight = 480;

cv::Mat frameBig, imgPreview, imgPreview1, imgWindow, imgcopy0, imgcopy1, imgORG;

int ModeView1=0;		//which image in window 1
int ModeView2=2;		//which image in window 2
float scaleView=1;	//window size e.g. 1 or 0.5
bool TwoWindow=true;	//one or two window

bool useWebserver=true;	// show window on http://raspberrypi:7777
bool showGUI=false;		// show window 
bool cambuffered=true;	// false=wait to frame, frame by frame; true get last grabed frame (more CPU)
bool camFile=false;		// frames from file video.mp4 - if camfile then camOpenCV=true
bool camOpenCV=false;	//OpenCV or RaspiCam

cv::VideoCapture capcv0, capcv1;
RaspiCamCvCapture * caprp0, * caprp1;

char window_name[] = "SaraVision";

int file_slider_max = 200;
int threshold_slider = 20;
int file_slider = 10;

bool ispause=false;
bool control_c=false;

float timeSum = 0.0f;
int timeIdx = 0;
bool tick2on = false;

float angle_scale;