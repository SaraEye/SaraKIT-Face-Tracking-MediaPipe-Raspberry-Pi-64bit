#ifndef _STRUCT_H_
#define _STRUCT_H_

#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "lib/camera/RaspiCamCV.hpp"

const int camwidth = 640;
const int camheight = 480;
const int cam1width = 640;
const int cam1height = 480;

const bool useWebserver = true;  // show window on http://ip:7777 
const bool showGUI = false;  // show window 
const bool cambuffered = true;    // false=wait to frame, frame by frame; true get last grabed frame (more CPU)
const bool camFile = false;       // frames from file video.mp4 - if camfile then camOpenCV=true
const bool camOpenCV = false;

extern cv::Mat frame,   frameGray,   frameGrayHalf,      frameGrayHalfEdge;  //cam0
extern cv::Mat frame1,  frame1Gray,  frame1GrayHalf,     frame1GrayHalfEdge; //cam1
extern cv::Mat frameBig, imgPreview, imgPreview1, imgWindow, imgcopy0, imgcopy1;
extern cv::Mat imgORG, imgEdge;

//extern Type::Frame edgesFrame;
extern cv::VideoCapture capcv0, capcv1;
extern RaspiCamCvCapture *caprp0, *caprp1;

extern char window_name[];
extern int file_slider_max;
extern int file_slider;
extern int threshold_slider;
static int64 tick, tick2;
extern float previewSize;
extern bool TwoWindow;

extern int ModeView1;
extern int ModeView2;

extern bool ispause;

enum class ViewerStatus { Continue, Pause, Depth, Exit };

extern ViewerStatus viewStatus;
extern cv::Point lastMotion;
extern int lastMotionframes;

namespace KeyboardKey {
const char BACKSPACE = 8;
const char ENTER = 13;
const char ESC = 27;
const char SPACE = 32;
}  // namespace KeyboardKey

extern bool control_c;

extern float timeSum;
extern int timeIdx;

extern float angle_scale;

const cv::Scalar redScalar = cv::Scalar(0, 0, 255);
const cv::Scalar greenScalar = cv::Scalar(0, 255, 0);
const cv::Scalar blueScalar = cv::Scalar(255, 0, 0);
const cv::Scalar whiteScalar = cv::Scalar(255, 255, 255);
const cv::Scalar yellowScalar = cv::Scalar(0, 255, 255);
const cv::Scalar cyanScalar = cv::Scalar(255, 255, 0);
const cv::Scalar blackScalar = cv::Scalar(0, 0, 0);

extern void sleepms(int ms);

extern bool tick2on;

#endif  // _STRUCT_H_
