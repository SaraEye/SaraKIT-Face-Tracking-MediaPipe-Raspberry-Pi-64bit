#ifndef _MEDIAPIPE_HPP_
#define _MEDIAPIPE_HPP_
#include "opencv2/core.hpp"
#include "opencv2/video.hpp"
#include "opencv2/highgui.hpp"
#include "mediapipe/mediapipe.h"
#include <map>
class FrameResultSynchroniser{
public:
    FrameResultSynchroniser(int historySize);
    ~FrameResultSynchroniser();
    void pushFrame(int64 id,cv::Mat &frame);
    int getFrameFromId(int64 id,cv::Mat &frame);
private:
    int historySize;
    std::map<int64,cv::Mat> history;
};

class MediapipeFaceMesh{
public:
    MediapipeFaceMesh();
    ~MediapipeFaceMesh();
    int64 getFaceMesh(cv::Mat &frame, std::vector<std::vector<cv::Point2f>> &faceLandmarks);
    int64 pushFrame(cv::Mat &frame);
    int64 getFaceMesh(std::vector<std::vector<cv::Point2f>> &faceLandmarks);
    void drawFaceMesh(cv::Mat &frame, std::vector<std::vector<cv::Point2f>> &faceLandmarks);
    void drawFaceElement(cv::Mat &frame, std::vector<cv::Point2i> &faceElementLines, std::vector<std::vector<cv::Point2f>> &faceLandmarks, cv::Scalar color);
    int getFacesRectsFromLandmarks(std::vector<std::vector<cv::Point2f>> &faceLandmarks,std::vector<cv::Rect> &facesRects);
    void drawFacesRects(cv::Mat &frame,std::vector<cv::Rect> &facesRects,cv::Scalar color);
};

class MediapipeFace{
public:
    MediapipeFace();
    ~MediapipeFace();
    int64 getFaces(cv::Mat &frame, std::vector<mpface::Face> &faces);
    int64 pushFrame(cv::Mat &frame);
    int64 getFaces(std::vector<mpface::Face> &faces);
    void drawFacesRects(cv::Mat &frame,  std::vector<mpface::Face> &faces);
};

class MediapipeObject{
public:
    MediapipeObject();
    ~MediapipeObject();
    int64 getObjects(cv::Mat &frame, std::vector<mpobject::Object> &object);
    int64 pushFrame(cv::Mat &frame);
    int64 getObjects(std::vector<mpobject::Object> &object);
    void drawObjectsRects(cv::Mat &frame,  std::vector<mpobject::Object> &object);
};


#endif
