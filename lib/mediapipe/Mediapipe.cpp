#include "Mediapipe.hpp"
#include <iostream>

float scalex=0;
float scaley=0;

cv::Point2f GetSPoint(cv::Point2f p)
{
    p.x=p.x*scalex;
    p.y=p.y*scaley;
    return p;
}

//==========================================================================================

FrameResultSynchroniser::FrameResultSynchroniser(int historySize) : historySize(historySize)
{
}
FrameResultSynchroniser::~FrameResultSynchroniser() {}
void FrameResultSynchroniser::pushFrame(int64 id, cv::Mat &frame)
{
    scalex=frame.cols/640.0;
    scaley=frame.rows/480.0;
    cv::Mat copy=cv::Mat(frame.rows, frame.cols, CV_8UC3);
    memcpy(copy.data, frame.data, frame.rows * frame.cols * 3);
    if(history.size()>=historySize)
        history.erase(history.begin());
    history.insert(std::pair<int64, cv::Mat>(id,copy));
}
int FrameResultSynchroniser::getFrameFromId(int64 id, cv::Mat &frame)
{
    auto it = history.find(id);
    if (it != history.end()){
       // frame=it->second.clone();
        memcpy( frame.data, it->second.data,it->second.rows * it->second.cols * 3);
        return 1;
    }
    return 0;
}

//==========================================================================================

MediapipeFaceMesh::MediapipeFaceMesh()
{
    mpfacemesh::initMediaPipe("mediapipe/graphs/face_mesh_desktop_live.pbtxt");
}

MediapipeFaceMesh::~MediapipeFaceMesh()
{
    mpfacemesh::release();
}

int64 MediapipeFaceMesh::getFaceMesh(cv::Mat &frame, std::vector<std::vector<cv::Point2f>> &faceLandmarks)
{
    pushFrame(frame);
    return getFaceMesh(faceLandmarks);
}
int64 MediapipeFaceMesh::pushFrame(cv::Mat &frame)
{
    return mpfacemesh::pushFrame(frame.cols, frame.rows, frame.data);
}
int64 MediapipeFaceMesh::getFaceMesh(std::vector<std::vector<cv::Point2f>> &faceLandmarks)
{
    return mpfacemesh::getResult(faceLandmarks);
}

void MediapipeFaceMesh::drawFaceMesh(cv::Mat &frame, std::vector<std::vector<cv::Point2f>> &faceLandmarks)
{
    if (faceLandmarks.size())
    {
        cv::Scalar color=cv::Scalar(0, 0, 255);
        drawFaceElement(frame, mpfacemesh::FACEMESH_LIPS, faceLandmarks, color);
        color=cv::Scalar(255, 0, 0);
        drawFaceElement(frame, mpfacemesh::FACEMESH_FACE_OVAL, faceLandmarks, color);
        color=cv::Scalar(0, 0, 255);
        drawFaceElement(frame, mpfacemesh::FACEMESH_LEFT_EYEBROW, faceLandmarks, color);
        drawFaceElement(frame, mpfacemesh::FACEMESH_RIGHT_EYEBROW, faceLandmarks, color);
        drawFaceElement(frame, mpfacemesh::FACEMESH_LEFT_EYE, faceLandmarks, color);
        drawFaceElement(frame, mpfacemesh::FACEMESH_RIGHT_EYE, faceLandmarks, color);
        color=cv::Scalar(0, 255, 255);
        int e1=((faceLandmarks[0][477].y-faceLandmarks[0][475].y)/2.0)*scaley;
        int e2=((faceLandmarks[0][472].y-faceLandmarks[0][470].y)/2.0)*scaley;
        cv::circle(frame, GetSPoint(faceLandmarks[0][473]) ,e1,color);
        cv::circle(frame, GetSPoint(faceLandmarks[0][468]) ,e2,color);

        color=cv::Scalar(255, 255, 255);
        for (int n = 0; n < faceLandmarks.size(); n++)
            for (int m = 0; m < faceLandmarks[n].size(); m++)
            {
                cv::circle(frame, GetSPoint(faceLandmarks[n][m]), 1, color);
            }
    }
}

void MediapipeFaceMesh::drawFaceElement(cv::Mat &frame, std::vector<cv::Point2i> &faceElementLines, std::vector<std::vector<cv::Point2f>> &faceLandmarks, cv::Scalar color)
{
    if (faceLandmarks.size())
    {
        for (int n = 0; n < faceLandmarks.size(); n++)
            for (int i = 0; i < faceElementLines.size(); i++)
            {
                //cv::circle(frame, faceLandmarks[n][faceElementLines[i].x], 5, color);
                //cv::circle(frame, faceLandmarks[n][faceElementLines[i].y], 3, color);                
                cv::line(frame,GetSPoint(faceLandmarks[n][faceElementLines[i].x]),GetSPoint(faceLandmarks[n][faceElementLines[i].y]),color,1);
            }
    }
}

int MediapipeFaceMesh::getFacesRectsFromLandmarks(std::vector<std::vector<cv::Point2f>> &faceLandmarks, std::vector<cv::Rect> &facesRects)
{
    facesRects.clear();
    for (int n = 0; n < faceLandmarks.size(); n++)
    {
        cv::Rect r=cv::boundingRect(std::vector<cv::Point2f>{GetSPoint(faceLandmarks[n][10]), GetSPoint(faceLandmarks[n][234]), GetSPoint(faceLandmarks[n][152]), GetSPoint(faceLandmarks[n][454]), GetSPoint(faceLandmarks[n][5])});
        r.x=r.x-(10*scalex);
        r.y=r.y-(10*scaley);
        r.width=r.width+(10*scalex*2);
        r.height=r.height+(10*scaley*2);
        facesRects.push_back(r);
    }
    return facesRects.size();
}
void MediapipeFaceMesh::drawFacesRects(cv::Mat &frame, std::vector<cv::Rect> &facesRects, cv::Scalar color)
{
    for (int n = 0; n < facesRects.size(); n++)
        cv::rectangle(frame, facesRects[n], color);
}

//==========================================================================================

MediapipeFace::MediapipeFace()
{
    mpface::initMediaPipe("mediapipe/graphs/face_detection_desktop_live.pbtxt");
}
MediapipeFace::~MediapipeFace()
{
    mpface::release();
}
int64 MediapipeFace::getFaces(cv::Mat &frame, std::vector<mpface::Face> &faces)
{
    scalex=frame.cols/640.0;
    scaley=frame.rows/480.0;

    pushFrame(frame);
    return getFaces(faces);
}
int64 MediapipeFace::pushFrame(cv::Mat &frame)
{
    return mpface::pushFrame(frame.cols, frame.rows, frame.data);
}
int64 MediapipeFace::getFaces(std::vector<mpface::Face> &faces)
{
    return mpface::getResult(faces);
}
void MediapipeFace::drawFacesRects(cv::Mat &frame, std::vector<mpface::Face> &faces)
{
    for (int n = 0; n < faces.size(); n++)
    {
        cv::Scalar color(0, 0, 255);
        cv::Rect frect=faces[n].getBoundingBox();
        frect.x=frect.x*scalex;
        frect.y=frect.y*scaley;
        frect.width=frect.width*scalex;
        frect.height=frect.height*scaley;
        cv::rectangle(frame, frect, color);
        for (int i = 0; i < faces[n].getFaceKeypoints().size(); i++)
        {
            cv::circle(frame, GetSPoint(faces[n].getFaceKeypoints()[i]), 2, color);
        }
    }
}

//==========================================================================================

MediapipeObject::MediapipeObject()
{
    mpobject::initMediaPipe("mediapipe/graphs/object_detection/object_detection_desktop_live.pbtxt");
                           
}
MediapipeObject::~MediapipeObject()
{
    mpobject::release();
}
int64 MediapipeObject::getObjects(cv::Mat &frame, std::vector<mpobject::Object> &object)
{
    pushFrame(frame);
    return getObjects(object);
}
int64 MediapipeObject::pushFrame(cv::Mat &frame)
{
    return mpobject::pushFrame(frame.cols, frame.rows, frame.data);
}
int64 MediapipeObject::getObjects(std::vector<mpobject::Object> &object)
{
    return mpobject::getResult(object);
}
void MediapipeObject::drawObjectsRects(cv::Mat &frame, std::vector<mpobject::Object> &objects)
{
    for (int n = 0; n < objects.size(); n++)
    {
        cv::Scalar color(0, 0, 255);
        //std::cout<<objects[n].getBoundingBox()<<" "<< objects[n].getObjectName()<<std::endl;
        cv::rectangle(frame, objects[n].getBoundingBox(),color);
        cv::putText(frame, //target image
            objects[n].getObjectName(), //text
            objects[n].getBoundingBox().tl(), //top-left position

            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            color, 
            2);
    }
   
}