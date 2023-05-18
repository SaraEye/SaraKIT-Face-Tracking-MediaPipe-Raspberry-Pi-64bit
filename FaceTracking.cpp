#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <math.h>
#include <arm_neon.h>
#include "unistd.h"

#include "struct.hpp"
#include "lib/viewer/viewer.hpp"
#include "lib/mediapipe/Mediapipe.hpp"
#include "lib/SaraKIT/devices.hpp"

using namespace std;
 
cv::Mat frame, frameGray, frameGrayHalf, frameGrayHalfEdge;     // cam0
cv::Mat frame1, frame1Gray, frame1GrayHalf, frame1GrayHalfEdge; // cam1
cv::Mat frameBig, imgPreview, imgPreview1, imgWindow, imgcopy0, imgcopy1;
cv::Mat imgORG, imgEdge;

ViewerStatus viewStatus;

//ctrl-c 
void ctrlc_handler(sig_atomic_t s){
    printf("\nCaught signal %d\n",s);
    BLDCMotor_MoveStop(0);
    BLDCMotor_MoveStop(1);
    control_c=true;	
    exit(1);
}

void moveCamToRect(cv::Rect faceRect){
    auto center=(faceRect.tl()+faceRect.br())/2;
    float angle_x, angle_y;    
    angle_scale = (124.0 / camwidth); //(for cam 62 degree)

    int setpoint_x_rel = (center.x - (camwidth >> 1)) >> 1;
    angle_x = setpoint_x_rel * angle_scale * (0.30 + ((1 - (faceRect.width) / camwidth)) / 10);

    int setpoint_y_rel = (center.y - (camheight >> 1)) >> 1;
    angle_y = setpoint_y_rel * angle_scale * (0.30 + ((1 - (faceRect.height) / camheight)) / 10);

    if (abs(angle_x)<0.5)
        angle_x=0;
    if (abs(angle_y)<0.5)
        angle_y=0;
    if (control_c!=true){
        //printf("x:%.2f y:%.2f\n",angle_x,angle_y);
        BLDCMotor_MoveByAngle(1, angle_x, 1, 20, true); 
        BLDCMotor_MoveByAngle(0, angle_y, 5, 20, true);
    }
}

int main(int argc, char** argv){
    signal(SIGINT,ctrlc_handler);
    imgEdge=cv::Mat(camheight, camwidth, CV_8UC3);

    init_camera(0, camwidth, camheight, false, camOpenCV, cambuffered, true, true);
    //init_camera(1, cam1width, cam1height, false, camOpenCV, cambuffered, false, false);

    //set gimbals pole
    BLDCMotor_PolePairs(0,11);
    BLDCMotor_PolePairs(1,11);

    //run once, get info
    //BLDCMotor_InitFOC(0,0,0,0);
    //and paste below
    BLDCMotor_InitFOC(0, 0, -1, 3.63);
    BLDCMotor_On(0,true);
    BLDCMotor_On(1,true);

    //dupa na en - na drgania, zależy od ciężaru silnika i tego co silnik niesie
    BLDCMotor_PIDVelocity(0, 0.4, 20, 0, 1000);
    BLDCMotor_PIDVelocity(1, 5, 20, 0, 0);
    //BLDCMotor_PIDVelocity(1, 10, 10, 0, 1000);
    BLDCMotor_IdleTorque(1, 0, 2500); 


    init_viewer();

    std::vector<std::vector<cv::Point2f>> faceLandmarks;
    std::vector<mpface::Face> faces;

    MediapipeFaceMesh mfm;
    FrameResultSynchroniser meshSync(100);
    //MediapipeFace mf;
    //FrameResultSynchroniser faceSync(100);

    int iz=0;
    while (_SPICheck()==false && iz<10) {
        iz++;
        sleepms(100);
    }

    printf("Start Loop\n");
    do {
        // Get frame to frame,frameGray,frameGrayHalf (CamWide is only gray)dupa
        if (GetFrame(0)!=1) { //GetFrame()==1 (new frame from cam0, ==2 from cam1, ==3 from cam0 & cam 1)

            //here you have time to do anything
            sleepms(1);
            continue;
        }

        std::vector<std::vector<cv::Point2f>> faceLandmarks;
        faces.clear();

        meshSync.pushFrame(mfm.pushFrame(frame),frame);
        int resm=meshSync.getFrameFromId(mfm.getFaceMesh(faceLandmarks),imgEdge);

        if (resm&&faceLandmarks.size()) {
            mfm.drawFaceMesh(imgEdge,faceLandmarks);
            std::vector<cv::Rect> facesRects;
            mfm.getFacesRectsFromLandmarks(faceLandmarks,facesRects);
            mfm.drawFacesRects(imgEdge,facesRects,greenScalar);
            tick2on=true;
            moveCamToRect(facesRects[0]);
        }
        viewStatus = viewer_refresh();

    } while (viewStatus != ViewerStatus::Exit && control_c != true);
    closing_function(0);
    return 1;
}
