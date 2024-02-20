#include "viewer.hpp"
#include <stdio.h>

#include "../../struct.hpp"

MJPEGWriter wserv;
float lastTemp=0;
int tempDelayTick=0;
float ffmin2 = 0;

void hconcat(const cv::Mat* src1, cv::Mat* src2, cv::Mat* dst) {
    for (int y = 0; y < src1->rows; y++) {
        memcpy(dst->data + (y * src1->cols * 2 * 3), src1->data + (y * src1->cols * 3), src1->cols * 3);
        memcpy(dst->data + (y * src1->cols * 2 * 3 + src1->cols * 3), src2->data + (y * src2->cols * 3), src2->cols * 3);
    }
}

void init_viewer(int firstViewMode, int secondViewMode, float ScaleView, bool ShowGUI, bool ShowOnWWW) {
	ModeView1=firstViewMode;
	ModeView2=secondViewMode;
	TwoWindow=false;
	if (secondViewMode>-1) {
		TwoWindow=true;
	}
	showGUI=ShowGUI;
	scaleView=ScaleView;
	useWebserver=ShowOnWWW;
	
    frame0 = cv::Mat(camheight, camwidth, CV_8UC3);
    frame0Gray = cv::Mat(camheight, camwidth, CV_8UC1);
    frame0GrayHalf = cv::Mat(camheight/2, camwidth/2, CV_8UC1);
    frame0GrayHalfEdge = cv::Mat(camheight/2, camwidth/2, CV_8UC1);

    frame1 = cv::Mat(camheight, camwidth, CV_8UC3);
    frame1Gray = cv::Mat(camheight, camwidth, CV_8UC1);
    frame1GrayHalf = cv::Mat(camheight/2, camwidth/2, CV_8UC1);

    imgcopy0 = cv::Mat(camheight, camwidth, CV_8UC3);
    imgcopy1 = cv::Mat(camheight, camwidth, CV_8UC3);

    imgPreview = cv::Mat(camheight * scaleView, camwidth * scaleView, CV_8UC3);
    imgPreview1 = cv::Mat(camheight * scaleView, camwidth * scaleView, CV_8UC3);

    imgWindow = cv::Mat(camheight * scaleView, camwidth * scaleView, CV_8UC3);
    if (showGUI) {
        cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
        cv::moveWindow(window_name, 0, 0);
        cv::createTrackbar("Threshold", window_name, &threshold_slider, 200, NULL /* on_trackbar*/);
        if (camFile) {
            file_slider_max = int(capcv0.get(cv::CAP_PROP_FRAME_COUNT));
            cv::createTrackbar("Position", window_name, &file_slider, file_slider_max, NULL /* on_trackbar*/);
        }
    }
    if (useWebserver) {
        wserv = MJPEGWriter(7777);
        wserv.start();
    }
    viewer_refresh();
}

std::vector<float> fs, fs2;

float GetTemp(bool show=true)
{
    FILE *tempfile;
    float temp;
    int n;

    tempfile = fopen("/sys/class/thermal/thermal_zone0/temp","r");
    n = fscanf(tempfile,"%f",&temp);
    fclose(tempfile);
    temp = temp / 1000;
    if (show)
        printf("Temp: %.2f\n",temp);
    return temp;
}

ViewerStatus viewer_refresh() {
    switch ((char)cv::waitKey(1)) {
        case KeyboardKey::ESC: {
            if (viewStatus==ViewerStatus::Depth)
                return ViewerStatus::Continue;
            return ViewerStatus::Exit;
        }
        case KeyboardKey::SPACE: {
            memcpy(imgcopy0.data, frame0.data, camwidth * camheight * 3);
            memcpy(imgcopy1.data, frame1.data, camwidth * camheight * 3);
            return ViewerStatus::Pause;
        }

        default:
            break;
    }

    if (!useWebserver && (int)cv::getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) == -1) {
        return ViewerStatus::Exit;
    }

    if (ModeView1 == 0) {
        if (viewStatus==ViewerStatus::Depth)
            resize(imgORG, imgPreview, cv::Size(), scaleView, scaleView, cv::INTER_LINEAR);
        else
            resize(frame0, imgPreview, cv::Size(), scaleView, scaleView, cv::INTER_LINEAR);
    } else if (ModeView1 == 1) {
        //cv::resize(frame1, imgPreview, cv::Size(), 1.0f * imgPreview1.cols / frame1.cols, 1.0f * imgPreview1.rows / frame1.rows, cv::INTER_LINEAR);
        cv::resize(frame1, imgPreview, cv::Size(), scaleView, scaleView, cv::INTER_LINEAR);
    } else if (ModeView1 == 2) {
        cv::resize(imgProcessed, imgPreview, cv::Size(), scaleView * camwidth / imgProcessed.cols, scaleView * camheight / imgProcessed.rows, cv::INTER_LINEAR);
    }

    if (TwoWindow) {
        if (ModeView2 == 0) {
            cv::resize(frame0, imgPreview1, cv::Size(), scaleView, scaleView, cv::INTER_LINEAR);
        } else if (ModeView2 == 1) {
            cv::resize(frame1, imgPreview1, cv::Size(), scaleView, scaleView, cv::INTER_LINEAR);
        } else if (ModeView2 == 2) {
            cv::resize(imgProcessed, imgPreview1, cv::Size(), scaleView * camwidth / imgProcessed.cols, scaleView * camheight / imgProcessed.rows, cv::INTER_LINEAR);
        }
        cv::hconcat(imgPreview, imgPreview1, imgWindow);
    } else
        imgWindow = imgPreview.clone();

    //tick1
    tick = cv::getTickCount() - tick;
    float f = tick * (1.0f / 1000000);
    if (f > 0) fs.push_back(f);

    float favg = 0;
    float ffmin = 0;
    float ffmax = 0;
    if (fs.size() > 0) {
        favg = accumulate(fs.begin(), fs.end(), 0.0) / fs.size();
        ffmin = *min_element(fs.begin(), fs.end());
        ffmax = *max_element(fs.begin(), fs.end());
    }
    if (fs.size() > 20) fs.erase(fs.begin());

    //tick2
    if (tick2on) {
        float favg2 = 0;
        float ffmax2 = 0;
        ffmin2 = 0;
        tick2 = cv::getTickCount() - tick2;
        float f2 = tick2 * (1.0f / 1000000);
        if (f2 > 0) fs2.push_back(f2);

        if (fs2.size() > 0) {
            favg2 = accumulate(fs2.begin(), fs2.end(), 0.0) / fs2.size();
            ffmin2 = *min_element(fs2.begin(), fs2.end());
            ffmax2 = *max_element(fs2.begin(), fs2.end());
        }
        if (fs2.size() > 20) fs2.erase(fs2.begin());
    }

    float fontsize = 1.4;
    if (frame0.cols<640) fontsize = fontsize / 2.0;
    if (frame0.cols>640) fontsize = fontsize * 2.0;

    //cv::putText(imgWindow, cv::format("FPS:%4.1fms (%4.1f) avg:%4.0fms max:%4.0fms", ffmin, 1000/ffmin, favg, ffmax), cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0));
    cv::putText(imgWindow, cv::format("FPS: %4.1f (%4.1fms)", 1000/ffmin, ffmin), cv::Point(10, 35*fontsize), cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(0, 255, 0));
    if (TwoWindow)
        cv::putText(imgWindow, cv::format("FPS: %4.1f (%4.1fms)", 1000/ffmin2, ffmin2), cv::Point(10+imgWindow.cols/2.0, 35*fontsize), cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(0, 255, 0));

    //show Raspberry Pi temperature
    if (tempDelayTick==0){
        tempDelayTick=50;
        lastTemp=GetTemp(false);
    }
    tempDelayTick--;
    if (lastTemp>85)
        cv::putText(imgWindow, cv::format("temp: %.0f 'C", lastTemp), cv::Point(10, 70*fontsize), cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(0, 0, 255));
    else
    if (lastTemp>80)
        cv::putText(imgWindow, cv::format("temp: %.0f 'C", lastTemp), cv::Point(10, 70*fontsize), cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(0, 255, 255));
    else
        cv::putText(imgWindow, cv::format("temp: %.0f 'C", lastTemp), cv::Point(10, 70*fontsize), cv::FONT_HERSHEY_SIMPLEX, fontsize, cv::Scalar(0, 255, 0));
    
    tick = cv::getTickCount();
    if (tick2on) {
        tick2on=false;
        tick2 = cv::getTickCount();
    }
    if (showGUI) {
        imshow(window_name, imgWindow);
    }
    if (useWebserver) {
        wserv.write(imgWindow);
    }

    if (viewStatus==ViewerStatus::Depth)
        return ViewerStatus::Depth;
    return ViewerStatus::Continue;
    ;
}

void closing_function(int sig)
{ 
    printf("CLOSING\n");
    if (camFile)
        capcv0.release();
    if (camOpenCV == false)
    {
        raspiCamCvReleaseCapture(&caprp0);
        raspiCamCvReleaseCapture(&caprp1);
    }
    wserv.stop();
}

void ShowTick(cv::TickMeter * tm, int v)
{
    timeSum += tm->getTimeMilli();
    tm->reset();
    timeIdx++;
    if (timeIdx >= v)
    {
        timeSum = (timeSum / timeIdx);
//        printAt(25,0,cv::format("avg time ms: %f size: %d\n", timeSum, sizeof(edgesFrame)));
        timeSum = 0.0f;
        timeIdx = 0;
    }
}

