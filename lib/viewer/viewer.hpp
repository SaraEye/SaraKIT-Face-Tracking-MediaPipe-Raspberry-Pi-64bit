#include <algorithm>
#include <numeric>

#include "../../struct.hpp"
#include "MJPEGWriter/MJPEGWriter.h"

void init_viewer(int firstViewMode=0, int secondViewMode=-1, float scaleView=1, bool ShowGUI=false, bool ShowOnWWW=true);
void closing_function(int sig);

ViewerStatus viewer_refresh();
void ShowTick(cv::TickMeter * tm, int v);
