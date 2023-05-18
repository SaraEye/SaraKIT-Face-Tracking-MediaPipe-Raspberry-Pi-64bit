#include <algorithm>
#include <numeric>

#include "../../struct.hpp"
#include "MJPEGWriter/MJPEGWriter.h"

void init_viewer();
void closing_function(int sig);

ViewerStatus viewer_refresh();
void ShowTick(cv::TickMeter * tm, int v);
