#pragma once

#include <limits>
#include <opencv2/opencv.hpp>

#include "EventArray.h"

cv::Mat makeTencodeImage(const std::shared_ptr<EventArray> &events, int width,
                         int height);