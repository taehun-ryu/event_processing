#include <iomanip>
#include <limits>
#include <opencv2/opencv.hpp>

#include "EventArray.h"
#include "H5Reader.h"
#include "Representation.h"

int main() {
  H5Reader reader("/root/datasets/self_dvs/event_cam2/250327/left_0.h5");
  auto full_events = reader.readEvents();

  std::cout << "Loaded " << full_events->size() << " events." << std::endl;

  full_events->sortByTime();
  full_events->normalizeTimestampsToStart();
  full_events->filterByTime(5.0, 6.0);

  // Tencode
  auto bins = full_events->splitByTimeBin(0.03);
  for (const auto &bin : bins) {
    cv::Mat f = makeTencodeImage(bin, 346, 260);
    cv::namedWindow("Bin View", cv::WINDOW_NORMAL);
    cv::imshow("Bin View", f);
    cv::waitKey(0);
  }

  return 0;
}
