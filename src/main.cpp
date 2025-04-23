#include <iomanip>

#include "H5Reader.h"
#include "EventArray.h"

int main() {
  H5Reader reader("/root/datasets/self_dvs/event_cam2/250327/left.h5");
  auto full_events = reader.readEvents();

  std::cout << "Loaded " << full_events->size() << " events." << std::endl;

  full_events->sortByTime();
  full_events->normalizeTimestampsToStart();
  //full_events->filterByTime(5, 6);

  auto bins = full_events->splitByTimeBin(0.1);  // 10ms
  std::cout << "Split into " << bins.size() << " time bins." << std::endl;

  return 0;
}
