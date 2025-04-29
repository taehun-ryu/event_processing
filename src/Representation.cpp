#include "Representation.h"

cv::Mat makeTencodeImage(const std::shared_ptr<EventArray> &events, int width,
                         int height) {
  cv::Mat img(height, width, CV_8UC3, cv::Scalar(0, 0, 0));
  size_t N = events->size();
  if (N == 0)
    return img;

  // assume events are sorted
  double t_min = events->getEvent(0)->getTimestamp();
  double t_max = events->getEvent(N - 1)->getTimestamp();
  double delta_t = t_max - t_min;

  if (delta_t <= 0.0)
    return img;

  const double NEG_INF = -std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> ts_map(height,
                                          std::vector<double>(width, NEG_INF));
  std::vector<std::vector<int>> pol_map(height, std::vector<int>(width, 0));

  for (const auto &e_ptr : events->getEvents()) {
    double t = e_ptr->getTimestamp();
    int x = e_ptr->getPosition().x();
    int y = e_ptr->getPosition().y();
    int p = e_ptr->getPolarity();
    if (x < 0 || x >= width || y < 0 || y >= height)
      continue;

    if (t > ts_map[y][x]) {
      ts_map[y][x] = t;
      pol_map[y][x] = p;
    }
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double t = ts_map[y][x];
      if (t == NEG_INF)
        continue;

      double norm = (t_max - t) / delta_t;
      norm = std::clamp(norm, 0.0, 1.0);
      uint8_t g = static_cast<uint8_t>(norm * 255);

      cv::Vec3b &pix = img.at<cv::Vec3b>(y, x);
      if (pol_map[y][x] > 0) {
        pix = cv::Vec3b(0, g, 255); // ON
      } else {
        pix = cv::Vec3b(255, g, 0); // OFF
      }
    }
  }

  return img;
}