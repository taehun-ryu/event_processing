#include <chrono>
#include <deque>
#include <dvs_msgs/msg/event_array.hpp>
#include <pangolin/pangolin.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

// Just visualize (no need to use Event class in EventArray.h)
struct EventData {
  float x;
  float y;
  double time;
  bool polarity;
};

class DVS3DVisualizer : public rclcpp::Node {
public:
  DVS3DVisualizer() : Node("dvs_3d_visualizer"), start_time_set_(false) {
    using std::placeholders::_1;

    // QoS: SensorData (BEST_EFFORT)
    rclcpp::SensorDataQoS qos;
    subscription_ = this->create_subscription<dvs_msgs::msg::EventArray>(
        "/dvs/events", qos,
        std::bind(&DVS3DVisualizer::EventCallback, this, _1));

    // Pangolin window
    pangolin::CreateWindowAndBind("DVS 3D Visualizer", 1280, 720);
    glEnable(GL_DEPTH_TEST);

    s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrixOrthographic(250, -250,  // x
                                               -250, 250,  // y
                                               -2000, 2000 // z
                                               ),
        pangolin::ModelViewLookAt(0, 0, 1000, 0, 0, 0, 0, -1, 0));

    d_cam_ = &pangolin::CreateDisplay()
                  .SetBounds(0.0, 1.0, 0.0, 1.0)
                  .SetAspect(450.0f / 350.0f)
                  .SetHandler(new pangolin::Handler3D(s_cam_));
  }

  void Run() {
    while (!pangolin::ShouldQuit()) {
      rclcpp::spin_some(this->get_node_base_interface());
      RenderFrame();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
  }

private:
  std::mutex event_mutex_;
  void EventCallback(const dvs_msgs::msg::EventArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(event_mutex_);

    double latest_event_time = -1.0;
    for (const auto &e : msg->events) {
      double event_time = e.ts.sec + e.ts.nanosec * 1e-9;
      if (!start_time_set_) {
        start_time_ = event_time;
        start_time_set_ = true;
      }
      double t_rel;
      t_rel = event_time - start_time_;
      events_.push_back({static_cast<float>(e.x), static_cast<float>(e.y),
                         static_cast<double>(t_rel), e.polarity});

      if (event_time > latest_event_time) {
        latest_event_time = event_time;
      }
    }

    if (latest_event_time > 0) {
      while (!events_.empty() &&
             (events_.front().time - latest_event_time) > 0.05) {
        events_.pop_front();
      }
    }
  }

  void RenderFrame() {
    std::deque<EventData> events_copy;
    {
      std::lock_guard<std::mutex> lock(event_mutex_);
      events_copy = events_; // copy
    }

    if (events_copy.empty()) {
      pangolin::FinishFrame();
      return;
    }

    double window_size = 0.05;

    double latest_time = events_copy.back().time;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam_->Activate(s_cam_);

    glPointSize(1.0f);
    glBegin(GL_POINTS);
    for (const auto &e : events_copy) {
      if (e.time < latest_time - window_size) {
        continue; // skip event
      }

      if (e.polarity) {
        glColor3f(1.0f, 0.0f, 0.0f);
      } else {
        glColor3f(0.0f, 0.0f, 1.0f);
      }

      float time_scale = 1000.0f;
      glVertex3f((e.x - 173.0f), (e.y - 130.0f),
                 (latest_time - e.time) * time_scale);
    }
    glEnd();
    pangolin::FinishFrame();
  }

  rclcpp::Subscription<dvs_msgs::msg::EventArray>::SharedPtr subscription_;
  std::deque<EventData> events_;
  double start_time_;
  bool start_time_set_;

  pangolin::OpenGlRenderState s_cam_;
  pangolin::View *d_cam_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto visualizer = std::make_shared<DVS3DVisualizer>();
  visualizer->Run();
  return 0;
}
