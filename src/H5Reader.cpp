#include "H5Reader.h"
#include <iostream>

H5Reader::H5Reader(const std::string &fileName)
    : fileName_(fileName), file_(nullptr) {
  try {
    file_ = new H5::H5File(fileName_, H5F_ACC_RDONLY);
  } catch (H5::FileIException &error) {
    error.printErrorStack();
    throw;
  }
}

H5Reader::~H5Reader() {
  if (file_) {
    delete file_;
    file_ = nullptr;
  }
}

EventArray::Ptr H5Reader::readEvents() {
  auto event_array = std::make_shared<EventArray>();

  try {
    H5::DataSet dataset = file_->openDataSet("events_data");
    H5::DataSpace dataspace = dataset.getSpace();

    hsize_t dims_out[2];
    dataspace.getSimpleExtentDims(dims_out, nullptr);
    size_t nrows = dims_out[0];
    size_t ncols = dims_out[1];

    if (ncols != 4) {
      std::cerr << "[H5Reader] Expected 4 columns (t, x, y, p), got " << ncols
                << std::endl;
      return event_array;
    }

    std::vector<double> flat_data(nrows * ncols);
    dataset.read(flat_data.data(), H5::PredType::NATIVE_DOUBLE);

    for (size_t i = 0; i < nrows; ++i) {
      double t = flat_data[i * 4 + 0];
      int x = static_cast<int>(flat_data[i * 4 + 1]);
      int y = static_cast<int>(flat_data[i * 4 + 2]);
      int p = static_cast<int>(flat_data[i * 4 + 3]);
      if (p == 0)
        p = -1;

      auto evt = std::make_shared<Event>(t, Event::LocType(x, y), p);
      event_array->addEvent(evt);
    }

    std::cout << "[H5Reader] Loaded " << event_array->size() << " events.\n";

  } catch (H5::Exception &error) {
    error.printErrorStack();
  }

  return event_array;
}
