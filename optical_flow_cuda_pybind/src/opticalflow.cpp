#include <pybind11/pybind11.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/imgcodecs.hpp"

#include <string>
#include <iostream>

#include "ndarray_converter.h"

namespace py = pybind11;

void show_image(cv::Mat image)
{
    cv::imshow("image_from_Cpp", image);
    cv::waitKey(0);
}

cv::Mat read_image(std::string image_name)
{
    cv::Mat image = cv::imread(image_name, cv::IMREAD_COLOR);
    return image;
}

cv::Mat passthru(cv::Mat image)
{
    return image;
}

cv::Mat cloneimg(cv::Mat image)
{
    return image.clone();
}

cv::Mat optical_flow(cv::Mat& prev, cv::Mat& curr) {
  cv::cuda::GpuMat g_frame0(prev);
  cv::cuda::GpuMat g_frame1(curr);
  cv::cuda::GpuMat g_flow(prev.size(), CV_32FC2);

  cv::Ptr<cv::cuda::OpticalFlowDual_TVL1> tvl1 = cv::cuda::OpticalFlowDual_TVL1::create();
  tvl1->calc(g_frame0, g_frame1, g_flow);

  cv::Mat flow(g_flow);
  return flow;
}

struct OpticalFlow {

  cv::cuda::GpuMat* prev = nullptr;
  cv::cuda::GpuMat* curr = nullptr;
  cv::cuda::GpuMat flow;
  cv::Ptr<cv::cuda::OpticalFlowDual_TVL1> tvl1 = cv::cuda::OpticalFlowDual_TVL1::create();

  OpticalFlow(cv::Mat first): flow(first.size(), CV_32FC2) {
    prev = new cv::cuda::GpuMat(first);
  }

  ~OpticalFlow() {
    if (prev != nullptr) {
      delete prev;
    }
    if (curr != nullptr) {
      delete curr;
    }
  }

  cv::Mat calc(cv::Mat new_frame) {
    if (curr != nullptr) {
      delete prev;
      prev = curr;
    }

    curr = new cv::cuda::GpuMat(new_frame);
    tvl1->calc(*prev, *curr, flow);

    cv::Mat mat_flow(flow);
    return mat_flow;
  }
  
};

PYBIND11_MODULE(opticalflow, m)
{
    NDArrayConverter::init_numpy();

    py::class_<OpticalFlow>(m, "OpticalFlow")
      .def(py::init<cv::Mat>())
      .def("calc", &OpticalFlow::calc, "cv::cuda::OpticalFlowDual_TVL1", py::arg("new_frame"));

    m.def("read_image", &read_image, "A function that read an image",
        py::arg("image"));

    m.def("show_image", &show_image, "A function that show an image",
        py::arg("image"));

    m.def("passthru", &passthru, "Passthru function", py::arg("image"));
    m.def("clone", &cloneimg, "Clone function", py::arg("image"));
    m.def("optical_flow", &optical_flow, "cv::cuda::OpticalFlowDual_TVL1", py::arg("prev"), py::arg("curr"));

}

