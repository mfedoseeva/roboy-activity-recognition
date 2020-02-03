#include "opencv2/cudaoptflow.hpp"
#include "opencv2/imgcodecs.hpp"

#include <iostream>

int main(int argc, const char* argv[]) {
  std::string img1_path = "/usr/share/opencv-cuda/samples/data/basketball1.png";
  std::string img2_path = "/usr/share/opencv-cuda/samples/data/basketball2.png";

  cv::Mat frame0 = imread(img1_path, cv::IMREAD_GRAYSCALE);
  cv::Mat frame1 = imread(img2_path, cv::IMREAD_GRAYSCALE);

  cv::cuda::GpuMat g_frame0(frame0);
  cv::cuda::GpuMat g_frame1(frame1);
  cv::cuda::GpuMat g_flow(frame0.size(), CV_32FC2);

  cv::Ptr<cv::cuda::OpticalFlowDual_TVL1> tvl1 = cv::cuda::OpticalFlowDual_TVL1::create();

  tvl1->calc(g_frame0, g_frame1, g_flow);

  cv::Mat flow(g_flow);
  std::cout << flow << std::endl;
}
