#ifndef MIDAS_INFERENCE_H
#define MIDAS_INFERENCE_H
#include <opencv2/opencv.hpp>
#include <onnxruntime/core/session/onnxruntime_cxx_api.h>
#include <iostream>
#include <cstring>
#include <vector>
#include <string>


class MidasInference {
public:
    MidasInference(const std::string& modelPath, bool useCUDA = false);
    ~MidasInference();

    std::vector<float> PreProcess(cv::Mat& iImg);
    cv::Mat verifyOutput(std::vector<float> output, std::string pkg_path);
    cv::Mat draw_depth(const cv::Mat& depth_map, int oriW, int oriH, std::string pkg_path);
    cv::Mat runInference(cv::Mat& img, std::string pkg_path);

private:
    Ort::SessionOptions sessionOptions;
    Ort::Env env;
    Ort::Session* session;
    Ort::RunOptions run_options;
    int numthreads = 0;

    int H = 384;
    int W = 384;
    const char* input_node_name = "x.1";
    const char* output_node_name = "3195";
    
    const double set_min_depth = 0.0;
    const double set_max_depth = 100.0;
};

#endif // MIDAS_INFERENCE_H

