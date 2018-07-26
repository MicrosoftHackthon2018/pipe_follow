//
// Created by elliot-zh on 2018/7/25.
//

#include "PipelineFrame.h"
#include <iostream>

using namespace cv;
using namespace std;

PipelineFrame::PipelineFrame(const Mat &img) {
    currentImg = new Mat(img);
    originImg = new Mat(img);
}

void PipelineFrame::toSharpenGray(float grayThreshold) {
    for (int j = 0; j < currentImg->rows; j++) {
        for (int i = 0; i < currentImg->cols; i++) {
            if (currentImg->at<uchar>(j, i) > grayThreshold) {
                currentImg->at<uchar>(j, i) = 255; //white
            } else {
                currentImg->at<uchar>(j, i) = 0; //black
            }
        }
    }
}

void PipelineFrame::detectPipelineCandidates(float min_area_rate, int epsilon) {

    vector<vector<Point> > contours0;
    vector<Vec4i> hierarchy;

    cv::findContours(*currentImg, contours0, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    vector<float> areas(contours0.size());

    for (int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
        auto &contour = contours0[idx];
        float area = contourArea(contour);
        areas[idx] = area;
    }

    int largest_con_idx = 0;
    auto min_area = (*max_element(areas.begin(), areas.end())) * min_area_rate;

    for (int idx = 0; idx >= 0; idx = hierarchy[idx][0]) {
        auto area = areas[idx];
        if (area < min_area) continue;

        cv::approxPolyDP(Mat(contours0[idx]), contours0[idx], epsilon, true);
        area = contourArea(contours0[idx]);
        areas[idx] = area;
        pipeline_candidates.push_back(contours0[idx]);
        if (area > areas[largest_con_idx]) {
            largest_con_idx = idx;
        }
    }
    largestContour = contours0[largest_con_idx];
}

PipelineFrame::~PipelineFrame() {
    delete currentImg;
    delete originImg;
}