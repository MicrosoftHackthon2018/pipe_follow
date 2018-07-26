//
// Created by 张醒之 on 2018/7/25.
//

#ifndef DETECT_PIPELINE_PIPELINEMAT_H
#define DETECT_PIPELINE_PIPELINEMAT_H

#include <opencv2/opencv.hpp>
#include <cmath>

using namespace cv;
using namespace std;

class PipelineFrame {
public:
    void toSharpenGray(float grayThreshold);
    void detectPipelineCandidates(float min_area_rate = 0.5, int epsilon=10);
    explicit PipelineFrame(const Mat &img);
    ~PipelineFrame();

    vector<Point> largestContour;

    vector<vector<Point> > pipeline_candidates;
    Mat * originImg;

private:
    Mat * currentImg;
};


#endif //DETECT_PIPELINE_PIPELINEMAT_H
