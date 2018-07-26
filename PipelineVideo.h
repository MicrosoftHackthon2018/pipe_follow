//
// Created by 张醒之 on 2018/7/25.
//

#ifndef DETECT_PIPELINE_PIPELINEVEDIO_H
#define DETECT_PIPELINE_PIPELINEVEDIO_H

#include "PipelineFrame.h"
#include <queue>
#include <cstdint>

using namespace std;


class PipelineVideo {
public:
    PipelineVideo(int memory_size = 1);

    void appendFrame(const Mat &frame);

    void showBack();

    float getDestinationDist();

    float getDist(const Point &point) const;

    Point getCurrentAbsoluteDestination();
    void getCurrentAbsoluteDestination(int *x, int *y);

    int append(void *buffer, int width, int height, uint32_t frame_num);
    int getAbnormalStatus() { return abnormal_status; }

private:
    int memory_size;
    PipelineFrame *last_memory;
    vector<vector<Point>> pipelines;
    Point currentPosition;
    vector<Point> relative_destinations;

    Point getRecentRelativeDestinationCenter(int step = 4);

    float r;

    int abnormal_status;

    void detectPipeline();

    vector<Point> detectDestinationCandidate(const vector<Point> &contour);

    vector<Point> relative_destination_candidates;

    vector<Point> crossPoints;
};


#endif //DETECT_PIPELINE_PIPELINEVEDIO_H
