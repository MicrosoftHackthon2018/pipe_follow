//
// Created by 张醒之 on 2018/7/25.
//

#include "PipelineVideo.h"
#include <iostream>

using namespace std;


float getPointDist(const Point &point1, const Point &point2)
{
    return sqrt(pow((float)(point1.x - point2.x), 2) + pow((float)(point1.y - point2.y), 2));
}

float getPointDist(const Point &point)
{
    return sqrt((float)(point.x * point.x + point.y * point.y));
}

float PipelineVideo::getDist(const Point &point) const
{
    return getPointDist(point, currentPosition);
}

Point getCenter(const vector<Point> &points)
{
    float sum_x = 0.0f, sum_y = 0.0f;
    for (size_t i = 0; i < points.size(); i++) {
        sum_x += points[i].x;
        sum_y += points[i].y;
    }
    sum_x /= points.size();
    sum_y /= points.size();
    return Point(sum_x, sum_y);
}

Point getCenter(const Point &p1, const Point & p2)
{
    return Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
}


bool getCircleSegmentIntersection(
    const Point &center,
    float r,
    const Point &start,
    const Point &end,
    Point &intersection
)
{
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float dr = sqrt(pow(dx, 2) + pow(dy, 2));
    float d = (start.x - center.x) * (end.y - center.y) - (start.y - center.y) * (end.x - center.x);
    float sgn_dy = dy < 0.0f ? -1.0f : 1.0f;
    float delta = pow(r * dr, 2) - pow(d, 2);
    float sqrt_delta = sqrt(delta);
    float x = (d * dy + sgn_dy * dx * sqrt_delta) / pow(dr, 2);
    float y = (-d * dx + abs(dy) * sqrt_delta) / pow(dr, 2);
    intersection.x = center.x + x;
    intersection.y = center.y + y;
    if ((intersection.x - start.x) * (intersection.x - end.x) > 0 ||
        (intersection.y - start.y) * (intersection.y - end.y) > 0) {
        intersection.x = center.x + (d * dy - sgn_dy * dx * sqrt_delta) / pow(dr, 2);
        intersection.y = center.y + (-d * dx - abs(dy) * sqrt_delta) / pow(dr, 2);
    }
    return !((intersection.x - start.x) * (intersection.x - end.x) > 0 ||
        (intersection.y - start.y) * (intersection.y - end.y) > 0);
}

PipelineVideo::PipelineVideo(int memory_size) :
    last_memory(nullptr),
    r(200.0f)
{
    this->memory_size = memory_size;
}

void PipelineVideo::appendFrame(const Mat &frame)
{
    delete last_memory;
    last_memory = new PipelineFrame(frame);
    float grayThreshold = 0.7f * 255.0f;
    last_memory->toSharpenGray(grayThreshold);
    last_memory->detectPipelineCandidates(0.3f);

    if (relative_destinations.empty()) {
        currentPosition = Point(frame.cols / 2, frame.rows);
        relative_destinations.emplace_back(currentPosition);
        r = (int)(frame.rows * 0.75f);
    }
    crossPoints.clear();
    abnormal_status = 0;

    detectPipeline();
}

bool compare_x(const Point& p1, const Point & p2)
{
    return p1.x > p2.x;
}


vector<Point> PipelineVideo::detectDestinationCandidate(const vector<Point> &contour)
{
    vector<Point> temp_crossPoints;
    vector<Point> destination_candidates;

    float last_dist = getDist(contour[contour.size() - 1]) - r;
    float nearest_dist = abs(last_dist);
    Point nearestPoint = contour[contour.size() - 1];
    for (unsigned long idx = 0; idx < contour.size(); idx++) {
        float next_dist = getDist(contour[idx]) - r;
        if (last_dist * next_dist <= 0) {
            Point intersection;
            if (getCircleSegmentIntersection(
                currentPosition,
                r,
                contour[idx == 0 ? contour.size() - 1 : idx - 1],
                contour[idx],
                intersection)) {
                temp_crossPoints.push_back(intersection);
            }
        }
        last_dist = next_dist;
        if (abs(last_dist) < nearest_dist) {
            nearest_dist = abs(last_dist);
            nearestPoint = contour[idx];
        }
    }

    for (size_t i = 0; i < temp_crossPoints.size(); i++) {
        crossPoints.push_back(temp_crossPoints[i]);
    }

    switch (temp_crossPoints.size()) {
    case 0:
        destination_candidates.push_back(nearestPoint - currentPosition);
        break;
    case 1:
        destination_candidates.push_back(temp_crossPoints[0] - currentPosition);
        break;
    case 2:
        destination_candidates.push_back(getCenter(temp_crossPoints) - currentPosition);
        break;
    default:
        std::sort(temp_crossPoints.begin(), temp_crossPoints.end(), compare_x);
        destination_candidates.push_back(getCenter(temp_crossPoints[0], temp_crossPoints[1]) - currentPosition);
        destination_candidates.push_back(getCenter(
                temp_crossPoints[temp_crossPoints.size() - 1],
                temp_crossPoints[temp_crossPoints.size() - 2]) - currentPosition);
        break;
    }
    return destination_candidates;
}

Point PipelineVideo::getRecentRelativeDestinationCenter(int step)
{
    return relative_destinations[relative_destinations.size() - 1];
}

void PipelineVideo::detectPipeline()
{
    // no contour is detected, keep previous state
    if (last_memory->pipeline_candidates.empty()) {
        return;
    }

    // gather destination candidates
    relative_destination_candidates.clear();
    vector<int> pipeline_dictionary;
    if (pipelines.empty()) {
        vector<Point> relative_destinations = detectDestinationCandidate(last_memory->largestContour);
        for (size_t i = 0; i < relative_destinations.size(); i++) {
            relative_destination_candidates.push_back(relative_destinations[i]);
            pipeline_dictionary.push_back(-1);
        }
    } else {
        for (unsigned long idx = 0; idx < last_memory->pipeline_candidates.size(); ++idx) {
            auto pipelineCandidate = last_memory->pipeline_candidates[idx];
            vector<Point> relative_destinations = detectDestinationCandidate(pipelineCandidate);
            for (size_t i = 0; i < relative_destinations.size(); i++) {
                relative_destination_candidates.push_back(relative_destinations[i]);
                pipeline_dictionary.push_back(int(idx));
            }
        }
    }

    // determine nearest destination candidate
    unsigned long nearest_idx = 0;
    float min_dist = -1.0f;
    Point recentRelativeDestinationCenter = getRecentRelativeDestinationCenter();
    for (size_t idx = 0; idx < relative_destination_candidates.size(); ++idx) {
        float dist = getPointDist(relative_destination_candidates[idx], recentRelativeDestinationCenter) +
            abs(getPointDist(relative_destination_candidates[idx]) - r);
        if (min_dist < 0 || dist < min_dist) {
            min_dist = dist;
            nearest_idx = idx;
        }
    }

    // set destination & pipeline
    double max_step = -1;
//    if (relative_destination_candidates.size() > 1) {
//        cout << relative_destination_candidates.size() << endl;
//    }
    if (!pipelines.empty() && min_dist > 50) {
        abnormal_status = 1;
    }

    if (min_dist >= 0.0f && (pipelines.empty() || max_step < 0.0f || min_dist < max_step)) {
        pipelines.clear();
        int p_idx = pipeline_dictionary[nearest_idx];
        if (p_idx < 0) {
            pipelines.push_back(last_memory->largestContour);
        } else {
            pipelines.push_back(last_memory->pipeline_candidates[p_idx]);
        }
        relative_destinations.push_back(relative_destination_candidates[nearest_idx]);
    } else {
        relative_destinations.emplace_back(Point(-1, -1));
    }
}

void PipelineVideo::showBack()
{
    // last_memory->show();
    Mat origin_frame;
    cv::cvtColor(*(last_memory->originImg), origin_frame, cv::COLOR_GRAY2BGR);

    // draw all pipeline candidate
    for (size_t idx = 0; idx < last_memory->pipeline_candidates.size(); ++idx) {
        drawContours(origin_frame, last_memory->pipeline_candidates, idx, Scalar(0, 0, 155), 2);
    }
    // draw pipeline
    for (size_t idx = 0; idx < pipelines.size(); ++idx) {
        drawContours(origin_frame, pipelines, idx, Scalar(0, 0, 255), 2);
    }

    // draw cross points
    circle(origin_frame, currentPosition, int(getDestinationDist()), Scalar(0, 155, 0));
    for (const Point &point : crossPoints) {
        circle(origin_frame, point, 2, Scalar(0, 255, 0), 2);
    }

    // draw all destination candidates
    for (const Point &point : relative_destination_candidates) {
        circle(origin_frame, point + currentPosition, 2, Scalar(0, 150, 150), 2);
    }
    // draw destination
    circle(origin_frame, getCurrentAbsoluteDestination(), 2, Scalar(255, 0, 0), 2);

    imshow("video", origin_frame);
}

float PipelineVideo::getDestinationDist()
{
    return getDist(getCurrentAbsoluteDestination());
}

Point PipelineVideo::getCurrentAbsoluteDestination()
{
    return relative_destinations[relative_destinations.size() - 1] + currentPosition;
}

void PipelineVideo::getCurrentAbsoluteDestination(int *x, int *y)
{
    Point p = getCurrentAbsoluteDestination();
    *x = p.x; *y = p.y;
}

int PipelineVideo::append(void *buffer, int width, int height, uint32_t frame_num)
{
    Mat frame = cv::Mat(height, width, CV_8UC1, buffer);
    cv::imwrite("/hover/dump/" + to_string(frame_num) + ".jpg", frame);
    appendFrame(frame);
    return 0;
}
