//
// Created by 张醒之 on 2018/7/25.
//

#ifndef DETECT_PIPELINE_LINE3F_H
#define DETECT_PIPELINE_LINE3F_H
// Copyright 2001 softSurfer, 2012 Dan Sunday
// This code may be freely used, distributed and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.


// Assume that classes are already given for the objects:
//    Point and Vector with
//        coordinates {float x, y, z;}
//        operators for:
//            Point   = Point ± Vector
//            Vector =  Point - Point
//            Vector =  Vector ± Vector
//            Vector =  Scalar * Vector
//    Line and Segment with defining points {Point  P0, P1;}
//    Track with initial position and velocity vector
//            {Point P0;  Vector v;}
//===================================================================

#include <opencv2/opencv.hpp>
using namespace cv;

#define SMALL_NUM   0.00000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)
#define norm(v)    sqrt(dot(v,v))  // norm = length of  vector
#define d(u,v)     norm(u-v)        // distance = norm of difference

class Line3f {
public:
    Line3f(const Vec4i& l);
    Point3f P0, P1, delta;
    float dist(const Line3f& S2);
    void mergeLine(const Line3f& S2);
    int compare(const Line3f& S2) const;
};

#endif //DETECT_PIPELINE_LINE3F_H
