//
// Created by 张醒之 on 2018/7/25.
//

#include "Line3f.h"

Line3f::Line3f(const Vec4i& l) {
    P0 = Point3f(l[0], l[1], 0);
    P1 = Point3f(l[2], l[3], 0);
    delta = P1 - P0;
}

int floatCompare(float a, float b){
    if (a < b-SMALL_NUM) return -1;
    if (a > b+SMALL_NUM) return 1;
    return 0;
}

int comparePoint3f(const Point3f& p1, const Point3f& p2)
{
    int res = floatCompare(p1.x, p2.x);
    if (res == 0) res = floatCompare(p1.y, p2.y);
    if (res == 0) res = floatCompare(p1.z, p2.z);
    return res;
}

bool _comparePoint3f(const Point3f& p1, const Point3f& p2) {
    return comparePoint3f(p1, p2) >= 0;
}

void Line3f::mergeLine(const Line3f& S2) {
    std::vector<Point3f> points {
        P0,
        P1,
        S2.P0,
        S2.P1
    };
    std::sort(points.begin(), points.begin() + points.size(), _comparePoint3f);
    P1 = Point3f(points[0]);
    P0 = Point3f(points[3]);
}

int Line3f::compare(const Line3f &S2) const {
    int res = comparePoint3f(S2.P0, P0);
    if (res == 0) res = comparePoint3f(S2.P1, P1);
    return res;
}

// dist3D_Segment_to_Segment(): get the 3D minimum distance between 2 segments
//    Input:  two 3D line segments S1 and S2
//    Return: the shortest distance between S1 and S2
float Line3f::dist(const Line3f& S2) {
    Point3f   u = P1 - P0;
    Point3f   v = S2.P1 - S2.P0;
    Point3f   w = P0 - S2.P0;
    float    a = dot(u,u);         // always >= 0
    float    b = dot(u,v);
    float    c = dot(v,v);         // always >= 0
    float    d = dot(u,w);
    float    e = dot(v,w);
    float    D = a*c - b*b;        // always >= 0
    float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
    float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
    if (D < SMALL_NUM) { // the lines are almost parallel
        sN = 0.0f;         // force using point P0 on segment S1
        sD = 1.0f;         // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                 // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0f) {        // sc < 0 => the s=0 edge is visible
            sN = 0.0f;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0f) {            // tc < 0 => the t=0 edge is visible
        tN = 0.0f;
        // recompute sc for this edge
        if (-d < 0.0f)
            sN = 0.0f;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0f)
            sN = 0.0f;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d +  b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (abs(sN) < SMALL_NUM ? 0.0f : sN / sD);
    tc = (abs(tN) < SMALL_NUM ? 0.0f : tN / tD);

    // get the difference of the two closest points
    Point3f   dP = w + (sc * u) - (tc * v);  // =  S1(sc) - S2(tc)

    return norm(dP);   // return the closest distance
}
