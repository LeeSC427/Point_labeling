#include "point_labeling/headers.h"

#ifndef POINT_H
#define POINT_H
class pointCorner
{
    public:
        cv::Point coord;
        int label;

    pointCorner(){}
    ~pointCorner(){}
};
#endif