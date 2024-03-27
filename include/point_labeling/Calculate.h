#include "point_labeling/Points.h"

bool compareLabel(const pointCorner& _corner_1, const pointCorner& _corner_2)
{
    return _corner_1.label < _corner_2.label;
}

class Calculate
{
    public:
        double distance(const cv::Point& _point_1, const cv::Point& _point_2);
        double angle(const cv::Point& _point_1, const cv::Point& _point_2);
        int minIndex(const std::vector<double>& _vector);

        cv::Point translate(const cv::Point& _prev_point, const cv::Point& _moved_point);
        cv::Point cornerCentroid(const std::vector<cv::Point>& _points);
        cv::Point cornerCentroid(const std::vector<pointCorner>& _points);

        std::vector<cv::Point> angleOrdering(const std::vector<cv::Point>& _corners);
        void labelOrdering(std::vector<pointCorner>& _corners);

    Calculate(){}
    ~Calculate(){}
};

double Calculate::distance(const cv::Point& _point_1, const cv::Point& _point_2)
{
    return std::sqrt(std::pow(_point_1.x-_point_2.x,2)+std::pow(_point_1.y-_point_2.y,2));
}

double Calculate::angle(const cv::Point& _point_1, const cv::Point& _point_2)
{
    return std::atan2(_point_2.y-_point_1.y,_point_2.x-_point_1.x);
}

int Calculate::minIndex(const std::vector<double>& _vector)
{
    return std::min_element(_vector.begin(), _vector.end())-_vector.begin();
}

cv::Point Calculate::translate(const cv::Point& _prev_point, const cv::Point& _moved_point)
{
    //cv::Point trans(_prev_point.x-_moved_point.x, _prev_point.y-_moved_point.y);
    cv::Point trans(_moved_point.x-_prev_point.x, _moved_point.y-_prev_point.y);
    return trans;
}

cv::Point Calculate::cornerCentroid(const std::vector<cv::Point>& _points)
{
    cv::Point centroid(0,0);
    for(int i = 0; i < _points.size(); i++)
    {
        centroid.x += _points[i].x;
        centroid.y += _points[i].y;
    }
    centroid.x /= _points.size();
    centroid.y /= _points.size();
    return centroid;
}

cv::Point Calculate::cornerCentroid(const std::vector<pointCorner>& _points)
{
    cv::Point centroid(0,0);
    for(int i = 0; i < _points.size(); i++)
    {
        centroid.x += _points[i].coord.x;
        centroid.y += _points[i].coord.y;
    }
    centroid.x /= _points.size();
    centroid.y /= _points.size();
    return centroid;
}

std::vector<cv::Point> Calculate::angleOrdering(const std::vector<cv::Point>& _corners)
{
    cv::Point centroid = cornerCentroid(_corners);
    std::vector<double> angle_vec;
    std::vector<cv::Point> ordered_corners;
    for(const auto& corner : _corners)
    {
        double temp_angle = angle(centroid, corner);
        angle_vec.push_back(temp_angle);
    }
    for(int i = 0; i < _corners.size(); i++)
    {
        int min_idx = minIndex(angle_vec);
        ordered_corners.push_back(_corners[min_idx]);
        angle_vec[min_idx] = INFINITY;
    }
    return ordered_corners;
}

void Calculate::labelOrdering(std::vector<pointCorner>& _corners)
{
    std::sort(_corners.begin(), _corners.end(), compareLabel);
}