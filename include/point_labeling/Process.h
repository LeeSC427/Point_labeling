#include "point_labeling/Calculate.h"

class Process
{
    public:
        Calculate cal;
        cv::Scalar low_HSV = cv::Scalar(40,50,50);
        cv::Scalar up_HSV = cv::Scalar(80,255,255);
        
        ros::NodeHandle nh_img;
        ros::Subscriber sub_img;
        std::mutex mtx_img;
        std::vector<pointCorner> prev_corner;
        bool initial_flag;

        void subscribe();
        void loop(const sensor_msgs::ImageConstPtr& _msg);

        cv::Mat Preprocess(cv::Mat& _img);

        std::vector<cv::Point> findContour(cv::Mat& _img, cv::Mat& _proc_img);
        std::vector<cv::Point> findCorner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point>& _contour);

        std::vector<pointCorner> findFirstCorner(std::vector<cv::Point>& _corner_vec);
        //std::vector<pointCorner> findMatchingCorner(std::vector<cv::Point>& _corner_vec);

        void drawCorner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<pointCorner> _corner_vec);
        void showVideo(cv::Mat& _img, cv::Mat& _proc_img);

    Process(): initial_flag(true){}
    ~Process(){}
};

void Process::subscribe()
{
    sub_img = nh_img.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, &Process::loop, this);
}

void Process::loop(const sensor_msgs::ImageConstPtr& _msg)
{
    mtx_img.lock();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        return;
    }
    cv::Mat img;
    cv_ptr->image.copyTo(img);
    mtx_img.unlock();

    cv::Mat proc_img = Preprocess(img);
    std::vector<cv::Point> contour = findContour(img, proc_img);
    if(!contour.empty())
    {
        std::vector<cv::Point> corner_vec = findCorner(img, proc_img, contour);
        std::vector<pointCorner> ordered_corner_vec = findFirstCorner(corner_vec);
        prev_corner = ordered_corner_vec;
        for(int i = 0; i < prev_corner.size(); i++)
        {
            ROS_INFO("Prev_corner Corner %d, Coordinate: (%d, %d)", i, prev_corner[i].coord.x, prev_corner[i].coord.y);
        }
    }
    else
    {
        prev_corner.clear();
    }
    showVideo(img, proc_img);
}

cv::Mat Process::Preprocess(cv::Mat& _img)
{
    cv::Mat proc_img;
    cv::cvtColor(_img, proc_img, cv::COLOR_BGR2HSV);
    cv::inRange(proc_img, low_HSV, up_HSV, proc_img);
    return proc_img;
}

std::vector<cv::Point> Process::findContour(cv::Mat& _img, cv::Mat& _proc_img)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(_proc_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    int largestContourIndex = 0;
    double largestArea = 0;
    std::vector<cv::Point> largestContour;
    largestContour.clear();

    if(contours.size() != 0)
    {
        for(int i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if(area > largestArea)
            {
                largestArea = area;
                largestContourIndex = i;
            }
        }
        cv::drawContours(_img, contours, largestContourIndex, cv::Scalar(0,255,255), 3);
        cv::drawContours(_proc_img, contours, largestContourIndex, cv::Scalar(0,255,255), 3);
        largestContour = contours[largestContourIndex];
    }
    return largestContour;
}

std::vector<cv::Point> Process::findCorner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<cv::Point>& _contour)
{
    double epsilon = 0.1 * cv::arcLength(_contour, true);
    std::vector<cv::Point> corners;
    std::vector<cv::Point2f> double_corners;
    //cv::goodFeaturesToTrack(_img, corners, 4, 0.01, 10, cv::Mat(), 3, false, 0.04);
    cv::TermCriteria termcriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1);
    cv::approxPolyDP(_contour, double_corners, epsilon, true);
    cv::cornerSubPix(_proc_img, double_corners, cv::Size(11,11), cv::Size(-1,-1), termcriteria);
    for(int i = 0; i < double_corners.size(); i++)
    {
        cv::Point temp_corner;
        temp_corner.x = (int)double_corners[i].x;
        temp_corner.y = (int)double_corners[i].y;
        corners.push_back(temp_corner);
    }

    return corners;
}

std::vector<pointCorner> Process::findFirstCorner(std::vector<cv::Point>& _corner_vec)
{
    std::vector<cv::Point> ordered_vec;
    std::vector<pointCorner> ordered_corner;

    ordered_vec = cal.angleOrdering(_corner_vec);
    for(int i = 0; i < ordered_vec.size(); i++)
    {
        pointCorner corner;
        corner.coord = ordered_vec[i];
        corner.label = i;
        ordered_corner.push_back(corner);
        ROS_INFO("IN findFirstCorner: Corner %d, Coordinate: (%d, %d)", i, corner.coord.x, corner.coord.y);
    }
    return ordered_corner;
}

//std::vector<pointCorner> Process::findMatchingCorner(std::vector<cv::Point>& _corner_vec)
//{
//    
//}

void Process::drawCorner(cv::Mat& _img, cv::Mat& _proc_img, std::vector<pointCorner> _corner_vec)
{
    for(int i = 0; i < _corner_vec.size(); i++)
    {
        cv::circle(_img, _corner_vec[i].coord, 4, cv::Scalar(0,0,255), -1);
        cv::putText(_img, std::to_string(_corner_vec[i].label), _corner_vec[i].coord, cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255,0,0), 3);
    }
}

void Process::showVideo(cv::Mat& _img, cv::Mat& _proc_img)
{
    cv::Mat resize_img;
    cv::Mat resize_proc_img;

    if(_img.cols >= 1920 || _img.rows >= 1080)
    {
        cv::resize(_img, resize_img, cv::Size(_img.size[1] / 2, _img.size[0] / 2));
        cv::resize(_proc_img, resize_proc_img, cv::Size(_proc_img.size[1] / 2, _proc_img.size[0] / 2));
    }
    else
    {
        resize_img = _img;
        resize_proc_img = _proc_img;
    }
    cv::imshow("Filtered image", resize_proc_img);
    cv::imshow("Original image", resize_img);
    cv::moveWindow("Filtered image", 960, 640);
    cv::moveWindow("Original image", 960, 0);
    cv::waitKey(1);
}