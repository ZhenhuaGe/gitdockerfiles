# pragma once

// 各种头文件 
// C++标准库
#include <fstream>
#include <vector>
#include <map>
using namespace std;

// OpenCV
//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/core/eigen.hpp>

struct FRAME
{
    cv::Mat rgb; //该帧对应的彩色图与深度图
    cv::Mat desp;       //特征描述子
    vector<cv::KeyPoint> kp; //关键点
};

void ComputeFeatureandDesc(FRAME& frame);
vector<cv::DMatch> cmatch(FRAME& frame1,FRAME& frame2);
void motionToColor(cv::Mat flow, cv::Mat &color) ;