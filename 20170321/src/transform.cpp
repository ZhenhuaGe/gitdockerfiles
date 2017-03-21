#include"base.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>

int main(int argc,char **argv){
  FRAME frame1,frame2;
  frame1.rgb=cv::imread(argv[1],1);
  frame2.rgb=cv::imread(argv[2],1);
  ComputeFeatureandDesc(frame1);
  ComputeFeatureandDesc(frame2);
  vector<cv::DMatch> matches=cmatch(frame1,frame2);
  cv::Mat imgresult;
  cv::drawMatches(frame1.rgb,frame1.kp,frame2.rgb,frame2.kp,matches,imgresult);
  cv::imshow("matches",imgresult);
  cv::waitKey();
  
}