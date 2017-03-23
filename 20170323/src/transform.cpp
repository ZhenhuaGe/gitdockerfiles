#include"base.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include <boost/concept_check.hpp>

int main(int argc,char **argv){
  FRAME frame1,frame2;
  frame1.rgb=cv::imread(argv[1],1);
  frame2.rgb=cv::imread(argv[2],1);
  if(!frame1.rgb.data||!frame2.rgb.data){
    std::cout<<"--Error image"<<std::endl;
   return -1;
  }
  ComputeFeatureandDesc(frame1);
  ComputeFeatureandDesc(frame2);
  vector<cv::DMatch> matches=cmatch(frame1,frame2);
  double max_dist=0;
  double min_dist=100;
  //compute the max and min distance between keypoints(using the descriptors)
  for (int i=0;i<frame1.desp.rows;i++){
    double dist=matches[i].distance;
    if (dist<min_dist) min_dist=dist;
    if (dist>max_dist) max_dist=dist;  
  }
  std::cout<<"max_dist="<<max_dist<<std::endl;
  std::cout<<"min_dist="<<min_dist<<std::endl;
  //find good matches
  std::vector<cv::DMatch> good_mathches;
  for(int i=0;i<frame2.desp.rows;i++){
    if(matches[i].distance<3*min_dist)
    {good_mathches.push_back(matches[i]);}  
  }
  cv::Mat imgresult;
  cv::drawMatches(frame1.rgb,frame1.kp,frame2.rgb,frame2.kp,good_mathches,imgresult);
  //localize the object
  std::vector<cv::Point2f> fr1;
  std::vector<cv::Point2f> fr2;
  for (int i=0;i<good_mathches.size();i++)
  {
    fr1.push_back(frame1.kp[good_mathches[i].queryIdx].pt);
    fr2.push_back(frame2.kp[good_mathches[i].queryIdx].pt); 
  } 
  cv::Mat  H=findHomography(fr1,fr2,CV_RANSAC);
  //the points of the frame1
  std::vector<cv::Point2f> object_corners(4);
  object_corners[0]=cvPoint(0,0);
  object_corners[1]=cvPoint(frame1.rgb.cols,0);
  object_corners[2]=cvPoint(frame1.rgb.cols,frame1.rgb.rows);
  object_corners[3]=cvPoint(0,frame1.rgb.rows);
  std::vector<cv::Point2f> sence_corners(4);
  cv::perspectiveTransform(object_corners,sence_corners,H);
  
  //draw the line
  cv::line(imgresult,sence_corners[0]+cv::Point2f(frame1.rgb.cols,0),sence_corners[1]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  cv::line(imgresult,sence_corners[1]+cv::Point2f(frame1.rgb.cols,0),sence_corners[2]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  cv::line(imgresult,sence_corners[2]+cv::Point2f(frame1.rgb.cols,0),sence_corners[3]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  cv::line(imgresult,sence_corners[3]+cv::Point2f(frame1.rgb.cols,0),sence_corners[0]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  
  cv::imshow("good_mathches&object_detect",imgresult);
  cv::waitKey();
  return 0;
  
  /* cv::Mat imgresult;
  cv::drawMatches(frame1.rgb,frame1.kp,frame2.rgb,frame2.kp,matches,imgresult);
  cv::imshow("matches",imgresult);
  cv::waitKey();
  */
}