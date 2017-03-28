#include"base.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/video/tracking.hpp>
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
    fr2.push_back(frame2.kp[good_mathches[i].trainIdx].pt); 
  } 
  cv::Mat  H=findHomography(fr1,fr2,CV_RANSAC);
  //the points of the frame1
  std::vector<cv::Point2f> object_corners(4);
  object_corners[0]=cvPoint(0,0);
  object_corners[1]=cvPoint(frame1.rgb.cols,0);
  object_corners[2]=cvPoint(frame1.rgb.cols,frame1.rgb.rows);
  object_corners[3]=cvPoint(0,frame1.rgb.rows);
  std::vector<cv::Point2f> sence_corners(4);
  cv::Mat dst;
 
 //warp the first picture
  vector<cv::Point2f> dstP;
  cv::warpPerspective(frame1.rgb,dst,H,frame2.rgb.size());
  cv::imshow("after",dst);
 
  //warp four points(corners) of the first picture 
 cv::perspectiveTransform(object_corners,sence_corners,H);
  
  //draw the line　in the second picture
  cv::line(imgresult,sence_corners[0]+cv::Point2f(frame1.rgb.cols,0),sence_corners[1]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  cv::line(imgresult,sence_corners[1]+cv::Point2f(frame1.rgb.cols,0),sence_corners[2]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  cv::line(imgresult,sence_corners[2]+cv::Point2f(frame1.rgb.cols,0),sence_corners[3]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  cv::line(imgresult,sence_corners[3]+cv::Point2f(frame1.rgb.cols,0),sence_corners[0]+cv::Point2f(frame1.rgb.cols,0),cv::Scalar(0,255,0),4);
  
 // cv::imshow("good_mathches&object_detect",imgresult);
 cv::imshow("imgresult",imgresult);
 

//artificial optical flow
//detect sift KeyPoint and write them into vector<Point2f> 
  vector<cv::KeyPoint> siftkeypoint;
  vector<cv::Point2f> ffr1;
  cv::SIFT sift;
  sift.detect(frame1.rgb,siftkeypoint);
  for (int k=0;k<siftkeypoint.size();k++){
    ffr1.push_back(siftkeypoint[k].pt);
    
  }
  
  //compute the second point of artificial optical flow
  vector<cv::Point2f> artificialpoint;
  cv::perspectiveTransform(ffr1,artificialpoint,H);
  //draw artificial optical flow
  cv::Mat artificial=frame1.rgb;
  int r=3;
  for (int j=0;j<ffr1.size();j++){
  cv::line(artificial,ffr1[j],artificialpoint[j],cv::Scalar(0,255,0),1);
  cv::circle(artificial,artificialpoint[j],r,cv::Scalar(0,0,255));
  }
  cv::imshow("artificial",artificial);
  //compute the DenseOpticalFlow and show 
  
  cv::Mat_<cv::Point2f> flow;
  cv::Mat out;
  cv::Ptr<cv::DenseOpticalFlow> realopticalflow=cv::createOptFlow_DualTVL1();
  cv::Mat framegray1;
  cv::Mat framegray2;
  cv::cvtColor(frame1.rgb,framegray1,CV_RGB2GRAY);
  cv::cvtColor(frame2.rgb,framegray2,CV_RGB2GRAY);
  realopticalflow->calc(framegray1,framegray2,flow);
  drawOpticalFlow(flow,out);
  imshow("out",out);
  
  
  cv::waitKey();
  return 0;
  
  /* cv::Mat imgresult;
  cv::drawMatches(frame1.rgb,frame1.kp,frame2.rgb,frame2.kp,matches,imgresult);
  cv::imshow("matches",imgresult);
  cv::waitKey();
  */
}