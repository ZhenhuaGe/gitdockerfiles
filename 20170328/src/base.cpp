#include"base.h"
#include <boost/concept_check.hpp>
using namespace cv;
void ComputeFeatureandDesc(FRAME& frame)
{
  cv::ORB orb;
  orb.detect(frame.rgb,frame.kp);
  orb.compute(frame.rgb,frame.kp,frame.desp);
  return;
}
vector<cv::DMatch> cmatch(FRAME& frame1,FRAME& frame2){
  vector<cv::DMatch> matches;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  matcher.match(frame1.desp,frame2.desp,matches);
  return matches;
}
 bool isFlowCorrect(Point2f u){
  return !cvIsNaN(u.x)&&!cvIsNaN(u.y)&&std::fabs(u.y)<1e9;
  }
static Vec3b computeColor(float fx, float fy){
  static bool first=true;
  const int Ry=15;
  const int Yg=6;
  const int Gc=4;
  const int Cb=11;
  const int Bm=13;
  const int Mr=6;
  const int NCOLS=Ry+Yg+Gc+Cb+Bm+Mr;
  static Vec3i colorWheel[NCOLS];
  if (first){
    int k=0;
    for(int i=0;i<Ry;++i,++k)
      colorWheel[k]=Vec3i(255,255*i/Ry,0);
    for(int i=0;i<Yg;++i,++k)
      colorWheel[k]=Vec3i(255-255*i/Yg,255,0);
    for(int i=0;i<Gc;++i,++k)
      colorWheel[k]=Vec3i(0,255,255*i/Gc);
    for(int i=0;i<Cb;++i,++k)
      colorWheel[k]=Vec3i(0,255-255*i/Cb,255);
    for(int i=0;i<Bm;++i,++k)
      colorWheel[k]=Vec3i(255*i/Bm,0,255);
    for(int i=0;i<Mr;++i,++k)
      colorWheel[k]=Vec3i(255,0,255-255*i/Mr);
    first=false;
  }
  const float rad=sqrt(fx*fx+fy*fy);
  const float a=atan2(-fy,-fx)/(float)CV_PI;
  const float fk=(a+1.0f)/2.0f*(NCOLS-1);
  const int k0=static_cast<int>(fk);
  const int k1=(k0+1)%NCOLS;
  const float f=fk-k0;
  Vec3b pix;
  for(int b=0;b<3;b++){
    const float col0=colorWheel[k0][b]/255.f;
    const float col1=colorWheel[k1][b]/255.f;
    float col=(1-f)*col0+f*col1;
    if(rad<=1)
      col=1-rad*(1-col);
    else
      col*=.75;
    pix[2-b]=static_cast<uchar> (255.f*col);
    }
    return pix;
  
}
void drawOpticalFlow(const cv::Mat_<cv::Point2f>& flow, cv::Mat& dst,float maxmotion)//
{
  dst.create(flow.size(),CV_8UC3);
  dst.setTo(Scalar::all(0));
  //determine motion range
  float maxrad=maxmotion;
  if (maxmotion<=0){
    maxrad=1;
    for (int y=0;y<flow.rows;++y){
      for(int x=0;x<flow.cols;++x){
	Point2f u=flow(y,x);
	if(!isFlowCorrect(u))
	  continue;
	maxrad=max(maxrad,std::sqrt(u.x*u.x+u.y*u.y));
      }
    }
  }
  for (int y=0;y<flow.rows;++y){
    for(int x=0;x<flow.cols;++x){
      Point2f u=flow(y,x);
      if(isFlowCorrect(u))
	dst.at<Vec3b>(y,x)=computeColor(u.x/maxrad,u.y/maxrad);
      
    }
    
  }
}