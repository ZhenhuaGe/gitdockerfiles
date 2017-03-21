#include"base.h"
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