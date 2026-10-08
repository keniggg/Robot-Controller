#include "alicia_d_driver/gui_direct_profile.hpp"
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
int main(){
 GuiDirectProfileLimits l; l.velocity=.26; l.acceleration=1; l.jerk=10; l.maximum_tick=.02;
 GuiDirectProfile p(l); p.seed(std::vector<double>(6,0)); std::vector<double> samples; size_t reject=0;
 for(int i=0;i<3000;i++){
  const size_t axis=i%6; const double goal=((i/6)%2?-.1:.1);
  const auto a=std::chrono::steady_clock::now(); const bool ok=p.request(axis,goal);
  const auto b=std::chrono::steady_clock::now(); samples.push_back(std::chrono::duration<double,std::milli>(b-a).count());
  if(!ok)++reject;
  p.advance(.02);const auto q=p.positions();if(q.size()!=6)return 2;
 }
 std::sort(samples.begin(),samples.end());
 std::cout<<"{\"kind\":\"offline_bursty_redirect_microbenchmark\",\"requests\":"<<samples.size()<<",\"rejected_redirects\":"<<reject<<",\"p50_ms\":"<<samples[samples.size()/2]<<",\"p95_ms\":"<<samples[size_t(samples.size()*.95)]<<",\"max_ms\":"<<samples.back()<<"}\n";
}
