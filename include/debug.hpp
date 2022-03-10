

#ifndef ZHONGQIANGREN_BASIC_DEBUG_H_
#define ZHONGQIANGREN_BASIC_DEBUG_H_

#include <ctime>
#include <iostream>

namespace rzq{
namespace basic{

class SimpleTimer{
public:
  SimpleTimer() {};
  virtual ~SimpleTimer() {};
  virtual int Start() {
    this->start_ = std::clock();
    return 0; // normal
  };
  virtual double GetDurationSecond() {
    auto duration = ( std::clock() - this->start_ ) / (double) CLOCKS_PER_SEC;
    return duration;
  };
  virtual void PrintDuration() {
    std::cout<<" System Clock Duration: " << GetDurationSecond() << std::endl;
  }
protected:
  std::clock_t start_ ;
};



}
}


#endif  // ZHONGQIANGREN_BASIC_DEBUG_H_


