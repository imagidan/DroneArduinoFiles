#include "mbed.h"

class Multishot{
  
  public:
    Multishot(PinName const _pin);
    ~Multishot();
    void setPercent(float const _percent);
    float getPercent();
    
  private:
    PwmOut *motor;
    float min_us;
    float mid_us;
    float max_us;
    float percent;

    float multishotCalculatePercent(float const _percent);
    
}
