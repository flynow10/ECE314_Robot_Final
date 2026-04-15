#ifndef TOF
#define TOF

#include <VL53L0X.h> //Library for ToF sensor

class ToF {
  private:
    VL53L0X Sensor;
    uint8_t xShutPin;
  public:
    ToF(uint8_t xShutPin);
    bool init(uint8_t address);
    uint16_t getRangeMilimeters();
};


#endif