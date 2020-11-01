#ifndef BMX055_lib_h
#define BMX055_lib_h

//The minimalistic library for BMX055 setting and data capture
class BMX055
{
  public:
  //initialize BMXSensor in constructor
    BMX055();
    void getSensorData(byte* data);
    void init();
};

#endif
