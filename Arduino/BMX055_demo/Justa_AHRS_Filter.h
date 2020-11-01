#ifndef AHRS_Filter_h
#define AHRS_Filter_h

class AHRS
{  
  public:
    AHRS();
    void updateRotationEtimation(float* sensorData, float weigthAcc, float weigthMag, float deltaTime);;
    void updateMagnetCalib(byte* sensorData);
    void convertData(byte* sensorData, float* sensorDataF);
    float quaternion[4];
    
    struct CalibVals{
      int min_x;
      int max_x;
      int min_y;
      int max_y;
      int min_z;
      int max_z;
      CalibVals(){
        min_x=10000;
        max_x=-10000;
        min_y=10000;
        max_y=-10000;
        min_z=10000;
        max_z=-10000;
        };
      };
      
    CalibVals calibVals;
  private:
    float* quaternProd(float* a,float* b);
    void normalizeQ(float* data);
    void normalize(float* data);
    
};

#endif
