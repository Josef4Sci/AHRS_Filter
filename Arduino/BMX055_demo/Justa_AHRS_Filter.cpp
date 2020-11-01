#ifndef AHRS_Filter
#define AHRS_Filter

#include "Arduino.h"
#include "Justa_AHRS_Filter.h"

// JUSTA, Josef; ŠMÍDL, Václav; HAMÁČEK, Aleš. Fast AHRS Filter for Accelerometer, Magnetometer, 
// and Gyroscope Combination with Separated Sensor Corrections. Sensors, 2020, 20.14: 3824.

// If this library was useful in your work, don't forget to reference the paper above!

AHRS::AHRS(void){
//  quaternion=new float[4];
  *(quaternion+0)=1.0f;
  *(quaternion+1)=0.0f;
  *(quaternion+2)=0.0f;
  *(quaternion+3)=0.0f;
  calibVals = CalibVals();
}

void AHRS::convertData(byte* sensorData, float* sensorDataF){
  
  // Convert the data to 12-bits
  int xAccl = ((sensorData[1] * 256) + (sensorData[0] & 0xF0)) / 16;
  if (xAccl > 2047)
  {
    xAccl -= 4096;
  }
  int yAccl = ((sensorData[3] * 256) + (sensorData[2] & 0xF0)) / 16;
  if (yAccl > 2047)
  {
    yAccl -= 4096;
  }
  int zAccl = ((sensorData[5] * 256) + (sensorData[4] & 0xF0)) / 16;
  if (zAccl > 2047)
  {
    zAccl -= 4096;
  }

  *(sensorDataF+0)=(float)xAccl * 0.00391f;
  *(sensorDataF+1)=(float)yAccl * 0.00391f;
  *(sensorDataF+2)=(float)zAccl * 0.00391f;
  
    // Convert the data
  int xGyro = (sensorData[7] * 256) + sensorData[6];
  if (xGyro > 32767)
  {
    xGyro -= 65536;
  }
  int yGyro = (sensorData[9] * 256) + sensorData[8];
  if (yGyro > 32767)
  {
    yGyro -= 65536;
  }
  int zGyro = (sensorData[11] * 256) + sensorData[10];
  if (zGyro > 32767)
  {
    zGyro -= 65536;
  }

  *(sensorDataF+3)=(float)xGyro * 0.0305f * 3.14159 / 180.0; // 0.0305 °/s/LSB
  *(sensorDataF+4)=(float)yGyro * 0.0305f * 3.14159 / 180.0; // 0.0305 °/s/LSB
  *(sensorDataF+5)=(float)zGyro * 0.0305f * 3.14159 / 180.0; // 0.0305 °/s/LSB

  // Convert the data - axis aligned with accelerometer and gyroscope by datasheet BMX055 (Fig. 41 - orientation of sensor sensing axis)
  int yMag = ((sensorData[13] * 256) + (sensorData[12] & 0xF8)) / 8;
  if (yMag > 4095)
  {
    yMag -= 8192;
  }
  int xMag = -((sensorData[15] * 256) + (sensorData[14] & 0xF8)) / 8;
  if (xMag > 4095)
  {
    xMag -= 8192;
  }
  int zMag = ((sensorData[17] * 256) + (sensorData[16] & 0xFE)) / 2;
  if (zMag > 16383)
  {
    zMag -= 32768;
  }
  
  //Set calibrated magnetometer output
  *(sensorDataF+6)=((float)(xMag-calibVals.min_x)/(calibVals.max_x-calibVals.min_x))*2-1;
  *(sensorDataF+7)=((float)(yMag-calibVals.min_y)/(calibVals.max_y-calibVals.min_y))*2-1;
  *(sensorDataF+8)=((float)(zMag-calibVals.min_z)/(calibVals.max_z-calibVals.min_z))*2-1;
}

void AHRS::updateMagnetCalib(byte* sensorData){
  // Convert the data - axis aligned with accelerometer and gyroscope by datasheet BMX055 (Fig. 41 - orientation of sensor sensing axis)
  int yMag = ((sensorData[13] * 256) + (sensorData[12] & 0xF8)) / 8;
  if (yMag > 4095)
  {
    yMag -= 8192;
  }
  int xMag = -((sensorData[15] * 256) + (sensorData[14] & 0xF8)) / 8;
  if (xMag > 4095)
  {
    xMag -= 8192;
  }
  int zMag = ((sensorData[17] * 256) + (sensorData[16] & 0xFE)) / 2;
  if (zMag > 16383)
  {
    zMag -= 32768;
  }

  if(xMag<calibVals.min_x)
    calibVals.min_x=xMag;
  if(xMag>calibVals.max_x)
    calibVals.max_x=xMag;
    
  if(yMag<calibVals.min_y)
    calibVals.min_y=yMag;
  if(yMag>calibVals.max_y)
    calibVals.max_y=yMag;
    
  if(zMag<calibVals.min_z)
    calibVals.min_z=zMag;
  if(zMag>calibVals.max_z)
    calibVals.max_z=zMag;
}

void AHRS::updateRotationEtimation(float* sensorData, float weigthAcc, float weigthMag, float deltaTime){

  float qp[4];
  char buffer1[200];      
  float accMesPred[3];  
  float magMesPred[3];
  float ca[3];  
  float cm[3];  
  float qCor[4];
  float g[4];
  
  float* acc=sensorData;
  float* gyr=sensorData+3;
  float* mag=sensorData+6;
  
  normalize(acc);
  normalize(mag);
  
  g[0]= 1;
  g[1]=*(gyr+0);
  g[2]=*(gyr+1);
  g[3]=*(gyr+2);

  float* qDot=quaternProd(quaternion,g);
  for(int i=0;i<4;i++){
    *(qDot+i) *= deltaTime;
  }
  
  for(int i=0;i<4;i++){
    *(qp+i) = *(quaternion+i) + *(qDot+i);
  }

  normalizeQ(qp);

  float R11=2 * (0.5f - *(qp+2)*(*(qp+2)) - *(qp+3)*(*(qp+3)));
  float R21=2 * (*(qp+1)*(*(qp+2)) - *(qp+0)*(*(qp+3)));
  float R31=2 * (*(qp+0)*(*(qp+2)) + *(qp+1)*(*(qp+3)));

  float R13=2 * (*(qp+1)*(*(qp+3)) - *(qp+0)*(*(qp+2)));
  float R23=2 * (*(qp+0)*(*(qp+1)) + *(qp+2)*(*(qp+3)));
  float R33=2 * (0.5 - (*(qp+1))*(*(qp+1)) - *(qp+2)*(*(qp+2)));  
  
  //ar=[0 0 1];
  *(accMesPred+0)=R13;
  *(accMesPred+1)=R23;
  *(accMesPred+2)=R33;

  //mr=[mrx 0 mrz]
  float mr_z= *(accMesPred+0)*(*(mag+0))+*(accMesPred+1)*(*(mag+1))+*(accMesPred+2)*(*(mag+2));
  float mr_x=sqrt(1-mr_z*mr_z);
  
  *(magMesPred+0)=R11*mr_x+R13*mr_z;
  *(magMesPred+1)=R21*mr_x+R23*mr_z;
  *(magMesPred+2)=R31*mr_x+R33*mr_z;  

  *(ca+0)=(*(acc+1))*(*(accMesPred+2))-(*(acc+2))*(*(accMesPred+1));
  *(ca+1)=(*(acc+2))*(*(accMesPred+0))-(*(acc+0))*(*(accMesPred+2));
  *(ca+2)=(*(acc+0))*(*(accMesPred+1))-(*(acc+1))*(*(accMesPred+0));
  normalize(ca);

  *(cm+0)=(*(mag+1))*(*(magMesPred+2))-(*(mag+2))*(*(magMesPred+1));
  *(cm+1)=(*(mag+2))*(*(magMesPred+0))-(*(mag+0))*(*(magMesPred+2));
  *(cm+2)=(*(mag+0))*(*(magMesPred+1))-(*(mag+1))*(*(magMesPred+0));
  normalize(cm);

  *(qCor+0)=1;
  *(qCor+1)=(*(ca+0)*weigthAcc)+(*(cm+0)*weigthMag);
  *(qCor+2)=(*(ca+1)*weigthAcc)+(*(cm+1)*weigthMag);
  *(qCor+3)=(*(ca+2)*weigthAcc)+(*(cm+2)*weigthMag);
  
  float* quat=quaternProd(qp,qCor);
  
  if(*(quat+0)<0){
    *(quat+0)=-*(quat+0);
    *(quat+1)=-*(quat+1);
    *(quat+2)=-*(quat+2);
    *(quat+3)=-*(quat+3);
  }

  normalizeQ(quat);
  
  quaternion[0]=*quat;
  quaternion[1]=*(quat+1);
  quaternion[2]=*(quat+2);
  quaternion[3]=*(quat+3);
}

void AHRS::normalize(float* sensorData){
  float factor=sqrt(
    (*(sensorData+0))*(*(sensorData+0))+
    (*(sensorData+1))*(*(sensorData+1))+
    (*(sensorData+2))*(*(sensorData+2)));

  *(sensorData+0)/=factor;
  *(sensorData+1)/=factor;
  *(sensorData+2)/=factor;
}

void AHRS::normalizeQ(float* sensorData){
  float factor=sqrt(
    (*(sensorData+0))*(*(sensorData+0))+
    (*(sensorData+1))*(*(sensorData+1))+
    (*(sensorData+2))*(*(sensorData+2))+
    (*(sensorData+3))*(*(sensorData+3)));

  *(sensorData+0)/=factor;
  *(sensorData+1)/=factor;
  *(sensorData+2)/=factor;
  *(sensorData+3)/=factor;
}

//first element is real number
float* AHRS::quaternProd(float a[],float b[]){    
    static float ab[4];
    ab[0]= a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    ab[1]= a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    ab[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    ab[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
    return ab;
}

#endif
