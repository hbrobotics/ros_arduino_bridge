/* Provide a unified interface to IMU devices */

#ifndef IMU_H
#define IMU_H

typedef struct imuData_s
    {
      float ax = -999;
      float ay = -999;
      float az = -999;
      float gx = -999;
      float gy = -999;
      float gz = -999;
      float mx = -999;
      float my = -999;
      float mz = -999;
      float roll = -999;
      float pitch = -999;
      float uh = -999;
    } imuData;

    void initIMU();
    imuData readIMU();
    imuData imu_data;

#endif
