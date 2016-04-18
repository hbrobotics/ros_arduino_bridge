/* Provide a unified interface to IMU devices */

#ifndef IMU_H
#define IMU_H

/*
  IMU data is assumed to be returned in the following order:
    
  [ax, ay, az, gx, gy, gz, mx, my, mz, roll, pitch, ch]
    
  where a stands for accelerometer, g for gyroscope and m for magnetometer.
  The last value ch stands for "compensated heading" that some IMU's can 
  compute to compensate magnetic heading for the current roll and pitch.
*/

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
      float ch = -999;
    } imuData;

    void initIMU();
    imuData readIMU();
    imuData imu_data;

#endif
