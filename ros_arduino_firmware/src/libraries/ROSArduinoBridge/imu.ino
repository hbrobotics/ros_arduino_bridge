#ifdef USE_IMU

  #ifdef ADAFRUIT_9DOF
  
    #include <Wire.h>
    #include <Adafruit_Sensor.h>
    #include <Adafruit_LSM303_U.h>
    #include <Adafruit_L3GD20_U.h>
    #include <Adafruit_9DOF.h>
    
    /* Assign a unique ID to the sensors */
    Adafruit_9DOF                 dof   = Adafruit_9DOF();
    Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
    Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
    Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

    
    /* Update this with the correct SLP for accurate altitude measurements */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    
    void initIMU()
    {
      accel.begin();
      mag.begin();
      gyro.begin();
    }
  
    imuData readIMU() {
      imuData_s data;
    
      sensors_event_t accel_event;
      sensors_event_t mag_event;
      sensors_vec_t   orientation;
      sensors_event_t event;
      
      /* Calculate pitch and roll from the raw accelerometer data */
      accel.getEvent(&accel_event);
      data.ax = accel_event.acceleration.x;
      data.ay = accel_event.acceleration.y;
      data.az = accel_event.acceleration.z;

      gyro.getEvent(&event);
      data.gx = event.gyro.x;
      data.gy = event.gyro.y;
      data.gz = event.gyro.z;
/*
      if (dof.accelGetOrientation(&accel_event, &orientation))
      {
        data.roll = orientation.roll;
        data.pitch = orientation.pitch;
      }
*/

      /* Calculate the heading using the magnetometer */
      mag.getEvent(&mag_event);

/*
      if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
      {
        data.mz = orientation.heading;
      }
*/

      if (dof.fusionGetOrientation(&accel_event, &mag_event, &orientation))
      {
        data.roll = orientation.roll;
        data.pitch = orientation.pitch;
        data.ch = orientation.heading;
      }
      
      return data;
    }
  
  #endif

#endif
