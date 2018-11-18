Changelog for ROS Arduino Bridge since Indigo

* support for Robogaia 3-axis Encoder Shield
* support for Adafruit Motor Shield V2
* support for single-axis gyro with analog output
* support for Adafruit 9-axis IMU
* fixed bug in firmware causing motion hysteresis when robot stopped
* added min, max, scale and offset parameters for analog sensors
* added ROS diagnostics publishers for sensors, joints (servos) and base controller
* added per-component diagnostics parameters diagnostics_error_threshold and diagnostics_rate
* added reset_odometry service
* added dynamic reconfigure support for PID parameters
* added dynamic reconfigure support for odom scale correction parameters
* improved robustness of serial command
* added heartbeat/watchdog check to recover from loss of serial link
* added heartbeat publisher to reflect the state of the connection
* added default covariance values for Odometry message

TODO: reconnect servos after USB disconnect/reconnect
TODO: min/max parameters for sensor readings --> NaN and inf
TODO: diagnostic error counts for service calls
TODO: publish a value that gets set by a service call?
TODO: continuous rotation servos?