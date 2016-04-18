cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_msgs)

find_package(catkin REQUIRED COMPONENTS std_msgs message_generation)

add_message_files(FILES
                  AnalogFloat.msg
                  Analog.msg
                  ArduinoConstants.msg
                  Digital.msg
                  SensorState.msg
                 )

add_service_files(FILES
                  AnalogWrite.srv
                  AnalogSensorWrite.srv
                  AnalogFloatSensorWrite.srv
                  AnalogPinMode.srv           
                  AnalogRead.srv
                  AnalogSensorRead.srv
                  AnalogFloatSensorRead.srv
                  DigitalPinMode.srv
                  DigitalRead.srv
                  DigitalSensorRead.srv
                  DigitalSetDirection.srv
                  DigitalSensorPinMode.srv
                  DigitalWrite.srv
                  DigitalSensorWrite.srv
                  Enable.srv
                  Relax.srv
                  AnalogSensorRead.srv
                  ServoAttach.srv
                  ServoDetach.srv
                  ServoRead.srv
                  ServoWrite.srv
                  SetSpeed.srv
                  SetServoSpeed.srv
                  UpdatePID.srv
                 )

generate_messages(   
	DEPENDENCIES  
	std_msgs  
)  

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
