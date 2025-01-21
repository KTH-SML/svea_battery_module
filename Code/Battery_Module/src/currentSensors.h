#include <Adafruit_INA260.h>
#include <Arduino.h>
#include <ros.h>
#include <settings.h>
#include <std_msgs/Float32.h>
void setupSensors();
void sensorSpin();
void setup_nh();