#include <Adafruit_INA260.h>
#include <Arduino.h>
#include <globals.h>
#include <ros.h>
#include <settings.h>
#include <std_msgs/Float32.h>
#include <svea_msgs/energy_sensor.h>
void setupSensors();
void sensorSpin();
void setup_nh();
