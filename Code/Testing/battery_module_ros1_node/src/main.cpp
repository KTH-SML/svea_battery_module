#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <Wire.h>
#include "DFRobot_INA219.h"

// I2C Addresses for the INA219 sensors
#define INA219_I2C_ADDRESS_BATTERY 0x40 // Replace with your battery INA219 I2C address
#define INA219_I2C_ADDRESS_CHARGER 0x41 // Replace with your charger INA219 I2C address

// Create INA219 instances
DFRobot_INA219_IIC ina219_battery(&Wire, INA219_I2C_ADDRESS_BATTERY);
DFRobot_INA219_IIC ina219_charger(&Wire, INA219_I2C_ADDRESS_CHARGER);

// Initialize ROS node handle
ros::NodeHandle nh;

// Define ROS publishers for Battery
std_msgs::Float32 battery_shunt_voltage_msg;
std_msgs::Float32 battery_current_msg;
std_msgs::Float32 battery_bus_voltage_msg;

// Define ROS publishers for Charger
std_msgs::Float32 charger_shunt_voltage_msg;
std_msgs::Float32 charger_current_msg;
std_msgs::Float32 charger_bus_voltage_msg;

// Create ROS publisher objects
ros::Publisher pub_battery_shunt_voltage("battery/shunt_voltage", &battery_shunt_voltage_msg);
ros::Publisher pub_battery_current("battery/current", &battery_current_msg);
ros::Publisher pub_battery_bus_voltage("battery/bus_voltage", &battery_bus_voltage_msg);

ros::Publisher pub_charger_shunt_voltage("charger/shunt_voltage", &charger_shunt_voltage_msg);
ros::Publisher pub_charger_current("charger/current", &charger_current_msg);
ros::Publisher pub_charger_bus_voltage("charger/bus_voltage", &charger_bus_voltage_msg);

// Array of publishers for easy management
ros::Publisher* publishers[] = {
  &pub_battery_shunt_voltage,
  &pub_battery_current,
  &pub_battery_bus_voltage,
  &pub_charger_shunt_voltage,
  &pub_charger_current,
  &pub_charger_bus_voltage
};

const uint8_t NUM_PUBLISHERS = sizeof(publishers) / sizeof(ros::Publisher*);

void setup() {
  // Initialize serial communication for ROS
  nh.initNode();
  
  // Advertise all publishers
  for (uint8_t i = 0; i < NUM_PUBLISHERS; i++) {
    nh.advertise(*publishers[i]);
  }

  // Initialize I2C communication
  Wire.begin();

  // Initialize the battery INA219 sensor
  while (!ina219_battery.begin()) {
    // If initialization fails, print error and halt
      Serial.println("Error: INA219 Battery sensor initialization failed.");
      delay(1000);
  }

  // Set INA219 configuration for Battery (No calibration)
  ina219_battery.setPGA(ina219_battery.eIna219PGABits_8);
  ina219_battery.setBADC(ina219_battery.eIna219AdcBits_12, ina219_battery.eIna219AdcSample_32);
  ina219_battery.setSADC(ina219_battery.eIna219AdcBits_12, ina219_battery.eIna219AdcSample_128);

  // Initialize the charger INA219 sensor
  if (!ina219_charger.begin()) {
    // If initialization fails, print error and halt
    while (1) {
      Serial.println("Error: INA219 Charger sensor initialization failed.");
      delay(1000);
    }
  }

  // Set INA219 configuration for Charger (No calibration)
  ina219_charger.setPGA(ina219_charger.eIna219PGABits_8);
  ina219_charger.setBADC(ina219_charger.eIna219AdcBits_12, ina219_charger.eIna219AdcSample_32);
  ina219_charger.setSADC(ina219_charger.eIna219AdcBits_12, ina219_charger.eIna219AdcSample_128);
}

void loop() {
  // Handle ROS communication events
  nh.spinOnce();

  // Read and publish Battery sensor data
  float battery_shunt_voltage = ina219_battery.getShuntVoltage_mV() / 1000.0; // Convert mV to V
  float battery_current = ina219_battery.getCurrent_mA() / 1000.0; // Convert mA to A
  float battery_bus_voltage = ina219_battery.getBusVoltage_V();

  battery_shunt_voltage_msg.data = battery_shunt_voltage;
  battery_current_msg.data = battery_current;
  battery_bus_voltage_msg.data = battery_bus_voltage;

  pub_battery_shunt_voltage.publish(&battery_shunt_voltage_msg);
  pub_battery_current.publish(&battery_current_msg);
  pub_battery_bus_voltage.publish(&battery_bus_voltage_msg);

  // Read and publish Charger sensor data
  float charger_shunt_voltage = ina219_charger.getShuntVoltage_mV() / 1000.0; // Convert mV to V
  float charger_current = ina219_charger.getCurrent_mA() / 1000.0; // Convert mA to A
  float charger_bus_voltage = ina219_charger.getBusVoltage_V();

  charger_shunt_voltage_msg.data = charger_shunt_voltage;
  charger_current_msg.data = charger_current;
  charger_bus_voltage_msg.data = charger_bus_voltage;

  pub_charger_shunt_voltage.publish(&charger_shunt_voltage_msg);
  pub_charger_current.publish(&charger_current_msg);
  pub_charger_bus_voltage.publish(&charger_bus_voltage_msg);

  // Small delay to prevent flooding the ROS network
  delay(50);
}