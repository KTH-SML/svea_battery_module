#include <Arduino.h>
#include "DFRobot_INA219.h"
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32.h>

const int updateFrequency = 50;  // 1 Hz
// Create INA219 instances
DFRobot_INA219_IIC ina219_battery(&Wire, INA219_I2C_ADDRESS4);
DFRobot_INA219_IIC ina219_charger(&Wire, INA219_I2C_ADDRESS3);

ros::NodeHandle nh;

// Battery publishers
std_msgs::Float32 bat_cur_msg;
ros::Publisher bat_cur_pub("/battery/current", &bat_cur_msg);

std_msgs::Float32 bat_vol_msg;
ros::Publisher bat_vol_pub("/battery/voltage", &bat_vol_msg);

std_msgs::Float32 bat_shunt_msg;
ros::Publisher bat_shunt_pub("/battery/shunt_voltage", &bat_shunt_msg);

// Charger publishers
std_msgs::Float32 chg_cur_msg;
ros::Publisher chg_cur_pub("/charger/current", &chg_cur_msg);

std_msgs::Float32 chg_vol_msg;
ros::Publisher chg_vol_pub("/charger/voltage", &chg_vol_msg);

std_msgs::Float32 chg_shunt_msg;
ros::Publisher chg_shunt_pub("/charger/shunt_voltage", &chg_shunt_msg);

void setup_nh() {
  nh.getHardware()->setBaud(115200);  // Set to 115200 baud rate
  nh.initNode();
  nh.advertise(bat_cur_pub);
  nh.advertise(bat_vol_pub);
  nh.advertise(bat_shunt_pub);
  nh.advertise(chg_cur_pub);
  nh.advertise(chg_vol_pub);
  nh.advertise(chg_shunt_pub);
}

void setup_ina219() {
  // Initialize INA219 instances
  while (!ina219_battery.begin()) {
    delay(1000);
  }

  while (!ina219_charger.begin()) {
    delay(1000);
  }

  // Set INA219 configurations
  ina219_battery.setPGA(ina219_battery.eIna219PGABits_8);
  ina219_battery.setBADC(ina219_battery.eIna219AdcBits_12, ina219_battery.eIna219AdcSample_32);
  ina219_battery.setSADC(ina219_battery.eIna219AdcBits_12, ina219_battery.eIna219AdcSample_128);

  ina219_charger.setPGA(ina219_charger.eIna219PGABits_8);
  ina219_charger.setBADC(ina219_charger.eIna219AdcBits_12, ina219_charger.eIna219AdcSample_32);
  ina219_charger.setSADC(ina219_charger.eIna219AdcBits_12, ina219_charger.eIna219AdcSample_128);
}

void setup() {
  setup_nh();
  setup_ina219();
}

void update() {
  // Read and update battery data
  bat_cur_msg.data = ina219_battery.getCurrent_mA() / 1000.0;  // Convert mA to A
  bat_vol_msg.data = ina219_battery.getBusVoltage_V();         // Voltage in V
  bat_shunt_msg.data = ina219_battery.getShuntVoltage_mV() / 1000.0;  // Convert mV to V

  // Read and update charger data
  chg_cur_msg.data = ina219_charger.getCurrent_mA() / 1000.0;  // Convert mA to A
  chg_vol_msg.data = ina219_charger.getBusVoltage_V();         // Voltage in V
  chg_shunt_msg.data = ina219_charger.getShuntVoltage_mV() / 1000.0;  // Convert mV to V
}

void publish() {
  // Publish battery data
  bat_cur_pub.publish(&bat_cur_msg);
  bat_vol_pub.publish(&bat_vol_msg);
  bat_shunt_pub.publish(&bat_shunt_msg);

  // Publish charger data
  chg_cur_pub.publish(&chg_cur_msg);
  chg_vol_pub.publish(&chg_vol_msg);
  chg_shunt_pub.publish(&chg_shunt_msg);
}

void loop() {
  if (nh.connected()) {  // Ensure NodeHandle is fully initialized
    update();
    publish();
  } else {
    setup_nh();
    setup_ina219();
  }
  nh.spinOnce();
  delay(1000/updateFrequency);  // Convert Hz to ms for delay
}