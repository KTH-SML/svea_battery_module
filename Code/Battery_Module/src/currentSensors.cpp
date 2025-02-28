#include <Wire.h>           // Ensure Wire is included
#include <currentSensors.h> // Your sensor configuration header
#include <ros.h>
#include <svea_msgs/energy_sensor_readings.h> // Individual sensor reading message

// Global timing variables
unsigned long lastCallTime_Cur = 0;
unsigned long lastCallTime_setup = 0;
bool prev_battery_connected = false;
bool prev_charger_connected = false;

#define bat_address 0x41
#define charge_address 0x40

ros::NodeHandle nh;

// Create INA260 sensor instances
Adafruit_INA260 ina260_battery;
Adafruit_INA260 ina260_charger;

// Create separate messages and publishers
svea_msgs::energy_sensor_readings battery_reading;
svea_msgs::energy_sensor_readings charger_reading;
ros::Publisher battery_pub("/battery_sensor", &battery_reading);
ros::Publisher charger_pub("/charger_sensor", &charger_reading);

void setup_nh() {
    nh.getHardware()->setBaud(250000);
    nh.initNode();
    delay(2000);
    nh.advertise(battery_pub);
    nh.advertise(charger_pub);
}

bool setup_ina260() {
    // Initialize INA260 sensors
    if (!ina260_battery.begin(bat_address, &Wire)) {
#ifdef DEBUG
        MYSERIAL.println("Battery INA260 not found");
#endif
        return false;
    }
    if (!ina260_charger.begin(charge_address, &Wire)) {
#ifdef DEBUG
        MYSERIAL.println("Charger INA260 not found");
#endif
        return false;
    }
    // Configure sensors
    ina260_battery.setAveragingCount(INA260_COUNT_16);
    ina260_battery.setVoltageConversionTime(INA260_TIME_140_us);
    ina260_battery.setCurrentConversionTime(INA260_TIME_140_us);

    ina260_charger.setAveragingCount(INA260_COUNT_16);
    ina260_charger.setVoltageConversionTime(INA260_TIME_140_us);
    ina260_charger.setCurrentConversionTime(INA260_TIME_140_us);

    return true;
}

bool setupSensorsSpin() {
    // Check if sensors are connected via I2C
    Wire.beginTransmission(bat_address);
    bool battery_connected = (Wire.endTransmission() == 0);
    Wire.beginTransmission(charge_address);
    bool charger_connected = (Wire.endTransmission() == 0);

    if (battery_connected && charger_connected) {
        if (!prev_battery_connected || !prev_charger_connected) {
            bool setup_success = setup_ina260();
            prev_battery_connected = battery_connected;
            prev_charger_connected = charger_connected;
#ifdef DEBUG
            MYSERIAL.println("Battery INA260 connected: " + String(battery_connected));
            MYSERIAL.println("Charger INA260 connected: " + String(charger_connected));
#endif
            return setup_success;
        }
        prev_battery_connected = battery_connected;
        prev_charger_connected = charger_connected;
        return true;
    }
#ifdef DEBUG
    MYSERIAL.println("Battery INA260 connected: " + String(battery_connected));
    MYSERIAL.println("Charger INA260 connected: " + String(charger_connected));
#endif
    prev_battery_connected = battery_connected;
    prev_charger_connected = charger_connected;
    return false;
}

void update() {
    // Update battery values
    battery_reading.current = ina260_battery.readCurrent();
    battery_reading.voltage = ina260_battery.readBusVoltage();
    battery_reading.power = ina260_battery.readPower();
    battery_reading.sensor_id = "battery";

    // Update charger values
    charger_reading.current = ina260_charger.readCurrent();
    charger_reading.voltage = ina260_charger.readBusVoltage();
    charger_reading.power = ina260_charger.readPower();
    charger_reading.sensor_id = "charger";

#ifdef DEBUG
    MYSERIAL.print("Battery: I=" + String(battery_reading.current) + "A V=" +
                   String(battery_reading.voltage) + "V P=" +
                   String(battery_reading.power) + "W | ");
    MYSERIAL.println("Charger: I=" + String(charger_reading.current) + "A V=" +
                     String(charger_reading.voltage) + "V P=" +
                     String(charger_reading.power) + "W");
#endif
}

void publish() {
    battery_pub.publish(&battery_reading);
    charger_pub.publish(&charger_reading);
}

void sensorSpin() {
    nh.spinOnce();

    if (millis() - lastCallTime_setup < I2C_LOOP_DELAY)
        return;
    lastCallTime_setup = millis();
    if (!setupSensorsSpin())
        return;

    if (millis() - lastCallTime_Cur < CUR_SENSOR_SPIN_DELAY)
        return;
    lastCallTime_Cur = millis();

    update();
    publish();
}