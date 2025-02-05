#include <currentSensors.h> // Your sensor configuration header

// Global timing variables
unsigned long lastCallTime_Cur = 0;
unsigned long lastCallTime_setup = 0;
bool prev_battery_connected = false;
bool prev_charger_connected = false;

#define bat_address 0x41
#define charge_address 0x40

ros::NodeHandle nh;

// Create INA260 sensor instances
Adafruit_INA260 ina260_battery = Adafruit_INA260();
Adafruit_INA260 ina260_charger = Adafruit_INA260();

// Create custom messages (using the energy_sensor message) and publishers
svea_msgs::energy_sensor battery_msg;
ros::Publisher battery_pub("/battery/data", &battery_msg);

svea_msgs::energy_sensor charger_msg;
ros::Publisher charger_pub("/charger/data", &charger_msg);

void advertise() {
    nh.advertise(battery_pub);
    nh.advertise(charger_pub);
}

void setup_nh() {
    nh.getHardware()->setBaud(250000);
    nh.initNode();
    delay(2000);
    advertise();
}

bool setup_ina219() {
    // Initialize INA260 sensors
    if (!ina260_battery.begin(bat_address, &Wire)) {
#ifdef DEBUG
        MYSERIAL.println("Battery INA219 not found");
#endif
        return false;
    }
    if (!ina260_charger.begin(charge_address, &Wire)) {
#ifdef DEBUG
        MYSERIAL.println("Charger INA219 not found");
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
            bool setup_success = setup_ina219();
            prev_battery_connected = battery_connected;
            prev_charger_connected = charger_connected;
#ifdef DEBUG
            MYSERIAL.println("Battery INA219 " + String(battery_connected));
            MYSERIAL.println("Charger INA219 " + String(charger_connected));
#endif
            return setup_success;
        }
        prev_battery_connected = battery_connected;
        prev_charger_connected = charger_connected;
        return true;
    }
#ifdef DEBUG
    MYSERIAL.println("Battery INA219 " + String(battery_connected));
    MYSERIAL.println("Charger INA219 " + String(charger_connected));
#endif
    prev_battery_connected = battery_connected;
    prev_charger_connected = charger_connected;
    return false;
}

void update() {
    // Update battery values (converting as needed)
    battery_msg.current = ina260_battery.readCurrent() / 1000.0;
    battery_msg.voltage = ina260_battery.readBusVoltage() / 1000.0;
    battery_msg.power = ina260_battery.readPower() / 1000.0;

    // Update charger values
    charger_msg.current = ina260_charger.readCurrent() / 1000.0;
    charger_msg.voltage = ina260_charger.readBusVoltage() / 1000.0;
    charger_msg.power = ina260_charger.readPower() / 1000.0;

#ifdef DEBUG
    MYSERIAL.print("Battery: I=" + String(battery_msg.current) + "A V=" +
                   String(battery_msg.voltage) + "V P=" +
                   String(battery_msg.power) + "W | ");
    MYSERIAL.println("Charger: I=" + String(charger_msg.current) + "A V=" +
                     String(charger_msg.voltage) + "V P=" +
                     String(charger_msg.power) + "W");
#endif
}

void publish() {
    battery_pub.publish(&battery_msg);
    charger_pub.publish(&charger_msg);
}

void sensorSpin() {
    nh.spinOnce();

    // Run sensor setup check at defined loop delay
    if (millis() - lastCallTime_setup < I2C_LOOP_DELAY)
        return;
    lastCallTime_setup = millis();
    if (!setupSensorsSpin())
        return;

    // Update sensor readings at defined delay
    if (millis() - lastCallTime_Cur < CUR_SENSOR_SPIN_DELAY)
        return;
    lastCallTime_Cur = millis();

    update();
    publish();
}
