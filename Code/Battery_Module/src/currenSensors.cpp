
#include <currentSensors.h>
unsigned long lastCallTime_Cur = 0;
unsigned long lastCallTime_setup = 0;
bool prev_battery_connected = false;
bool prev_charger_connected = false;

#define bat_address 0x41
#define charge_address 0x40
// Create INA219 instances
Adafruit_INA260 ina260_battery = Adafruit_INA260();
Adafruit_INA260 ina260_charger = Adafruit_INA260();

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
    nh.getHardware()->setBaud(115200); // Set to 115200 baud rate
    nh.initNode();
    nh.advertise(bat_cur_pub);
    nh.advertise(bat_vol_pub);
    nh.advertise(bat_shunt_pub);
    nh.advertise(chg_cur_pub);
    nh.advertise(chg_vol_pub);
    nh.advertise(chg_shunt_pub);
}

bool setup_ina219() {
    // Initialize INA219 instances
    if (!ina260_battery.begin(bat_address, &WIRE)) {
        if (DEBUG)
            MYSERIAL.println("Battery INA219 not found");
        return false;
    }

    if (!ina260_charger.begin(charge_address, &WIRE)) {
        if (DEBUG)
            MYSERIAL.println("Charger INA219 not found");
        return false;
    }

    // Set INA219 configurations
    ina260_battery.setAveragingCount(INA260_COUNT_16);
    ina260_battery.setVoltageConversionTime(INA260_TIME_140_us);
    ina260_battery.setCurrentConversionTime(INA260_TIME_140_us);

    ina260_charger.setAveragingCount(INA260_COUNT_16);
    ina260_charger.setVoltageConversionTime(INA260_TIME_140_us);
    ina260_charger.setCurrentConversionTime(INA260_TIME_140_us);
    return true;
}

void update() {
    // Read and update battery data
    bat_cur_msg.data = ina260_battery.readCurrent() / 1000.0;    // Convert mA to A
    bat_vol_msg.data = ina260_battery.readBusVoltage() / 1000.0; // Voltage in V
    bat_shunt_msg.data = ina260_battery.readPower() / 1000.0;    // Convert mV to V

    // Read and update charger data
    chg_cur_msg.data = ina260_charger.readCurrent() / 1000.0;  // Convert mA to A
    chg_vol_msg.data = ina260_charger.readBusVoltage() / 1000; // Voltage in V
    chg_shunt_msg.data = ina260_charger.readPower() / 1000.0;  // Convert mV to V
    if (DEBUG) {
        MYSERIAL.print("Battery: I=" + String(bat_cur_msg.data) + "A V=" + String(bat_vol_msg.data) + "V Vs=" + String(bat_shunt_msg.data) + "V | ");
        MYSERIAL.println("Charger: I=" + String(chg_cur_msg.data) + "A V=" + String(chg_vol_msg.data) + "V Vs=" + String(chg_shunt_msg.data) + "V");
    }
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

// Cheaper way to check if INA219 sensors are connected
bool setupSensorsSpin() {
    // Check if INA219 sensors are connected
    Wire.beginTransmission(bat_address);
    bool battery_connected = (Wire.endTransmission() == 0);
    Wire.beginTransmission(charge_address);
    bool charger_connected = (Wire.endTransmission() == 0);

    // Only run setup if both sensors are connected
    if (battery_connected && charger_connected) {
        if (!prev_battery_connected || !prev_charger_connected) {
            bool setup_success = setup_ina219();

            prev_battery_connected = battery_connected;
            prev_charger_connected = charger_connected;
            if (DEBUG) {
                MYSERIAL.println("Battery INA219 " + String(battery_connected));
                MYSERIAL.println("Charger INA219 " + String(charger_connected));
            }
            return setup_success;
        }
        prev_battery_connected = battery_connected;
        prev_charger_connected = charger_connected;
        return true;
    }

    if (DEBUG) {
        MYSERIAL.println("Battery INA219 " + String(battery_connected));
        MYSERIAL.println("Charger INA219 " + String(charger_connected));
    }

    prev_battery_connected = battery_connected;
    prev_charger_connected = charger_connected;
    return false;
}

void sensorSpin() {

    nh.spinOnce();
    if (millis() - lastCallTime_setup < I2C_LOOP_DELAY) {
        return;
    }

    lastCallTime_setup = millis();
    if (!setupSensorsSpin()) {
        return;
    }

    if (millis() - lastCallTime_Cur < CUR_SENSOR_SPIN_DELAY) {
        return;
    }
    lastCallTime_Cur = millis();

    update();
    publish();
}