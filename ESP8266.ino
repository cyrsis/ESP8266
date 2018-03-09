#include <Adafruit_BNO055.h>

#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>
#include <elapsedMillis.h>
#include <SLIPEncodedSerial.h>
#include <SLIPEncodedUSBSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <EEPROM.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP8266WiFi.h>          //ESP8266 Core WiFi Library (you most likely already have this in your sketch)
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic

// how often to report battery level to adafruit IO (in minutes)
#define BATTERY_INTERVAL 1
#define SLEEP_LENGTH 2

Adafruit_BNO055 bno = Adafruit_BNO055();

char DEVICE_NAME[32] = "VICTOR";

#define BNO055_SAMPLERATE_DELAY_MS (30)
// Connection to AP
const char ssid[] = "SOMICA";     // Network SSID (name)
const char pass[] = "Baudolino2012";       // Network password
IPAddress OUT_IP(192,168,21,101);

const unsigned int OUT_PORT=8889;
elapsedMillis imuElapsed;
bool Connected = false;
// Packet for send data

// An EthernetUDP instance to let us send packets over UDP
WiFiUDP Udp;

void setup()
{
    // get the current count position from eeprom
    EEPROM.begin(512);
    byte battery_count = EEPROM.read(0);



    // we only need this to happen once every X minutes,
    // so we use eeprom to track the count between resets.
    if(battery_count >= ((BATTERY_INTERVAL * 60) / SLEEP_LENGTH)) {
        // reset counter
        battery_count = 0;
        // report battery level to Adafruit IO
        battery_level();
    } else {
        // increment counter
        battery_count++;
    }

    // save the current count
    EEPROM.write(0, battery_count);
    EEPROM.commit();

    Serial.begin(9600);
    Serial.println();

    if(!bno.begin())    /* There was a problem detecting the BNO055 ... check your connections */
    {
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    // Connecting to a WiFi network
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    // Connection success
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("Local IP address: ");
    Serial.println(WiFi.localIP());


    bno.setExtCrystalUse(true);
    /* Display the current temperature */
    delay(1000);
    int8_t temp = bno.getTemp();
    Serial.print(F("Current Temperature: "));
    Serial.print(temp);
    Serial.println(F(" C"));
    Serial.println();


} //End of Setup

void loop()
{
    if (WiFi.status() == WL_CONNECTED) {
        if (!Connected) {
            Serial.print(F("WiFi connected! IP address: "));
            Serial.println(WiFi.localIP());
            Connected = true;
        }
    }
    else {
        if (Connected) {
            Serial.println(F("WiFi not connected!"));
            Connected = false;
        }

        WiFiManager wifiManager;
        wifiManager.autoConnect(DEVICE_NAME);

    }

    if (imuElapsed > BNO055_SAMPLERATE_DELAY_MS) {
        imu_loop();
        imuElapsed = 0;
    }
}//End of the Hardareloop



void imu_loop()
{
    static uint8_t last_cal = 0xC0;

    imu::Vector<3> gyro2 = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
#if DEBUG_IMU
    /* Display the floating point data */
  Serial.print(F("X: "));
  Serial.print(gyro2.x());
  Serial.print(F(" Y: "));
  Serial.print(gyro2.y());
  Serial.print(F(" Z: "));
  Serial.print(gyro2.z());
  Serial.print(F("\t\t"));
#endif


    imu::Vector<3> vgra = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
#if DEBUG_IMU
    /* Display the floating point data */
  Serial.print(F("X: "));
  Serial.print(vgra.x());
  Serial.print(F(" Y: "));
  Serial.print(vgra.y());
  Serial.print(F(" Z: "));
  Serial.print(vgra.z());
  Serial.print(F("\t\t"));
#endif

    imu::Vector<3> lac = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
#if DEBUG_IMU
    /* Display the floating point data */
  Serial.print(F("X: "));
  Serial.print(lac.x());
  Serial.print(F(" Y: "));
  Serial.print(lac.y());
  Serial.print(F(" Z: "));
  Serial.print(lac.z());
  Serial.print(F("\t\t"));
#endif

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
#if DEBUG_IMU
    /* Display the floating point data */
  Serial.print(F("X: "));
  Serial.print(euler.x());
  Serial.print(F(" Y: "));
  Serial.print(euler.y());
  Serial.print(F(" Z: "));
  Serial.print(euler.z());
  Serial.print(F("\t\t"));
#endif

    // Quaternion data
    imu::Quaternion quat = bno.getQuat();
#if DEBUG_IMU
    Serial.print(F("qW: "));
  Serial.print(quat.w(), 4);
  Serial.print(F(" qX: "));
  Serial.print(quat.y(), 4);
  Serial.print(F(" qY: "));
  Serial.print(quat.x(), 4);
  Serial.print(F(" qZ: "));
  Serial.print(quat.z(), 4);
  Serial.print(F("\t\t"));
#endif

    /* Display calibration status for each sensor. */
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    uint8_t now_cal = (system << 10) | (gyro << 20) | (accel << 15) | mag;
    // Show calibration statuses only when they change
    if (now_cal != last_cal) {
        Serial.print(F("CAL: Sys="));
        Serial.print(system, DEC);
        Serial.print(F(" G="));
        Serial.print(gyro, DEC);
        Serial.print(F(" A="));
        Serial.print(accel, DEC);
        Serial.print(F(" M="));
        Serial.println(mag, DEC);
        last_cal = now_cal;
    }

    if (true) {
        // Send OSC message
        char tag[64];
        snprintf(tag, sizeof(tag), "/%s/imu", DEVICE_NAME);
        OSCMessage msg(tag);
//    msg.add((float)euler.x());
//    msg.add((float)euler.y());
//    msg.add((float)euler.z());

        msg.add((float)quat.w());
        msg.add((float)quat.x());
        msg.add((float)quat.y());
        msg.add((float)quat.z());

        msg.add((float)lac.x());
        msg.add((float)lac.y());
        msg.add((float)lac.z());

        msg.add((float)vgra.x());
        msg.add((float)vgra.y());
        msg.add((float)vgra.z());

        msg.add((float)gyro2.x());
        msg.add((float)gyro2.y());
        msg.add((float)gyro2.z());


        Udp.beginPacket(OUT_IP, OUT_PORT);
        msg.send(Udp);
        Udp.endPacket();
        msg.empty();

    }
}
void battery_level() {

    // read the battery level from the ESP8266 analog in pin.
    // analog read level is 10 bit 0-1023 (0V-1V).
    // our 1M & 220K voltage divider takes the max
    // lipo value of 4.2V and drops it to 0.758V max.
    // this means our min analog read value should be 580 (3.14V)
    // and the max analog read value should be 774 (4.2V).
    int level = analogRead(A0);

    // convert battery level to percent
    level = map(level, 580, 774, 0, 100);
    Serial.print("Battery level: "); Serial.print(level); Serial.println("%");
//  // turn on wifi if we aren't connected
//  if(WiFi.status() != WL_CONNECTED)
//    wifi_init();
//
//  // grab the battery feed
//  Adafruit_IO_Feed battery = aio.getFeed("battery");
//
//  // send battery level to AIO
//  battery.send(level);

}
