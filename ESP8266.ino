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

const unsigned int OUT_PORT=8888;
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

    Serial.begin(115200);
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

//void lukiloop(){
//
//    char Packet[250];
//
//
//    imu::Quaternion quat = bno.getQuat();
//
//
//    char buf_qX[10];
//    float qX = quat.x();
//    dtostrf(qX, 6, 4, buf_qX);
//    //Serial.print(qX,2);
//    //Serial.print(" qX: ");
//    //Serial.print(quat.y(), 4);
//
//    char buf_qY[10];
//    float qY = quat.y();
//    dtostrf(qY, 6, 4, buf_qY);
//    //Serial.print(qY,2);
//    //Serial.print(" qY: ");
//    //Serial.print(quat.x(), 4);
//
//    char buf_qZ[10];
//    float qZ = quat.z();
//    dtostrf(qZ, 6, 4, buf_qZ);
//    //Serial.print(qZ,2);
//    //Serial.print(" qZ: ");
//    //Serial.print(quat.z(), 4);
//    //Serial.println("    ");
//
//    char buf_qW[10];
//    float qW = quat.w();
//    dtostrf(qW, 6, 4, buf_qW);
//    //Serial.print(qW,2);
//    //Serial.print(" qW: ");
//    //Serial.print(quat.w(), 4);
//
//
//    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//
//    char buf_gX[10];
//    float gX = gyro.x();
//    dtostrf(gX, 6, 4, buf_gX);
//    //Serial.print(gX,2);
//
//    char buf_gY[10];
//    float gY = gyro.y();
//    dtostrf(gY, 6, 4, buf_gY);
//    //Serial.print(gY,2);
//
//    char buf_gZ[10];
//    float gZ = gyro.z();
//    dtostrf(gZ, 6, 4, buf_gZ);
//    //Serial.print(gZ,2);
//
//
//   imu::Vector<3> lac = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//
//    char buf_lacX[10];
//    float lacX = lac.x();
//    dtostrf(lacX, 6, 4, buf_lacX);
//    Serial.print(lacX,2);
//
//    char buf_lacY[10];
//    float lacY = lac.y();
//    dtostrf(lacY, 6, 4, buf_lacY);
//    Serial.print(lacY,2);
//
//    char buf_lacZ[10];
//    float lacZ = lac.z();
//    dtostrf(lacZ, 6, 4, buf_lacZ);
//    Serial.print(lacZ,2);
//
//    imu::Vector<3> vgra = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
//
//    char buf_vgraX[10];
//    float vgraX = vgra.x();
//    dtostrf(vgraX, 6, 4, buf_vgraX);
//    Serial.print(vgra,2);
//
//    char buf_vgraY[10];
//    float vgraY = vgra.y();
//    dtostrf(vgraY, 6, 4, buf_vgraY);
//    Serial.print(vgraY,2);
//
//    char buf_vgraZ[10];
//    float vgraZ = vgra.z();
//    dtostrf(vgraZ, 6, 4, buf_vgraZ);
//    Serial.print(vgraZ,2);
//
//    /* Display calibration status for each sensor. */
//  uint8_t system, gyro, accel, mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//  uint8_t now_cal = (system << 6) | (gyro << 4) | (accel << 2) | mag;
//  // Show calibration statuses only when they change
//  if (now_cal != last_cal) {
//    Serial.print(F("CAL: Sys="));
//    Serial.print(system, DEC);
//    Serial.print(F(" G="));
//    Serial.print(gyro, DEC);
//    Serial.print(F(" A="));
//    Serial.print(accel, DEC);
//    Serial.print(F(" M="));
//    Serial.println(mag, DEC);
//    last_cal = now_cal;
//  }
//
//
////   sprintf(Packet, "  qX: %s qY: %s qZ: %s qW: %s lacX: %s lacY: %s lacZ: %s  vgraX: %s vgraY: %s vgraZ: %s gX: %s gY: %s gZ: %s t: %s p: %s a: %s \r \n", buf_qX, buf_qY, buf_qZ, buf_qW, buf_lacX, buf_lacY, buf_lacZ, buf_vgraX, buf_vgraY, buf_vgraZ, buf_gX, buf_gY, buf_gZ, buf_t, buf_p, buf_a );
//
//   sprintf(Packet, "  %s  %s  %s %s %s  %s  %s   %s  %s  %s  %s  %s  %s  %s  %s  %s \r \n", buf_qX, buf_qY, buf_qZ, buf_qW, buf_lacX, buf_lacY, buf_lacZ, buf_vgraX, buf_vgraY, buf_vgraZ, buf_gX, buf_gY, buf_gZ);
//
//
//    // Send a message, to the IP address and port of the specific device
//    Udp.beginPacket(OUT_IP,OUT_PORT);
//    Udp.write(Packet);
//    Udp.endPacket();
//
//
//    // Wait 30 msec
//    delay(30);
//}


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
    uint8_t now_cal = (system << 6) | (gyro << 4) | (accel << 2) | mag;
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

    if (system > 0) {
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