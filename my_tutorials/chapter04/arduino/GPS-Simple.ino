/*
 * Sample publisher code for LS20031 GPS module
 *   LS20031         Arduino
 *   3.3V(1) <-----> 3.3V
 *     TX(2) <-----> Pin-3
 *     RX(3) <-----> Pin-2
 *    GND(4) <-----> GND
 *    GND(5)
 */
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <ros.h>
#include <sensor_msgs/NavSatFix.h>

#define RXPin 2
#define TXPin 3

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

ros::NodeHandle nh;
sensor_msgs::NavSatFix gps_msg;
ros::Publisher pub_gps("/gps_publisher", &gps_msg);

void setup()
{
  Serial.begin(57600);
  ss.begin(9600);

  pinMode(TXPin, OUTPUT);
  pinMode(RXPin, INPUT);

  nh.initNode();
  nh.advertise(pub_gps);
  
  gps_msg.header.frame_id="/my_gps";
  gps_msg.latitude  = 0.0;
  gps_msg.longitude = 0.0;
  gps_msg.altitude  = 0.0;
}

void loop()
{
  // Dispatch incoming characters
  while(ss.available() > 0) {
    gps.encode(ss.read());
  }
  
  if (gps.location.isUpdated() || gps.altitude.isUpdated()) {
    gps_msg.header.stamp = nh.now();
    gps_msg.latitude  = gps.location.lat();
    gps_msg.longitude = gps.location.lng();
    gps_msg.altitude  = gps.altitude.meters();

    pub_gps.publish(&gps_msg);
  }

  delay(100);
  nh.spinOnce();
}

