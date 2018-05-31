#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>

int   Trig = 8;
int   Echo = 9;
int   Duration;
float Distance;

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range("/ultrasound", &range_msg);
char frameid[] = "/ultrasound";

void setup() 
{
  Serial.begin(9600);
  pinMode(Trig,OUTPUT);
  pinMode(Echo,INPUT);
  
  nh.initNode();
  nh.advertise(pub_range);

  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.1;  // fake
  range_msg.min_range = 2.0;
  range_msg.max_range = 450.0;
}

void loop() 
{
  digitalWrite(Trig,LOW);
  delayMicroseconds(2);
  digitalWrite(Trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig,LOW);
  Duration = pulseIn(Echo,HIGH);  

  if (Duration > 0) {
    Distance = Duration/2;
    // ultrasonic speed is 340m/s = 34000cm/s = 0.034cm/us 
    Distance = Distance*0.0340; 
      
    range_msg.range = Distance;
    range_msg.header.stamp = nh.now();
    pub_range.publish(&range_msg);
  }
  delay(100);
  
  nh.spinOnce();
}
