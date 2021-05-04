#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
//String pwd = String(1.20,3);
Servo fmotor,fservo,bmotor,bservo;
void messageCb( const std_msgs::String& msg){
  String srt=msg.data;
  float dis = srt.toFloat();
  //dis = dis*100;
  nh.loginfo(msg.data);
  //Serial.println(msg.data);

  if(dis <= 120)
  {
    //fmotor.write(80);
    bmotor.write(80);
  }
  if(dis <= 180 && dis >= 120)
  {
    //fmotor.write(120);
    bmotor.write(120);
  }
  if(dis >= 220)
  {
    //fmotor.write(180);
    bmotor.write(180);
  }
}

ros::Subscriber<std_msgs::String> sub("dis", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
  fmotor.attach(10,1000,2000);
  fservo.attach(9,1000,2000);
  bmotor.attach(6,1000,2000);
  bservo.attach(5,1000,2000);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
