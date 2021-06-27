#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <math.h>
#define pi 3.141592

unsigned long currentTime, previousTime;
float kp=1,ki=0,kd=0;
float elapsedTime,error,lastError,cumError,rateError;
float angle,setPoint=25;

std_msgs::Float32 angle_msg;
ros::NodeHandle  nh;
ros::Publisher pub_angle("angle", &angle_msg);
//String pwd = String(1.20,3);
Servo fmotor,fservo,bmotor,bservo;

void messageCb( const std_msgs::String& msg){
  String srt=msg.data;
  float slope = srt.toFloat();
  //dis = dis*100;
  nh.loginfo(msg.data);
  double rad = atan(slope);
  double rad_pid = rad*(360/(2*pi));
  angle = computePID(rad_pid);
  angle_msg.data = angle;
  pub_angle.publish(&angle_msg);
  fservo.write(angle);
  bmotor.write(127);
  
}

ros::Subscriber<std_msgs::String> sub("slope", &messageCb );

void setup()
{ 
  nh.initNode();
  nh.advertise(pub_angle);
  nh.subscribe(sub);
  fmotor.attach(3,1000,2000);
  fservo.attach(5,1000,2000);
  bmotor.attach(6,1000,2000);
  //bservo.attach(5,1000,2000);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

double computePID(double inp){     
        currentTime = millis();                //get current time
        elapsedTime = (double)(currentTime - previousTime);        //compute time elapsed from previous computation
        
        error = setPoint - inp;                                // determine error
        cumError += error * elapsedTime;                // compute integral
        rateError = (error - lastError)/elapsedTime;   // compute derivative
 
        double out = kp*error + ki*cumError + kd*rateError;                //PID output               
 
        lastError = error;                                //remember current error
        previousTime = currentTime;                        //remember current time
 
        return out;                                        //have function return the PID output
}
