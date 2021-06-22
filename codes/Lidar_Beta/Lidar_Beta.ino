//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//----------------------- Lidar_Beta MCU Code ------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------


//Include libraries
//-----------------------------------------------------------------------------------------

#include <ros.h>
#include <sensor_msgs/LaserScan.h>

#include <Wire.h>
#include <AS5600.h>
#include <LIDARLite.h>

//ROS definitions
//-----------------------------------------------------------------------------------------

ros::NodeHandle  nh;

sensor_msgs::LaserScan laser_msg;

ros::Publisher laser_pub("laser_scan", &laser_msg);

//PWM definition
//-----------------------------------------------------------------------------------------

#define pwm  5    // Arduino PWM pin that is connected to the base of the transistor
int pwm_vel;      //Variable that will store the current PWM value (0 -> 255)

//Lidar definition
//-----------------------------------------------------------------------------------------

LIDARLite myLidarLite;
#define mode 3    // Arduino pin that is connected to the MODE pin of the Lidar

//AS5600 encoder definition
//-----------------------------------------------------------------------------------------

AS5600 encoder;

long revolutions = 0;   // Number of revolutions the encoder has made
double position = 0;    // The calculated value the encoder is at
double lastPosition;
double output;          // Raw value from AS5600
long lastOutput;        // Last output from AS5600

unsigned long encoder_previous = 0;
unsigned long encoder_now;

float vel;

void setup() {

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(laser_pub);

  laser_msg.angle_min = 0;
  laser_msg.angle_max = 2*M_PI;
  laser_msg.range_min = 0.1;
  laser_msg.range_max = 40;
  
  Serial.begin(115200); // Initialize serial communication
  
  myLidarLite.begin();
  myLidarLite.beginContinuous();

  //  Configures the state of the previously declared pins
  pinMode(pwm, OUTPUT);
  pinMode(mode, INPUT);

  //  Setup the initial velocity of the motor pins
  analogWrite(pwm, 0);
  
}

void loop() {
  
  output = encoder.getPosition();           // Get the raw value of the encoder                     
  
  if ((lastOutput - output) > 2047 )        // Check if a full rotation has been made
    revolutions++;
  if ((lastOutput - output) < -2047 )
    revolutions--;
    
  position = revolutions * 4096 + output;   // Calculate the position the the encoder is at based off of the number of revolutions
  lastOutput = output;                      // Save the last raw value for the next loop 

  encoder_now = millis();
  if (encoder_now - encoder_previous >= 200) {

    vel = ((position-lastPosition)/(encoder_now-encoder_previous))*14,6555936;
    lastPosition = position;
    encoder_previous = encoder_now;
  }

  if(!digitalRead(2)){

    laser_msg.ranges[int((output)*360/4094)] = myLidarLite.distanceContinuous();
    
    //Serial.println(laser_msg.ranges[int((output)*360/4094)]);
    //Serial.print("Distance: ");
    //Serial.print(myLidarLite.distanceContinuous());
    //Serial.print(" Position: ");
    //Serial.println(int((output)*360/4094));
    //Serial.print(" Speed: ");
    //Serial.print(vel);
    //Serial.println(" rpm");
    
  }

  
  if((int((output)*360/4094)) == 360){

    laser_pub.publish(&laser_msg);
    
  }
  

  nh.spinOnce();
}
