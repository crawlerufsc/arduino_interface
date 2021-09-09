//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//----------------------- LAPIX Crawler MCU Code ------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------


//Include libraries
//-----------------------------------------------------------------------------------------

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <AS5600.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Simple_MPU6050.h>

//ROS definitions
//-----------------------------------------------------------------------------------------

ros::NodeHandle  nh;

sensor_msgs::Imu imu_msg;
sensor_msgs::JointState state;
sensor_msgs::NavSatFix sat_msg;
nav_msgs::Odometry odom_msg;

int m_vel;  // motor velocity between 0 and 255
int s_pos;  // steering servo angle in degrees

void messageCb(const sensor_msgs::JointState& msg) {
  m_vel = state.velocity;
  s_pos = state.position;
}

ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher odom_pub("odom", &odom_msg);
ros::Publisher sat_pub("gps", &sat_msg);
ros::Subscriber<sensor_msgs::JointState> state_sub("motor_state", &messageCb);

long publisher_timer;


//Servo class definition
//-----------------------------------------------------------------------------------------

class _servo
{

    Servo servo; // the servo
    int pos; // current servo position
    int init_pos; // servo initial position
    int taget; // servo target position
    int increment; // increment to move for each interval
    int updateInterval; // interval between updates
    unsigned long lastUpdate; // last update of position

  public:

    _servo(int interval)
    {
      updateInterval = interval;
      increment = 1;
    }

    void Attach(int pin, int init_pos)
    {
      servo.attach(pin);
      servo.write(init_pos);
    }

    void Detach()
    {
      servo.detach();
    }

    void Update(int target)
    {
      if ((millis() - lastUpdate) > updateInterval) // time to update
      {
        lastUpdate = millis();
        servo.write(target);
        pos = target;
      }
    }

};


//Actuators definition
//-----------------------------------------------------------------------------------------

_servo servo_front(100);
_servo servo_back(100);
_servo servo_pitch(100);
_servo servo_roll(100);

int init_vel = 255;

int distance = 0.4; //  distance between rear and front wheels in meters
int pitch;
int roll;

#define motor_front 31
#define motor_back 53


//AS5600 encoder definition
//-----------------------------------------------------------------------------------------

AS5600 encoder;

long revolutions = 0;   // number of revolutions the encoder has made
double position = 0;    // the calculated value the encoder is at
double lastPosition;
double output;          // raw value from AS5600
long lastOutput;        // last output from AS5600

unsigned long encoder_previous = 0;
unsigned long encoder_now;

float encoder_vel;
double encoder_pos;


//MPU6050 IMU definition
//-----------------------------------------------------------------------------------------

#define MPU6050_DEFAULT_ADDRESS     0x68
#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt

Simple_MPU6050 mpu;
ENABLE_MPU_OVERFLOW_PROTECTION();

Quaternion q;
VectorFloat gravity;

float ypr[3] = { 0, 0, 0 };
float xyz[3] = { 0, 0, 0 };
int16_t *acc;
int16_t *gyr;
float qua[4];

int servo_p;
int servo_y;
int servo_r;

int ChartAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {

  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);

    acc = accel;
    gyr = gyro;
    qua[0] = q.x;
    qua[1] = q.y;
    qua[2] = q.z;
    qua[3] = q.w;
  }
}

void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {

  uint8_t Spam_Delay = 100; // Built in Blink without delay timer preventing Serial.print SPAM
  ChartAllValues(gyro, accel, quat, Spam_Delay);

}

//GPS NEO-6M definition
//-----------------------------------------------------------------------------------------

int rx = 10, tx = 11;

double lt; 
double ln;
double alt;
double spd;

TinyGPSPlus gps;  // The TinyGPS++ object
SoftwareSerial Serial_gps(rx, tx);  // The serial connection to the GPS device

//Setup
//-----------------------------------------------------------------------------------------

void setup()
{

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(sat_pub);
  nh.subscribe(state_sub);

  uint8_t val;

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize serial communication
  Serial.begin(115200);

  Serial_gps.begin(9600);

#ifdef OFFSETS
  Serial.println(F("Using Offsets"));
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).load_DMP_Image(OFFSETS); // Does it all for you
#else
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
#endif

  mpu.on_FIFO(print_Values);

  servo_front.Attach(3, 45);
  servo_back.Attach(49, 45);
  servo_pitch.Attach(33, 0);
  servo_roll.Attach(35, 0);
  
  //  Configures the state of the previously declared pins
  pinMode(motor_front, OUTPUT); 
  pinMode(motor_back, OUTPUT);
  
  //  Setup the initial velocity of the motor pins
  
  analogWrite(motor_front, init_vel);
  analogWrite(motor_back, init_vel);

}


//Loop
//-----------------------------------------------------------------------------------------
void loop()
{

  _imu();
  _encoder();
  _gps();
  update_actuators();
  publish();
  debug_info();


}


//Encoder function
//-----------------------------------------------------------------------------------------

void _encoder()
{

  output = encoder.getPosition();           // get the raw value of the encoder

  if ((lastOutput - output) > 2047 )        // check if a full rotation has been made
    revolutions++;
  if ((lastOutput - output) < -2047 )
    revolutions--;

  position = revolutions * 4096 + output;   // calculate the position the the encoder is at based off of the number of revolutions
  lastOutput = output;   // save the last raw value for the next loop

  encoder_now = millis();
  if (encoder_now - encoder_previous >= 200) {

    encoder_vel = ((position - lastPosition) / (encoder_now - encoder_previous)) * 14, 6555936; //angular velocity in rpm
    lastPosition = position;
    encoder_previous = encoder_now;
  }

  encoder_pos = position * 360 / 4094; //conversion from ecoder ticks to degrees;

  odom_msg.twist.twist.linear.x  = (encoder_vel) * 0.09 / 60; //conversion from rpm to m/s
  odom_msg.twist.twist.linear.y  = 0;
  odom_msg.twist.twist.linear.z  = 0;
  odom_msg.twist.twist.angular.z = (encoder_vel) / (distance) * tan((s_pos) * M_PI / 180); //relation between angular velocity and steering angle in ackermann steering

}

//IMU function
//-----------------------------------------------------------------------------------------

void _imu()
{

  mpu.dmp_read_fifo();// Must be in loop

  servo_y = map(xyz[0], -90, 90, 0, 180);
  servo_p = map(xyz[1], -90, 90, 0, 180);
  servo_r = map(xyz[2], -90, 90, 180, 0);

  imu_msg.orientation.x = qua[0];
  imu_msg.orientation.y = qua[1];
  imu_msg.orientation.z = qua[2];
  imu_msg.orientation.w = qua[3];

  imu_msg.angular_velocity.x = ((gyr[0]) * (M_PI / 180) / 25);
  imu_msg.angular_velocity.y = ((gyr[1]) * (M_PI / 180) / 25);
  imu_msg.angular_velocity.z = ((gyr[2]) * (M_PI / 180) / 25);

  imu_msg.linear_acceleration.x = ((acc[0]) * (9.81) / 16384);
  imu_msg.linear_acceleration.y = ((acc[1]) * (9.81) / 16384);
  imu_msg.linear_acceleration.z = ((acc[2]) * (9.81) / 16384);


}

//Actuators function
//-----------------------------------------------------------------------------------------

void update_actuators()
{
  
  servo_front.Update(45 - s_pos);
  servo_back.Update(45 - s_pos);
  servo_pitch.Update(servo_p);
  servo_roll.Update(servo_r);
  servo_roll.Update(servo_r);

  digitalWrite(motor_front, m_vel);
  digitalWrite(motor_back, m_vel);
  
  
}

//GPS function
//-----------------------------------------------------------------------------------------

void _gps()
{


  while (Serial_gps.available() > 0) // This function displays information every time a new sentence is correctly encoded.
    if (gps.encode(Serial_gps.read())) {
      if (gps.location.isValid()) {

        sat_msg.longitude = gps.location.lng();
        sat_msg.latitude  = gps.location.lat();

      }
      if (gps.altitude.isValid()) {

        sat_msg.altitude = gps.altitude.meters();

      }
      if (gps.speed.isValid()) {

        spd = gps.speed.mps();

      }
    }

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    //Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}


//Publisher function
//-----------------------------------------------------------------------------------------

void publish()
{

  if (millis() > publisher_timer) {
    
    imu_pub.publish(&imu_msg);
    odom_pub.publish(&odom_msg);
    sat_pub.publish(&sat_msg);
    
    publisher_timer = millis() + 100; //publish ten times a second
    nh.spinOnce();
  }

}


//Debug function
//-----------------------------------------------------------------------------------------

void debug_info()
{

  //Serial.printfloatx("y ", ypr[0], 0, 4, F(" , "));
  //Serial.printfloatx("p ", ypr[1], 0, 4, F(" , "));
  //Serial.printfloatx("r ", ypr[2], 0, 4, F(" , "));
  //Serial.printfloatx("ax ", acc[0], 0, 0, F(" , "));
  //Serial.printfloatx("ay ", acc[1], 0, 0, F(" , "));
  //Serial.printfloatx("az ", acc[2], 0, 0, F(" , "));
  //Serial.printfloatx("gx ", gyr[0], 0, 0, F(" , "));
  //Serial.printfloatx("gy ", gyr[1], 0, 0, F(" , "));
  //Serial.printfloatx("gz ", gyr[2], 0, 0, F(" , "));
  //Serial.printfloatx("e_pos ", encoder_pos, 0, 0, F(" , "));
  //Serial.printfloatx("e_vel ", encoder_vel, 0, 0, F(" , "));
  //Serial.printfloatx("s_y ", servo_y, 0, 0, F(" , "));
  //Serial.printfloatx("s_p ", servo_p, 0, 0, F(" , "));
  //Serial.printfloatx("s_r ", servo_r, 0, 0, F(" , "));
  //Serial.printfloatx("s_pos ", 45-pos , 0, 0, F(" , "));
  //Serial.printfloatx("vel ", vel, 5, 0, F(" , "));
  //Serial.println();

}
