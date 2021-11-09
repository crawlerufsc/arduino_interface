//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
//----------------------- LAPIX Crawler MCU Code ------------------------------------------
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------


//Include libraries
//-----------------------------------------------------------------------------------------

#include <Servo.h>
#include <I2Cdev.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x0b

//I2C data definitions
//-----------------------------------------------------------------------------------------

byte dataArray[3]; // array to store the joystick variables
int m_vel;        // motor velocity between 0 and 255
int s_pos;       // steering servo angle in degrees 0 - 90
int last_m_vel    = 0;
int last_s_pos    = 45;
int m_vel_change_lim = 10;
int s_pos_change_lim = 5;

#define printfloatx(Name,Variable,Spaces,Precision,EndTxt) print(Name); {char S[(Spaces + Precision + 3)];Serial.print(F(" ")); Serial.print(dtostrf((float)Variable,Spaces,Precision ,S));}Serial.print(EndTxt);//Name,Variable,Spaces,Precision,EndTxt

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

int init_vel = 0;

int distance = 0.4; //  distance between rear and front wheels in meters
int pitch;
int roll;

#define motor_front 3
#define motor_back 4

//Setup
//-----------------------------------------------------------------------------------------

void setup()
{
  //  initialize dataArray
  dataArray[0] = 0;
  dataArray[1] = 0;
  dataArray[2] = 45;
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveEvent);

  // initialize serial communication
  Serial.begin(115200);

  //  Attaches the servos
  servo_front.Attach(35, 0);
  servo_back.Attach(49, 0);
  servo_pitch.Attach(33, 0);
  servo_roll.Attach(51, 0);

  //  Configures the state of the previously declared pins
  pinMode(motor_front, OUTPUT);
  pinMode(motor_back, OUTPUT);

  digitalWrite(motor_front, LOW);
  digitalWrite(motor_back, LOW);
}


//Loop
//-----------------------------------------------------------------------------------------
void loop()
{

  update_actuators();
  debug_info();
  
}

//I2C callback function
//-----------------------------------------------------------------------------------------


void receiveEvent(int howmany) //howmany = Wire.write()executed by Master
{
  for(int i=0; i<howmany; i++)
  {
    dataArray[i] = Wire.read();
  }
}

//Smooth update function
//-----------------------------------------------------------------------------------------
inline void smooth_update(){
  
  if(m_vel - last_m_vel > m_vel_change_lim){
    m_vel = last_m_vel + m_vel_change_lim;
  }else if(m_vel - last_m_vel < -m_vel_change_lim){
    m_vel = last_m_vel - m_vel_change_lim;
  }
  
  if(s_pos - last_s_pos > s_pos_change_lim){
    s_pos = last_s_pos + s_pos_change_lim;
  }else if(s_pos - last_s_pos < -s_pos_change_lim){
    s_pos = last_s_pos - s_pos_change_lim;
  }
}

//Actuators function
//-----------------------------------------------------------------------------------------

void update_actuators()
{

  m_vel = int(dataArray[1]);
//  s_pos = int(dataArray[2]) - 45;
  s_pos = int(dataArray[2]);
  
  smooth_update();

//  servo_front.Update(45 - s_pos);
//  servo_back.Update(45 + s_pos);
  servo_front.Update(s_pos);
  servo_back.Update(45);

  analogWrite(motor_front, m_vel);
  analogWrite(motor_back, m_vel);  
  
  analogWrite(13, m_vel);
  
  last_m_vel = m_vel;
  last_s_pos = s_pos;
}

//Debug function
//-----------------------------------------------------------------------------------------

void debug_info()
{
  Serial.printfloatx("pos ", s_pos , 0, 0, F(" , "));
  Serial.printfloatx("vel ", m_vel+255, 5, 0, F(" , "));
  Serial.println();
}
