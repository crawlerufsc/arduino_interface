#include <Simple_MPU6050.h>
#include <I2Cdev.h>
#include <Wire.h>

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
int32_t *qua;

int ChartAllValues(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, xyz);

    acc = accel;
    gyr = gyro;
    qua = quat;
    }
}

void print_Values (int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp) {
  
  uint8_t Spam_Delay = 100; // Built in Blink without delay timer preventing Serial.print SPAM
  ChartAllValues(gyro, accel, quat, Spam_Delay);
  
}

void setup() {
  
  uint8_t val;
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize serial communication
  Serial.begin(115200);
  
  #ifdef OFFSETS
    Serial.println(F("Using Offsets"));
    mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).load_DMP_Image(OFFSETS); // Does it all for you
  #else
    mpu.SetAddress(MPU6050_DEFAULT_ADDRESS).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
  #endif
  
  mpu.on_FIFO(print_Values);

}

void loop() {
  
  mpu.dmp_read_fifo();// Must be in loop


  //Serial.printfloatx("", qua[0], 9, 4, F(","));
  //Serial.printfloatx("", qua[1], 9, 4, F(",")); 
  //Serial.printfloatx("", qua[2], 9, 4, F(","));
  //Serial.printfloatx("", qua[3], 9, 4, F(","));   
  //Serial.printfloatx("", ypr[0], 9, 4, F(",")); 
  //Serial.printfloatx("", ypr[1], 9, 4, F(","));
  //Serial.printfloatx("", ypr[2], 9, 4, F(","));
  //Serial.printfloatx("", ((acc[0])*(9.81)/16384), 5, 4, F(","));
  //Serial.printfloatx("", ((acc[1])*(9.81)/16384), 5, 4, F(","));
  //Serial.printfloatx("", ((acc[2])*(9.81)/16384), 5, 4, F(","));
  Serial.printfloatx("", ((gyr[0])*(M_PI)/180), 9, 4, F(","));
  Serial.printfloatx("", ((gyr[1])*(M_PI)/180), 9, 4, F(","));
  Serial.printfloatx("", ((gyr[2])*(M_PI)/180), 9, 4, F(","));
  Serial.println();
  
  
}
