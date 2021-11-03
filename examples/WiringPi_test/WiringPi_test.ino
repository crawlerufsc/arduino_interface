#include <Wire.h>
#define SLAVE_ADDRESS 0x08
//byte data_to_echo = 0;

void setup() 
{
  Wire.begin(SLAVE_ADDRESS);
  //Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
}

void loop() { }
/*
void receiveData(int bytecount)
{
  for (int i = 0; i < bytecount; i++) {
    data_to_echo = Wire.read();
  }
}
*/
void sendData()
{
  Wire.write(15);
}
