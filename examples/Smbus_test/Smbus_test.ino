#include <Wire.h>

#define Slave_address 11 //

byte dataArray[3];
void setup()
{
  Wire.begin(Slave_address);
  Serial.begin(115200); 
  Wire.onReceive(receiveEvent);
}

void loop(){
  Serial.println(dataArray[2]);
  }

void receiveEvent(int howmany) //howmany = Wire.write()executed by Master
{
  for(int i=0; i<howmany; i++)
  {
    dataArray[i] = Wire.read();
  }
}
