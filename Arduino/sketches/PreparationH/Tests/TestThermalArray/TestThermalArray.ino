// Interfacing to the TPA81 Thermopile Array(PPTPA81)
// Created 20 May 2011

#include <Wire.h>

// TPA81 Constants
#define TPA81ADDR 0x68    //Address is $D0 but is in high 7 bits so divide by 2
#define REVISION 0x00
#define AMBIENT 0x01
#define PIXEL1TEMP 0x02
#define PIXEL2TEMP 0x03
#define PIXEL3TEMP 0x04
#define PIXEL4TEMP 0x05
#define PIXEL5TEMP 0x06
#define PIXEL6TEMP 0x07
#define PIXEL7TEMP 0x08
#define PIXEL8TEMP 0x09

int Revision, Ambient, Pixel1, Pixel2, Pixel3, Pixel4, Pixel5, Pixel6, Pixel7, Pixel8;
char buffer[39];
char inChar;

void setup()
{
 Wire.begin();  // join i2c bus  
 Serial.begin(115200);

}

void loop()
{
while (inChar = Serial.read() != '*')

Wire.beginTransmission(TPA81ADDR);
Wire.write(REVISION);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Revision = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(AMBIENT);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Ambient = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL1TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel1 = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL2TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel2 = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL3TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel3 = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL4TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel4 = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL5TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel5 = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL6TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel6 = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL7TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel7 = Wire.read();

Wire.beginTransmission(TPA81ADDR);
Wire.write(PIXEL8TEMP);    
Wire.endTransmission();

Wire.requestFrom(TPA81ADDR, 1);
while(!Wire.available());
Pixel8 = Wire.read();

 sprintf(buffer, "%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d,%03d",Revision,Ambient,Pixel1,Pixel2,Pixel3,Pixel4,Pixel5,Pixel6,Pixel7,Pixel8);
 Serial.print(buffer);


}




