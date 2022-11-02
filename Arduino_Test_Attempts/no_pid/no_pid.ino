#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int16_t gyroX, gyroRate;
float gyroAngle=0;
unsigned long currTime, prevTime=0, loopTime;

// Motor Links (vooraanzicht)
const int A1A = 6;      // orange
const int A1B = 9;      // brown
// Motor Rechts (vooraanzicht)
const int B1A = 11;     // yellow
const int B2A = 10;     // green

void setup() {
    mpu.initialize();

    Serial.begin(115200);

    //Initialise the Motor outpu pins
    pinMode (A1A, OUTPUT);
    pinMode (A1B, OUTPUT);
    pinMode (B1A, OUTPUT);
    pinMode (B2A, OUTPUT);

    //By default turn off both the motors
    analogWrite(A1A,LOW);
    analogWrite(A1B,LOW);
    analogWrite(B1A,LOW);
    analogWrite(B2A,LOW);
}

void loop() {
    currTime = millis();
    loopTime = currTime - prevTime;
    prevTime = currTime;
  
    gyroX = mpu.getRotationX();
    gyroRate = map(gyroX, -32768, 32767, -250, 250);
    gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;
  
    Serial.println(gyroAngle);
}

// Ff kijken hoe we dit gebruiken
// void valuesMPU() {
//     Wire.beginTransmission(mpu);
//     Wire.write(0x3B);  
//     Wire.endTransmission(false);
//     Wire.requestFrom(mpu,12,true);  
//     // AcX=Wire.read()<<8|Wire.read();  //Niet nodig?
//     // AcY=Wire.read()<<8|Wire.read();  //Niet nodig?
//     AcZ=Wire.read()<<8|Wire.read();  
//     GyX=Wire.read()<<8|Wire.read();  
//     GyY=Wire.read()<<8|Wire.read();  
//     GyZ=Wire.read()<<8|Wire.read();
// }

void Forward() //Code to rotate the wheel forward 
{
    analogWrite(A1A, 255);
    analogWrite(A1B, 0);
    analogWrite(B1A, 255);
    analogWrite(B2A, 0);
    Serial.print("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
    analogWrite(A1A, 0);
    analogWrite(A1B, 255);
    analogWrite(B1A, 0);
    analogWrite(B2A, 255); 
    Serial.print("R");
}

void Stop() //Code to stop both the wheels
{
    analogWrite(A1A, 0);
    analogWrite(A1B, 0);
    analogWrite(B1A, 0);
    analogWrite(B2A, 0); 
    Serial.print("S");
}