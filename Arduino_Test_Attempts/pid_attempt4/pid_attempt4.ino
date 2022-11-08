//https://circuitdigest.com/microcontroller-projects/arduino-based-self-balancing-robot

#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

#define A1B 3 // Brown
#define A1A 5 // Orange
#define B1A 9 // Green
#define B2A 6 // Yellow

MPU6050 mpu(0x68, &Wire);

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// middelpunt met afwijking naar voren
double setpoint= 189; 

//double Kp = 31;
//double Kd = 2.25;
//double Ki = 400; 
double Kp = 50;
double Ki = 0;
double Kd = 0;

double oldSetpoint = setpoint;

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
	Serial.begin(115200);

	Wire.begin();
	Wire.setClock(400000);

	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	devStatus = mpu.dmpInitialize();

//  mpu.setXGyroOffset(14);
//  mpu.setYGyroOffset(-93);
//  mpu.setZGyroOffset(6);
//  mpu.setZAccelOffset(1322); 

	mpu.setXGyroOffset(-1);
	mpu.setYGyroOffset(-91);
	mpu.setZGyroOffset(10);
	mpu.setZAccelOffset(1342); 

	if (devStatus == 0) {
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(2, dmpDataReady, RISING);

		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		pid.SetMode(AUTOMATIC);
		pid.SetSampleTime(10);
		pid.SetOutputLimits(-255, 255);  
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

  pid.SetTunings(Kp, Ki, Kd);

	pinMode (A1B, OUTPUT);
	pinMode (A1A, OUTPUT);
	pinMode (B2A, OUTPUT);
	pinMode (B1A, OUTPUT);

	Stop();
}

void loop() {
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        input = ypr[1] * 180/M_PI + 180;
    } else {
        pid.Compute();   
        Serial.println(input);
        Serial.println(output);

//        if (input<180 || input>190){
//        if (input<150 && input>200)

        //187.6; midden
        //190 afwijking
        // Achteruit of vooruit vallen
        if (input<190 || input>191){
          if (output<0) Forward();
          else if (output>0) Reverse();
        }
        else Stop();
    }
}

void Forward() {
  analogWrite(A1B,output*-1);
  analogWrite(A1A,0);
  analogWrite(B2A,output*-1);
  analogWrite(B1A,0);
  Serial.println("F");
}

void Reverse() {
  analogWrite(A1A,output);
  analogWrite(A1B,0);
  analogWrite(B1A,output);
  analogWrite(B2A,0);
  Serial.println("R");
}

void Stop() {
  analogWrite(A1B,0);
  analogWrite(A1A,0);
  analogWrite(B2A,0);
  analogWrite(B1A,0);
  Serial.println("S");
}
