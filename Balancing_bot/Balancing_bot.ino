/* ======================================================================================

IROB SELFBALANCING BOT

Self balancing bot gemaakt door: Zoe en Sabrina
Code gebaseerd op tutorial: https://circuitdigest.com/microcontroller-projects/arduino-based-self-balancing-robot

====================================================================================== */

#include "I2Cdev.h" // 12C Device lib
#include <PID_v1.h> // PID lib
#include "MPU6050_6Axis_MotionApps20.h" // MPU lib
#include <Wire.h> // Wire lib, voor communicatie met MPU

#define A1B 3 // Bruin
#define A1A 5 // Orange
#define B1A 9 // Groen
#define B2A 6 // Geel

MPU6050 mpu(0x68, &Wire); // Maak MPU object met wire connectie 

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// middelpunt met afwijking naar voren om gewicht van batterij te kunnen compenseren
double setpoint= 187; 

// bij stellen: https://www.youtube.com/watch?v=cjSw7sc2JKk&t=1s&ab_channel=CircuitDigest
double Kp = 51;
double Kd = 2.25;
double Ki = 200;

double oldSetpoint = setpoint;

double input, output;
// PID object waar doormiddel van input een output pwm waarden wordt gegeneerd. Setpoint is het middelpunt
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT); 

volatile bool mpuInterrupt = false;
void dmpDataReady() {
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

// Oude ofset waardes van vorige calibraties:

//  mpu.setXGyroOffset(14);
//  mpu.setYGyroOffset(-93);
//  mpu.setZGyroOffset(6);
//  mpu.setZAccelOffset(1322); 

//	mpu.setXGyroOffset(-1);
//	mpu.setYGyroOffset(-91);
//	mpu.setZGyroOffset(10);
//	mpu.setZAccelOffset(1342); 

  mpu.setXGyroOffset(3);
  mpu.setYGyroOffset(-92);
  mpu.setZGyroOffset(20);
  mpu.setZAccelOffset(1337); 

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
		// 1 = gefaald te laden
		// 2 = DMP configuratie update gefaald
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

  // MPU FIFO buffer met alle uitgelezen waardes:
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    input = ypr[1] * 180/M_PI + 180;
  } else {
    // Bereken PID
    pid.Compute();  
    Serial.println(input);
    Serial.println(output); 

    //187 midden met afwijking
    // < Achteruit of vooruit > vallen
    if (input<187 || input>187){
      if (output<0) Forward();
      else if (output>0) Reverse();
    }
    else Stop();
  }
}

void Forward() {
  analogWrite(A1B,output*-1); // Conventeer negatieve waarde naar bruikbare pwm
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
