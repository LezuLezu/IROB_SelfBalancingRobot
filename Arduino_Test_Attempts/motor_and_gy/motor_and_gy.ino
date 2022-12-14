#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#include "math.h"

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

// Motor Links (vooraanzicht)
const int A1A = 9;      // orange
const int A1B = 6;      // brown
// Motor Rechts (vooraanzicht)
const int B1A = 11;     // yellow
const int B2A = 10;     // green

const int Kp = 40;
const int Kd = 0.05;
const int Ki = 40;

int16_t accY, accZ, gyroX;
volatile int motorPower = 255;
volatile float accAngle;

void setup() {

    Serial.begin(115200);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(14);
    mpu.setYGyroOffset(-93);
    mpu.setZGyroOffset(6);
    // wont likely need x and y accel offsets
    mpu.setXAccelOffset(646);
    mpu.setYAccelOffset(1038);
    mpu.setZAccelOffset(1322); 

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
    // initialize PID sampling loop
    init_PID();
}

void loop() {
    // read acceleration and gyroscope values
    accY = mpu.getAccelerationY();
    accZ = mpu.getAccelerationZ();
    gyroX = mpu.getRotationX();

    Serial.println(accAngle);
   //Midden is 150.34 voor currentAngle
    if (accAngle < 0){
      Reverse();
    }
    if (accAngle > 0) {
      Forward();
    } 
//    if (accAngle >= 0 && accAngle <= 0.5) {
//      Stop();
//    }
}

void init_PID() {  
  // initialize Timer1
  cli();
  TCCR1A = 0;
  TCCR1B = 0;   
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11);
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
}

void Forward() //Code to rotate the wheel forward 
{
    analogWrite(A1A, motorPower);
    analogWrite(A1B, 0);
    analogWrite(B1A, motorPower);
    analogWrite(B2A, 0);
    Serial.println("F");
}

void Reverse() //Code to rotate the wheel Backward  
{
    analogWrite(A1A, 0);
    analogWrite(A1B, motorPower);
    analogWrite(B1A, 0);
    analogWrite(B2A, motorPower); 
    Serial.println("R");
}

void Stop() //Code to stop both the wheels
{
    analogWrite(A1A, 0);
    analogWrite(A1B, 0);
    analogWrite(B1A, 0);
    analogWrite(B2A, 0); 
    Serial.println("S");
}
