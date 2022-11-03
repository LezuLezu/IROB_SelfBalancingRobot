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
const int targetAngle = -2.5;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;

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

    mpu.setXGyroOffset(-53);
    mpu.setYGyroOffset(-93);
    mpu.setZGyroOffset(12);
    // wont likely need x and y accel offsets
    mpu.setXAccelOffset(444);
    mpu.setYAccelOffset(1044);
    mpu.setZAccelOffset(5420); 

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
//    Serial.println(accY);
    gyroX = mpu.getRotationX();

   
    if(currentAngle < targetAngle){
        Forward();
    }
    else if(currentAngle > targetAngle) {
        Reverse();
    } 
    else {
        Stop();
    }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum) - Kd*(currentAngle-prevAngle);
  Serial.println(accAngle);
  prevAngle = currentAngle;
}

void Forward() //Code to rotate the wheel forward 
{
    analogWrite(A1A, motorPower);
    analogWrite(A1B, 0);
    analogWrite(B1A, motorPower);
    analogWrite(B2A, 0);
    Serial.print("F");
}

void Reverse() //Code to rotate the wheel Backward  
{
    analogWrite(A1A, 0);
    analogWrite(A1B, motorPower);
    analogWrite(B1A, 0);
    analogWrite(B2A, motorPower); 
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