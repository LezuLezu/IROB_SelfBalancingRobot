#include "I2Cdev.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;

// Motor Links (vooraanzicht)
const int A1A = 6;      // orange
const int A1B = 9;      // brown
// Motor Rechts (vooraanzicht)
const int B1A = 11;     // yellow
const int B2A = 10;     // green



// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*********Tune these 4 values for your BOT*********/
double setpoint= 176; //set the value when the bot is perpendicular to ground using serial monitor. 

//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 21; //Set this first
double Kd = 0.8; //Set this secound
double Ki = 140; //Finally set this 
/******End of values setting*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

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

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-5);
    mpu.setYGyroOffset(-98);
    mpu.setZGyroOffset(-11);
    // wont likely need x and y accel offsets
    mpu.setXAccelOffset(650);
    mpu.setYAccelOffset(-656);
    mpu.setZAccelOffset(3338); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);  
    }

    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

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
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();   

        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input); Serial.print(" =>"); Serial.println(output);

        if (input>150 && input<200){//If the Bot is falling 
        if (output>0) //Falling towards front 
            Forward(); //Rotate the wheels forward 
        else if (output<0) //Falling towards back
            Reverse(); //Rotate the wheels backward 
        }
        else //If Bot not falling
            Stop(); //Hold the wheels still
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

        input = ypr[1] * 180/M_PI + 180;
   }
}

void Forward() //Code to rotate the wheel forward 
{
    analogWrite(A1A, output);
    analogWrite(A1B, 0);
    analogWrite(B1A, output);
    analogWrite(B2A, 0);
    Serial.print("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
    analogWrite(A1A, 0);
    analogWrite(A1B, output*-1);
    analogWrite(B1A, 0);
    analogWrite(B2A, output*-1); 
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