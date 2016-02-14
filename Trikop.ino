#include <Servo.h>
#include <Wire.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <math.h>
 
Servo escR, escM, escL;
MPU6050 mpu;

#define MAX_SIGNAL 2100
#define MIN_SIGNAL 700

enum Motors { Left, Right, Middle };

#define LED_PIN 13

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

double outPitch, outYaw, outRoll, lastPitch, lastRoll;
double calb_Pitch, calb_Yaw, calb_Roll;

double software_limit = 1;

double calcRollDrift = 0;
double calcPitchDrift = 0;

double coef = 2; //main
double coefRoll = 1.5;

int map_min = coef;
int map_max = 100 + coef;

int throttle, throttleL, throttleM, throttleR;

unsigned long time;
bool isReady = false;


void blinkLed()
{
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
void setup()
{
    Wire.begin(); 
    Serial.begin(115200);
    TWBR = 24; 
    mpu.initialize();
    mpu.testConnection();
    delay(500);
    devStatus = mpu.dmpInitialize();
    
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);
    
    
    if (devStatus == 0) {
          // turn on the DMP, now that it's ready
          mpu.setDMPEnabled(true);
    
          // enable Arduino interrupt detection
          attachInterrupt(0, dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();
    
          dmpReady = true;
    
          // get expected DMP packet size for later comparison
          packetSize = mpu.dmpGetFIFOPacketSize();
      } else {
          // ERROR!
          // 1 = initial memory load failed
          // 2 = DMP configuration updates failed
          // (if it's going to break, usually the code will be 1)
          Serial.print(F("DMP Initialization failed (code "));
          Serial.print(devStatus);
          Serial.println(F(")"));
      }
    
    
    escR.attach(9);
    escM.attach(10);
    escL.attach(11);
    
    pinMode(LED_PIN, OUTPUT);
    delay(1000);
    
    while(calb_Roll == 0  && dmpReady)
      getIMUData(true);
}

void setMotorSpeed(Motors index, int i_speed)
{
    i_speed = map(i_speed, 0, 100, MIN_SIGNAL, (MAX_SIGNAL * software_limit));
    
    if(i_speed < MIN_SIGNAL) i_speed = MIN_SIGNAL;
    if(i_speed > MAX_SIGNAL) i_speed = MAX_SIGNAL;
    
    if(index == Right)
      escR.writeMicroseconds(i_speed);
    else if(index == Middle)
      escM.writeMicroseconds(i_speed);
    else if(index == Left)
      escL.writeMicroseconds(i_speed);
    else //security
    {
      escR.writeMicroseconds(MIN_SIGNAL);
      escM.writeMicroseconds(MIN_SIGNAL);
      escL.writeMicroseconds(MIN_SIGNAL);
    }
}

void getIMUData(bool init)
{
    lastRoll = outRoll;
    lastPitch = outPitch;
    
     while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
    
    }
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    
     // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
        outYaw = ypr[0] * 180/M_PI;
        outPitch = ypr[1] * 180/M_PI;
        outRoll = ypr[2] * 180/M_PI;
    
        if(init)
        {
          calb_Pitch = outPitch;
          calb_Roll = outRoll;
          calb_Yaw = outYaw;
        }
        blinkLed();
    }
}

void loop()
{
    if (!dmpReady)
    {
       Serial.println(analogRead(0));
       delay(1000);
       blinkLed();
       return;
    }
    
    time = millis();
    
    if(time > 20000 && isReady != true)
    {
      isReady = true;
      getIMUData(true);
    }
    else
      getIMUData(false);
    
    throttle = analogRead(0);
    throttle = map(throttle, 0, 1023, map_min, map_max);
    
    calcRollDrift = 1 - outRoll/calb_Roll;
    calcPitchDrift = 1 - outPitch/calb_Pitch;
    
    //Serial.println(throttle - fabs(calcPitchThrottle) * coef);
    Serial.print(throttle, DEC);
    Serial.print(",");
    Serial.print(calcPitchDrift, DEC);
    Serial.print(",");
    Serial.print(calcRollDrift, DEC);
    Serial.print(",");
    Serial.print(calcPitchDrift * coef, DEC);
    Serial.print(",");
    Serial.print(calcRollDrift * coef * coefRoll, DEC);
    Serial.print(",");
    Serial.print(time, DEC);
    Serial.print(",");
    Serial.print(outRoll,DEC);
    Serial.print(",");
    Serial.print(outPitch, DEC);
    Serial.print(",");
    Serial.print(outYaw, DEC);
    Serial.println();
  
    throttleL = throttle;
    throttleR = throttle;
    
    throttleL = throttle - (calcRollDrift  * coef * coefRoll);
    
    throttleR = throttle + (calcRollDrift  * coef * coefRoll);
    
    throttleM = throttle + (calcPitchDrift * coef);
    
    if(isReady)
    {
      setMotorSpeed(Left, throttleL);
      setMotorSpeed(Right, throttleR);
      setMotorSpeed(Middle, throttleM);
    } else
    {
      setMotorSpeed(Left, 0);
      setMotorSpeed(Right, 0);
      setMotorSpeed(Middle, 0);
    }

}

//copyright moncef ben slimane
