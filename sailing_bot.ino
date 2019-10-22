#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define MODE_BUTTON 4

Servo sailRudder;  // create servo object to control a servo
Servo rudder;
MPU6050 mpu;
// twelve servo objects can be created on most boards

// led timer

unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;
unsigned int ledState = LOW;

const long interval = 500;           // interval at which to blink (milliseconds)

// button states

int pos = 0;    // variable to store the servo position
int windAngle = 0;

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

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}

// ================================================================
// ===                      ENUM                                ===
// ================================================================

enum mode_t
{
    LINE,
    CIRCLE,
    EIGHT,
};

mode_t mode = LINE;

// ================================================================
// ===                      FUNCTION PROTOTYPES                 ===
// ================================================================
void setMPU6050();
void getValMPU60();
unsigned int getMode();
void blinkLed();

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Serial.begin(115200);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    setMPU6050();

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    rudder.attach(9);
    rudder.write(90);

    sailRudder.attach(10);
    sailRudder.write(90);

    pinMode(MODE_BUTTON, INPUT_PULLUP);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    currentMillis = millis();
    getValMPU60();
    mode = getMode();

    switch(mode)
    {
        case LINE:
        {
            digitalWrite(LED_PIN, LOW);
            lineAction();
        }
        break;
        case CIRCLE:
        {
            digitalWrite(LED_PIN,HIGH);
            circleAction();
        }
        break;
        case EIGHT:
        {
            blinkLed();
            eightAction();
        }
        break;
    }
}

#define MAX_FLAP_ANGLE  165
#define MIN_FLAP_ANGLE  15

void lineAction()
{
  int sailAngle = 90+(2*ruzgarAcisi);
  int rudderAngle = 90+ruzgarAcisi;
  if(sailAngle > MAX_FLAP_ANGLE)
  {
    sailRudder.write(MAX_FLAP_ANGLE);
  }
  else if(sailAngle < MIN_FLAP_ANGLE)
  {
    sailRudder.write(MIN_FLAP_ANGLE);
  }
  else
  {
    sailRudder.write(sailAngle);
  }
  
  if(rudderAngle > 135)
  {
    rudder.write(135);
  }
  else if(rudderAngle < 45)
  {
    rudder.write(45);
  }
  else
  {
    rudder.write(rudderAngle);
  }
}

void circleAction()
{
  
}

void eightAction()
{

  
}
unsigned int getMode()
{
    static unsigned int res = 0;
    if(digitalRead(MODE_BUTTON) == 0)
    {
        delay(500);
        if(++res == 3) res = 0;
    }
    return res;
}

void blinkLed()
{
    if (currentMillis - previousMillis >= interval) 
    {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (ledState == LOW) 
    {
        ledState = HIGH;
    }
    else 
    {
        ledState = LOW;
    }
    // set the LED with the ledState of the variable:
    digitalWrite(LED_PIN, ledState);
  }
}

void setMPU6050()
{
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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
}

void getValMPU60()
{
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if(fifoCount < packetSize){
            //Lets go back and wait for another interrupt. We shouldn't be here, we got an interrupt from another event
            // This is blocking so don't do it   while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {

        // read a packet from FIFO
    while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            ruzgarAcisi = (int)(ypr[0] * 180/M_PI); 
            Serial.print("Rüzgar Açısı: \t");
            Serial.println(ruzgarAcisi);
        #endif
    }
}
