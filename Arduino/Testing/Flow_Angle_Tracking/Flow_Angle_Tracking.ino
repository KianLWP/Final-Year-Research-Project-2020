#include <Wire.h>
#include <VL53L1X.h>
#include "Bitcraze_PMW3901.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ================================================================
// ===                        VARIABLES                         ===
// ================================================================

MPU6050 mpu;
VL53L1X rangeSensor;
// Using digital pin 10 for chip select, SPI is initialized within the function
Bitcraze_PMW3901 flow(10);

//Program state
int ps = 0; //everytime push button is pressed or function is completed it is incremented
int btnPress = 0; 

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
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//FLowdeck
float dx, dtheta, dthetaErr, theta, thetaPrev, z2, z1, r, dz, temp1, temp2;
float r_w = 40/1000;
float FOV = 42*PI/180; //FOV of optical flow sensor
float Nx = 35; //pixel width
unsigned long dt, t0, t1;
int16_t deltaX,deltaY;

//***************PINS**********************
#define LED 3   // LED pin
#define BTN 9 // switch button pin
#define INTERRUPT_PIN 2 //used for DMP


void setup() {
  setupIMU();  
  // Initialize flow sensor
  if (!flow.begin()) {
    Serial.println("Initialization of the flow sensor failed");
    while(1) { }
  }
  //Initialize VL53L1X
  rangeSensor.init();
  rangeSensor.setTimeout(500);
  rangeSensor.setDistanceMode(VL53L1X::Short);
  rangeSensor.setMeasurementTimingBudget(20000); //20ms minimum timing budget for Short distance reading
  rangeSensor.startContinuous(20); //20ms >= than timing budget
    
  // Setup GPIO
  pinMode(LED, OUTPUT);
  pinMode (BTN, INPUT);
  delay(250);
}


void loop() {
  ps = ps%4;  //ensures program state is from 0-3, including both 0 and 3
//  IMU_data();
  switch (ps){
  case 1:
    
    calibrate();
    Serial.println("Ready... waiting for user input");
    ps++;
    break;
  case 2:
    // do nothing
    break;
  case 3:
    getAng();
    break;
  default:
    break;
  }  
  btnPress = digitalRead(BTN);  // Reading button status / input
  if (btnPress == HIGH)  // Condition to check button input
  {
    Serial.println("Button Press");
    ps++;
    delay(500); //debounce time
    if (ps == 3)
    {
      t1 = micros();  //ensures correct dt value for theta logging
    }
  }
}

void getAng() {
  //Get dtheta using IMU
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Log angle and sample time
    Serial.print(ypr[1],4); 
    Serial.print("\t");
    Serial.print(dt);  
  } 
  //using flow
  //get sample time
  t0 = t1;
  t1 = micros();
  dt = t1-t0;
  // Get data from sensors
  flow.readMotionCount(&deltaX, &deltaY);
  z2  = rangeSensor.readRangeContinuousMillimeters();
  z2 = z2/1000;
  //calculate angle
  thetaPrev = theta;
  if (z2<=z1) {
    theta = acos((z2-r_w)/r);
  } else { 
    theta = 0;
  }

  dx = -z2*FOV/(dt*Nx)*1000000 * deltaX; 
  dtheta = dx/r;
  temp1 = abs(thetaPrev + dtheta*dt/1000000 - theta);
  temp2 = abs(thetaPrev + dtheta*dt/1000000 + theta);
  if (temp1>temp2){ //deciding whether angle should be negative or positive
    theta = -theta;
  } 
  Serial.print("\t");
  Serial.println(theta);
  delay(20);
}


void calibrate() {
  calibrateTOF();
  initializeDMP();
}
 
void calibrateTOF() {
  Serial.println("TOF calbration");
  Serial.print("<");
  int c;
  z1 = 0;
  for (c=0; c<50; c= c+1) {
    z1 = z1 + rangeSensor.readRangeContinuousMillimeters();
    digitalWrite(LED, (c/5 % 2)); //Flashes every 100 iterations, notifying user of calibration
    if ((c%2) == 0) {
      Serial.print("."); //progress bar for serial monitor
    }
    delay(20);
  }
  digitalWrite(LED, 0);
  Serial.println(">");
  z1 = z1/c/1000; //find average and convert to meters
  r = z1-r_w; 
  Serial.print("ToF Sensor calibrated: ");
  Serial.println(z1,6);
  delay(1000);
}


// ================================================================
// ===                      IMU FUNCTIONS                       ===
// ================================================================
// Code in this section was taken and adapted from Jeff Rowbergs MPU6050 I2Clibrary for Arduino. The example used as reference is cited
/***************************************************************************************
*    Title: MPU6050_DMP6.ino
*    Author: Jeff Rowberg
*    Date: 2019
*    Code version: 2.0
*    Availability: https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6
*   
***************************************************************************************/
void setupIMU(){
  // IMU Setup, taken from Jeff Rowberg
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
}

void initializeDMP() { //adapted from Jeff Rowberg
  digitalWrite(LED, HIGH);
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
 
  mpu.setXAccelOffset(-4076);
  mpu.setYAccelOffset(-635);
  mpu.setZAccelOffset(1448); 
  mpu.setXGyroOffset(-709);
  mpu.setYGyroOffset(44);
  mpu.setZGyroOffset(-34);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    #ifdef CALIBRATE_IMU
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
    #endif
    
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
    Serial.println(F("DMP ready"));
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
  digitalWrite(LED, LOW);
}

//Interruption Detection

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
