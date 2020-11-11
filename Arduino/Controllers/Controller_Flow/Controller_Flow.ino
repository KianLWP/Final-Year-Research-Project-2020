#include <Wire.h>
#include <VL53L1X.h>
#include "Bitcraze_PMW3901.h"
#include "MPU6050_6Axis_MotionApps20.h"



// ================================================================
// ===                        VARIABLES                         ===
// ================================================================
VL53L1X rangeSensor;
// Using digital pin 10 for chip select, SPI is initialized within the function
Bitcraze_PMW3901 flow(10);
MPU6050 mpu;

//***************DMP VARIABLES*******************

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

//***************VARIABLES*******************
// Program State
int ps = 0; //everytime push button is pressed or function is completed it is incremented
int btnPress = 0; 
//Loop variable
int c = 0;

//Flow variables
float dx, dtheta, dthetaErr, theta, thetaPrev, z2, z1, r, dz, temp1, temp2;
float r_w = 40/1000;
float FOV = 42*PI/180; //FOV of optical flow sensor
float Nx = 35; //pixel width
unsigned long flow_dt, flow_t0, flow_t1;
int16_t deltaX,deltaY;


//***************CONTROLLER*****************
//float K1 = 0, K2 = 0, K3 = 0, K4 = 0;

//Ziegler Nichols
//Finding Ultimate gain and time
//float Kp = 150, Ki = 0, Kd = 0; //set gains to 0 and increase Kp until neutrally stable

//Ultimate Variables
float Ku = 150, Tu = 0.2168; //Found using Ziegler Nichols tuning method

      
//Gain Table

//Classic
// float Kp = Ku/2, Ki = 0, Kd = 0; //P
// float Kp = Ku/2.2, Ki = 1.2*Kp/Tu, Kd = 0; //PI
//float Kp = Ku*0.6, Ki = 2*Kp/Tu, Kd = Kp*Tu/8; //PID

//overshoot
//float Kp = 0.2*Ku;  //No overshoot
float Kp = 0.33*Ku; //Some overshoot
float Ki = 2*Kp/Tu, Kd = Kp*Tu/3; 

//Pessen Integral Rule
//float Kp = 0.7*Ku, Ki = 2.5*Kp/Tu, Kd = 0.15*Kp/Tu;



//Controller variables
float errTot = 0, errPrev = 0;
//Time
unsigned long dt = 0, t0 = 0, t1 = 0; //time variable
//output  
const float v_min = 3.5; 
const float v_max = 10.5;
float v_out;
int PWM_out;


//***************PINS**********************
#define INTERRUPT_PIN 2  // used for the DMP
#define LED 3   // LED pin
#define BTN 9 // switch button pin
//Left motor pins. 
//const int L_PWM = 9; 
#define L_IN1 8
#define L_IN2 7
//Right motor pins
#define R_PWM 6 //Using same PWM pin for consistency
#define R_IN1 5 
#define R_IN2 4

//**********PROGRAM SETTINGS**************
#define CALIBRATE_IMU;//comment to skip IMU calibration and use pre-calibrated offsets
//#define ZN_TUNING; //uncomment for Ziegler Nichols Tuning

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
  pinMode(BTN, INPUT);

  //Motor setup
  pinMode(R_PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

  Serial.println("Setup complete, awaiting user input");
  Serial.println("-----------------------------------");
  
  #ifdef ZN_TUNING
    Kp = 5;
    Ki = 0;
    Kd = 0;
    Serial.println("Ziegler Nichols Tuning");
  #endif

  Serial.print("Used gains: Kp=");
  Serial.print(Kp);
  Serial.print("  Ki=");
  Serial.print(Ki);
  Serial.print("  Kd=");
  Serial.println(Kd);
  Serial.println("-----------------------------------");
  
}

void loop() {
  ps = ps%5;  //ensures program state is from 0-3, including both 0 and 3
  switch (ps){
  case 1:
    calibrate();
    Serial.println("Ready to balance. Press button to start");
    Serial.println("---------------------------------------");
    ps++;
    break;
  case 2:
    c++;
    //do nothing, awaiting further user input
    digitalWrite(LED, (c/5000 % 2)); //blinks letting user know robot is ready
    break;
  case 3:
    getAng();
    v_out = PID_controller(theta); //angle in rads
    motorVoltage(v_out); //send voltage to motors
    
    //Log sample time
    Serial.print("\t");
    Serial.println(dt);   
    break;
  case 4:
    #ifdef ZN_TUNING
      Kp = Kp + 5;
      Serial.print("Increasing Kp: ");
      Serial.println(Kp);
    #endif
    ps++;
    break;
  default:
    motorStop();
    break;
  }  
  btnPress = digitalRead(BTN);  // Reading button status / input
  if (btnPress == HIGH)  // Condition to check button input
  {
    Serial.println("Button Press");
    ps++;
    delay(1000); //debounce time and delay
    if (ps == 3)
    {
      digitalWrite(LED,1);
      // ensure variables are reset for controller
      errTot = 0;
      errPrev = 0;
      t1 = micros(); 
    }
  }
}


// ================================================================
// ===                   CONTROLLER FUNCTION                    ===
// ================================================================
float PID_controller(float err) { 
  //input is error, desired angle-current angle
  float result;
  //time
  t0 = t1;
  t1 = micros();
  dt = (t1-t0); //in ms
  //Controller output
  //Kp[e(t)] + Ki[integral(e(t).dt)] + Kd[e(t).d/dt]
  errTot = errTot+err;
  result = Kp*err + Ki*errTot*dt/1000000 + Kd*(err-errPrev)*1000000/dt; 
  errPrev = err;
  return result;
}

// ================================================================
// ===                      MOTOR FUNCTION                      ===
// ================================================================
void motorVoltage(float v) {
  Serial.print("\t");
  if (v>0) {
    motorDirection(false);
  }
  else if (v<0) {
    motorDirection(true);
    v = v*-1; //only +ve PWM signals are sent out
    Serial.print("-"); //print sign of voltage, used for voltage logging
  }
  v = constrain(v, v_min, v_max); //prevent voltage from going out of bounds
  Serial.print(v);
  Serial.print("\t");
  motorPWM(v); //convert to PWM
}

void motorDirection(bool dir) { //0 +ve, 1 -ve direction
  if (dir) {
    digitalWrite(L_IN1, 1);
    digitalWrite(L_IN2, 0);
    digitalWrite(R_IN1, 1);
    digitalWrite(R_IN2, 0);
  } else {
    digitalWrite(L_IN1, 0);
    digitalWrite(L_IN2, 1);
    digitalWrite(R_IN1, 0);
    digitalWrite(R_IN2, 1);
  }
}

void motorStop() {
  digitalWrite(L_IN1, 1);
  digitalWrite(L_IN2, 1);
  digitalWrite(R_IN1, 1);
  digitalWrite(R_IN2, 1);
}

void motorPWM(float v) {
  PWM_out = round(v*255/v_max);  //maps voltage to appropriate PWM duty cycle, motor driver will convert into appropriate voltage. 
                              //Can use map function; however, it supresses fractions inside the function
  analogWrite(R_PWM, PWM_out);   
}

// ================================================================
// ===                      TOF FUNCTIONS                       ===
// ================================================================
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
  flow_t0 = flow_t1;
  flow_t1 = micros();
  flow_dt = flow_t1-flow_t0;
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

  dx = -z2*FOV/(flow_dt*Nx)*1000000 * deltaX; 
  dtheta = dx/r;
  temp1 = abs(thetaPrev + dtheta*flow_dt/1000000 - theta);
  temp2 = abs(thetaPrev + dtheta*flow_dt/1000000 + theta);
  if (temp1>temp2){ //deciding whether angle should be negative or positive
    theta = -theta;
  } 
  Serial.print("\t");
  Serial.print(theta);
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
  
//OFFSETS    -4076,    -635,    1448,    -709,      44,     -34
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
