#include <Wire.h>
#include <math.h>

//***************VARIABLES*******************
// Program State
int ps = 0; //everytime push button is pressed or function is completed it is incremented
int btnPress = 0; 

//output
float v_out;
const float v_max = 10.5;
int PWM_out;
//***************PINS**********************
const int BTN = 9; // Naming switch button pin
const int LED = 3;   // Namin LED pin
//Left motor pins.  
const int L_IN1 = 8;
const int L_IN2 = 7;
//Right motor pins
const int PWM = 6;
const int R_IN1 = 5; 
const int R_IN2 = 4;

void setup() {
  Serial.begin(115200);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  // Setup GPIO
  pinMode(LED, OUTPUT);
  pinMode (BTN, INPUT);

  //Motor setup
  pinMode(PWM, OUTPUT);
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);

//  motorStop();
  v_out= 2;
  motorDirection(true);
  motorPWM(v_out);
  Serial.print("V_out: ");
  Serial.print(v_out);
  Serial.print("  PWM: ");
  Serial.println(PWM_out);
}

void loop() {
  btnPress = digitalRead(BTN);  // Reading button status / input
  if (btnPress == HIGH)  // Condition to check button input
  {
    v_out = v_out+.25;
    motorPWM(v_out);
    Serial.print("V_out: ");
    Serial.print(v_out);
    Serial.print("  PWM: ");
    Serial.println(PWM_out);
    delay(1000);
  }
}


void motorDirection(bool dir) { //0 +ve, 1 -ve direction
  if (dir) {
    digitalWrite(L_IN1, 0);
    digitalWrite(L_IN2, 1);
    digitalWrite(R_IN1, 0);
    digitalWrite(R_IN2, 1);
  } else {
    digitalWrite(L_IN1, 1);
    digitalWrite(L_IN2, 0);
    digitalWrite(R_IN1, 1);
    digitalWrite(R_IN2, 0);
  }

}

void motorStop() {
  digitalWrite(L_IN1, 1);
  digitalWrite(L_IN2, 1);
  digitalWrite(R_IN1, 1);
  digitalWrite(R_IN2, 1);
}

void motorPWM(float v) {
  PWM_out = round(v*255/v_max);  //coverts controller v_out to appropriate PWM duty cycle, motor driver will convert into appropriate voltage.
                              //+0.5 ensures it always rounds to the nearest whole PWM, int truncates 
                             //Can use map function; however, it supresses fractions inside the function
  analogWrite(PWM, PWM_out);
}
