
// https://wired.chillibasket.com/2020/05/servo-trajectory/
#include "trajectory.h"
#include <math.h>
#include <time.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <string.h>
#include <stdio.h>
// #include <EEPROMex.h>
#include <Preferences.h>
#include "esp_timer.h"    // better alternative to micros()

Preferences preferences;

Adafruit_LSM6DS3TRC lsm6ds3trc;


// Z-Axis = Standing Desk frame (up/down)
// A-Axis = Rotation/Pivot motor (rotation)

// uC definitions: 


#define EN_A    19  // Red wire - Active Low Enable
#define IN1_A   18  // Grey wire 
#define IN2_A   17  // White wire 
#define ENC1_A  34  // White wire
#define ENC2_A  35  // Blue wire

#define ENC_DEBOUNCE 100 //micro seconds


#define KEY_MAX_ACCEL  "max_accel_"
#define KEY_MAX_SPEED  "max_speed_"
#define KEY_KP         "kp_"
#define KEY_KI         "ki_"
#define KEY_KD         "kd_"
#define KEY_DEADZONE   "deadzone_"
#define KEY_I_DECAY    "i_decay_"


#define PWM_FREQ 30000
#define PWM_BITS  8
#define PWM_MAX   ((1<<PWM_BITS) -1)

#define MIN_PWM_VAL_A 1  //60 is reasonable instantaneous start value for 18.4V supply
#define MAX_PWM_VAL_A 180 // actual max is 255 - motor at 30v is too powerful   Depending how timers are setup, the max "PWM" value might change
#define MAX_INTEGRAL_A 170.0 // A Basically limits the acceleration sorta?

#define ANGLE_LIM_CW    -10   // DEGREES as input by user.  Measured how the code sees it
#define ANGLE_LIM_CCW   179   // DEGREES as input by user.  If i go above 180 in this direction, it will roll over and make life more complicated


#define PRINT_INTERVAL 90
unsigned long lastPrint = 0;
float calcThreshold = 0.0001;         // used by trajectory library


float setAngle;    // (radians) Only angle on this project, so this should be a global dealio
float currAngle;   // (radians)

volatile int tempCurrPos = 0; // encoder count

struct AxisParams {
  char letter;       // using lowercase as convention

  volatile long pos_curr = 0;   // Encoder position
  volatile long pos_set = 0;

  float set = 0;        // SIGNED PWM value - stored in a much larger type to prevent overflow 
  float kp;
  float ki;
  float kd;
  int8_t saturation = 0;  // Saturation flag for A-axis
  bool stopped = false;
  volatile bool motorOff = false;
  float maxAccel;      // rads / sec^2
  float maxSpeed;     // rads / sec ?


  float prevPos = 0;   // for derivative calc (encoder pos)
  float prevError = 0;    // For a different derivative calc (encoder pos)
  
  float error = 0;      // Current pid raw error (encoder pos)
  float xi = 0;  // Integral Term Scaled Error (PWM Val)
  float iDecay = 1; // Integral Term decay (PWM Val*time coeff)
  float xd = 0;  // Derivative Term Scaled Error (PWM Val)
  float deadzone = 0;
  Trajectory *trajectory;
};



////////////// Serial vars
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

// variables to hold the parsed data
char cmd[numChars] = {0};
float cmdVal = -10.0;
bool newData = false;
#define DELIM " "


unsigned long time1 = 0;
unsigned long time2 = 0;
uint32_t tElapsed = 0;


float accX;
float accY;
float accZ;   //Not used

float gyroX;   //Not used
float gyroY;   //Not used
float gyroZ; 


void prefs_init(float* defaultVal, const char* keyName, char* letter) {
  
  char key[25];
  strcpy(key, keyName);
  strcat(key, letter);

  if (!preferences.isKey(key)) {
    preferences.putFloat(key, *defaultVal);
  }
  else {
    float temp = preferences.getFloat(key);
    if (isnan(temp)) {
      preferences.putFloat(key, *defaultVal);
    }
    else {
      *defaultVal = temp;
    }
  }
}


AxisParams a;
Trajectory trajectory_a(calcThreshold);

// Function called when interrupt on axis a encoder
void readEncoder_a() {
    int b = digitalRead(ENC1_A);
    if (b>0) {
      // a.pos_curr += 1;
      tempCurrPos -= 1;
    }
    else {
      // a.pos_curr -= 1;
      tempCurrPos += 1;
    }
}

unsigned long currMillis;

void setup() {

  Serial.begin(115200);
  
  // Defaults for axis A
  a.letter = 'a';
  a.kp = 0;
  a.ki = 0;
  a.kd = 0;
  a.maxAccel = .02;
  a.maxSpeed = .2;
  a.deadzone = 0.8*DEG_TO_RAD;
  a.trajectory = &trajectory_a;

  // Encoder Interrupt
  pinMode(ENC1_A, INPUT);
  pinMode(ENC2_A, INPUT_PULLDOWN);
  attachInterrupt(ENC2_A, readEncoder_a, RISING);


  ledcAttach(IN1_A, PWM_FREQ, PWM_BITS);
  ledcAttach(IN2_A, PWM_FREQ, PWM_BITS);
  ledcWrite(IN1_A, 0);
  ledcWrite(IN2_A, 0);
  
  pinMode(EN_A, OUTPUT);
  digitalWrite(EN_A, 0);

  // accel_test();
  // delay(200);

  preferences.begin("tv-stand", false);

  // lsm6ds3trc.readAcceleration(accX, accY, accZ);
  // setAngle = calculateAngle(accX, accY);  //Makes sure the system setpoint starts where it currently is, that way we can enforce acceleration control
  // lsm6ds3trc.readGyroscope(gyroX, gyroY, gyroZ);

  // a.trajectory->setTargetPos(0);
  // a.trajectory->setPos(0);
  

  prefs_init(&a.maxAccel, KEY_MAX_ACCEL, &a.letter);
  a.trajectory->setAcc(a.maxAccel);
  a.trajectory->setDec(a.maxAccel);

  prefs_init(&a.maxSpeed, KEY_MAX_SPEED, &a.letter); // max speed axis a (rad/sec)
  a.trajectory->setMaxVel(a.maxSpeed);

  prefs_init(&a.kp, KEY_KP, &a.letter);
  prefs_init(&a.ki, KEY_KI, &a.letter);
  prefs_init(&a.kd, KEY_KD, &a.letter);     // kd axis a
  prefs_init(&a.deadzone, KEY_DEADZONE, &a.letter);     // deadzone axis a
  prefs_init(&a.iDecay, KEY_I_DECAY, &a.letter);
  a.xi = 0;

}

void loop() {

  recvWithEndMarker();  // User input over serial
  if (newData) {
    strcpy(tempChars, receivedChars);

    parseCommand();

    Serial.print("ECHO: ");
    Serial.print(cmd);
    Serial.print("\tVAL: "); Serial.print(cmdVal,1);
    Serial.println();

    processCommand();

    newData = false;
  }
  

  // Should be more accurate, now that the code actually waits for a new acceleration
  // That way the timings are stable and actually apply to the time period - no sense in having the PID run faster than the accel?
  // Time =~ 19500 = 52HZ data rate from accel
  // if(lsm6ds3trc.accelerationAvailable()) {

  //   lsm6ds3trc.readAcceleration(accX, accY, accZ);
    
  // Time calc for PID (microseconds)
  time2 = micros();
  if ( time1 == 0) {
    time1 = time2 - 100000;
  }
  tElapsed = (time2 - time1);
  time1 = micros();
  
  // Acceleration, decel, max velocity
  // a.pos_set = a.trajectory->update(tElapsed);

  // Reference angle on bootup, will use later
  // currAngle = calculateAngle(accX, accY);


  a.error = calcPosError(a.pos_set, a.pos_curr);

  if (!a.motorOff) {
    digitalWrite(EN_A, 0);  
    digitalWrite(EN_A, 1);
    digitalWrite(EN_A, 0);  // Super jank way to maybe deal with H bridge cutting out?
  }

  // Pid output calculation ----------------------------------

  if (( a.saturation < 0 && a.error < 0) || (a.saturation > 0 && a.error > 0)) {
    // Do nothing if there is saturation and error is in the same direction
  }
  else if (a.stopped) {
    // Do nothing
    a.set = 0;
    a.xi = 0;
  }
  else {
    if(a.error == 0 && a.xi != 0) {
      if (abs(a.xi) < .01) {
        a.xi = 0;
      }
      a.xi = a.xi*a.iDecay;   // Probably isnt technically a good idea, but since this system requires the motor to be off at steady state, should be fine
    }
    else {
      a.xi = a.xi + a.ki*(a.error*tElapsed)/100000.0;    // would be (Degree * Seconds), but by scaling it, we integrate "PWM Values"
    }
    satLimit(&a.xi, &a.saturation, MAX_INTEGRAL_A);
    a.xd = a.kd*1000*(a.prevError - a.error)/tElapsed;     // Derivative of set point (might work better given the acceleration on the target)
    //a.xd = -a.kd*1000*(a.prevPos - a.currAngle)/tElapsed;   // Derivative (of input hence neg required but no derivative kick)
    // a.xd = a.kd*gyroZ/10000;   // instead of derivative -> use gyro output!

    a.set = (a.kp*a.error + a.xi + a.xd);
    limit_pwm(&a.set, MAX_PWM_VAL_A);
  }
  a.prevPos = a.pos_curr;
  a.prevError = a.error;

  //------------------------------------------------------------
  moveMotor(&a, EN_A, IN1_A, IN2_A, MIN_PWM_VAL_A);
  // if (a.set > 0) {
  //   digitalWrite(EN_A, 0);
  //   ledcWrite(IN1_A, a.set);
  //   ledcWrite(IN2_A, 0);
  // }
  // else {
  //   digitalWrite(EN_A, 1);
  //   ledcWrite(IN1_A, 0);
  //   ledcWrite(IN2_A, 0);
  // }


  //GUI Prints
  // if (millis() >= lastPrint + PRINT_INTERVAL) {
  //   lastPrint = millis();
  //   Serial.write(2);
  //   Serial.write((byte*)&a, sizeof(a));   // Hopefully this has a null termination?
  // }

  // Human Readable Prints
  currMillis = millis();
  if (currMillis >= lastPrint + PRINT_INTERVAL) {
    
    Serial.print("Curr: "); Serial.print(tempCurrPos);
    Serial.print("\tSet: "); Serial.print(a.pos_set);
    Serial.print("\tError: "); Serial.print(a.error);
    Serial.print("\tMotSpd:"); Serial.print(a.set);
    Serial.print("\txi: "); Serial.print(a.xi);
    Serial.print("\tkp: "); Serial.print(a.kp);
    Serial.print("\tki: "); Serial.print(a.ki);
    Serial.print("\tkd: "); Serial.print(a.kd);
    // Serial.print("\tLastTime (ms): "); Serial.print(millis()-encoderLastTime);
    Serial.print("\tPWM_MAX: "); Serial.print(PWM_MAX);
    Serial.print("\tprintTime: "); Serial.println(currMillis-lastPrint);
    lastPrint = millis();
  }
}




float calcPosError(float set, float measured) {
  float error = set - measured;
  return error;
}



void setMaxSpeed(float input, AxisParams *axis) {
    // Check if it is within limits
    axis->maxSpeed = input;
    
    char key[20];
    strcpy(key, KEY_MAX_SPEED);
    strcat(key, &axis->letter);
    
    axis->trajectory->setMaxVel(axis->maxSpeed);
    preferences.putFloat(key,axis->maxSpeed);
    Serial.println(axis->trajectory->getMaxVel());
}

void setToCurrPos(AxisParams *axis) {
  axis->pos_set = axis->pos_curr;
  axis->error = 0;
  axis->prevError = 0;
  axis->trajectory->setTargetPos(axis->pos_set);
  axis->trajectory->setPos(axis->pos_set);
  Serial.println(a.pos_set);
}


// Not really used in the same way anymore
void getCurrAngle() {
    setAngle = currAngle;
    // a.xi = 0;
    // a.trajectory->setTargetPos(a.setAngle);
    // a.trajectory->setPos(a.setAngle);
    // Serial.println(a.setAngle*RAD_TO_DEG);
}

void setPosTarget(int input, AxisParams *axis) {
  axis->pos_set = input;
  Serial.println("SET POS");
  Serial.println(currMillis-lastPrint);
}


//Not used rn
void setAngleTarget(float input) {
    // Check within limits
    if ( input < ANGLE_LIM_CW || input > ANGLE_LIM_CCW) {
      Serial.println("***ANGLE OUT OF RANGE***");
    }
    else {
      float set = (input*DEG_TO_RAD);
      a.trajectory->setTargetPos(set);
      Serial.print("ANGLE = ");
      Serial.println(a.trajectory->getTarget()*RAD_TO_DEG);
    }
}

void setAccel(float input, AxisParams *axis) {
    axis->maxAccel = input;
    axis->trajectory->setAcc(axis->maxAccel);
    axis->trajectory->setDec(axis->maxAccel);
    char key[20];
    strcpy(key, KEY_MAX_ACCEL);
    strcat(key, &axis->letter);
    preferences.putFloat(key,axis->maxAccel);
    Serial.println(axis->trajectory->getAcc());
}

void setDeadzone(float input, AxisParams *axis) {
    axis->deadzone = input*DEG_TO_RAD;
    char key[20];
    strcpy(key, KEY_DEADZONE);
    strcat(key, &axis->letter);
    preferences.putFloat(key,axis->deadzone);
    Serial.println(axis->deadzone);
}

void setDecay(float input, AxisParams *axis) {
    axis->iDecay = input;
    char key[20];
    strcpy(key, KEY_I_DECAY);
    strcat(key, &axis->letter);
    preferences.putFloat(key,axis->iDecay);
    Serial.println(axis->iDecay);
}

void setKp(float input, AxisParams *axis) {
    axis->kp = input;
    char key[20];
    strcpy(key, KEY_KP);
    strcat(key, &axis->letter);
    preferences.putFloat(key,axis->kp);
    Serial.println(axis->kp);
}

void setKi(float input, AxisParams *axis) {
    axis->ki = input;
    axis->xi = 0;
    char key[20];
    strcpy(key, KEY_KI);
    strcat(key, &axis->letter);
    preferences.putFloat(key,axis->ki);
    Serial.println(axis->ki);
}

void setKd(float input, AxisParams *axis) {
    axis->kd = input;
    char key[20];
    strcpy(key, KEY_KD);
    strcat(key, &axis->letter);
    preferences.putFloat(key,axis->kd);
    Serial.println(axis->kd);
}

// Looks at cmd[] and cmdVal
void processCommand() {

  if(strcmp(cmd, "kp") == 0) {           // Set kp
    setKp(cmdVal, &a);
  }
  else if(strcmp(cmd, "ki") == 0) {           // Set ki
    setKi(cmdVal, &a);
  }
  else if(strcmp(cmd, "kd") == 0) {           // Set kd
    setKd(cmdVal, &a);
  }
  else if(strcmp(cmd, "set") == 0) {           // Set position (encoder values)
    Serial.println((int)cmdVal);
    setPosTarget((int)cmdVal, &a);
    // a.set = (int)cmdVal;  
  }
  else if( (strcmp(cmd, "speed") == 0) || (strcmp(cmd, "spd") == 0)) {           // Set max speed a (rad/sec)
    setMaxSpeed(cmdVal, &a);
  }
  else if( (strcmp(cmd, "accel") == 0) || (strcmp(cmd, "acc") == 0)) {           // Set max accel a (rad/sec^2)
    setAccel(cmdVal, &a);
  }
  else if( (strcmp(cmd, "deadzone") == 0) || (strcmp(cmd, "dead") == 0)) {           // Set angle deadzone
    setDeadzone(cmdVal, &a);
  }
  else if ( (strcmp(cmd, "decay") == 0) ) {
    setDecay(cmdVal, &a);
  }
  else if(strcmp(cmd, "stop") == 0) {         // Stop
    a.stopped = true;
    Serial.println("------STOPPED-------");
  }
  else if(strcmp(cmd, "start") == 0) {        // Start
    a.stopped = false;
    Serial.println("=======    START    ========");
  }
  else if(strcmp(cmd, "curr") == 0) {        // Get current angle - sets to setpoint
    setToCurrPos(&a);
  }
  else {
    Serial.println("UNKNOWN COMMAND");
  }
}


void moveMotor(AxisParams *axis, uint8_t en, uint8_t in1, uint8_t in2, int16_t minPWM) {
  if (axis->stopped) {
    digitalWrite(en, 1);    //Disables motor
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
    return;
  }
  if ( ((int)axis->set == 0) || ( (-minPWM < axis->set) && (axis->set < minPWM) )) {
    // Set motor speed to 0 because it is in the control dead zone
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
    digitalWrite(en, 1);
    axis->motorOff = true;
    return;
  }
  if ( axis->set < 0 ) {
    ledcWrite(in1, (uint16_t)abs(axis->set));
    ledcWrite(in2, 0);
    digitalWrite(en, 0);
    axis->motorOff = false;
    return;
  }
  if ( axis->set > 0 ) {
    ledcWrite(in1, 0);
    ledcWrite(in2, (uint16_t)abs(axis->set));
    digitalWrite(en, 0);
    axis->motorOff = false;
    return;
  }
}

void limit_pwm(float* signedSpeed, long max) {
  if (*signedSpeed < -max) {
    *signedSpeed = -max;
    return;
  }
  if (*signedSpeed > max) {
    *signedSpeed = max;
    return;
  }
}

void satLimit(float* x, int8_t* sat, long max) {
  if (-max < *x && *x < max) {
    *sat = 0;
    return; 
  }
  if (*x < -max) {
    *x = -max;
    *sat = -1;
    return;
  }
  if (*x > max) {
    *x = max;
    *sat = 1;
    return;
  }
}


// Angle in radians - why bother scaling it? would be slower for no reason
float calculateAngle(float accX, float accY) {
  float rads = atan2f(accY , accX);
  return rads;
}


// lol the following functions are basically the same thing -> could just calculate Angle and then call angleWrap....
float angleWrap(float rad) {
  while (rad > PI) {
    rad -= 2.0*PI;
  }
  while (rad < -PI) {
    rad += 2.0*PI;
  }
  return rad;
}

// maybe could just call above function, but maybe that is slower 
float calcAngleError(float set, float measured, float deadzone) {
  float error = set - measured;
  if (error > PI) {
    error -= (PI+PI);
  }
  else if (error < -PI) {
    error += (PI+PI);
  }
  else if (-deadzone < error && error < deadzone) {
    error = 0;     //Thanks Tzion for this tip!
    //xi = 0;
  }
  return error;
}


////////////////////////  Serial
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void parseCommand() {      // split the data into its parts
    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(tempChars, DELIM);      // get the first part - the string
    if (strtokIndx != NULL) {
      strcpy(cmd, strtokIndx);                  // copy it to cmd
      strtokIndx = strtok(NULL, DELIM);         // this continues where the previous call left off
      if (strtokIndx == NULL) {
        cmdVal = NAN;
      }
      else {
        cmdVal = (float)atof(strtokIndx);
      }
    }
}


/////////////////////////////////////////////////////


void accel_test() {
  
  Serial.println("Adafruit LSM6DS3TR-C test!");
  if (!lsm6ds3trc.begin_I2C()) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS)) {
    // if (!lsm6ds3trc.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
    Serial.println("Failed to find LSM6DS3TR-C chip");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DS3TR-C Found!");

  lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G); 
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds3trc.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  lsm6ds3trc.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
  Serial.print("Gyro range set to: ");
  switch (lsm6ds3trc.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DS33
  }
  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_104_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds3trc.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_104_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds3trc.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
  lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2
}







