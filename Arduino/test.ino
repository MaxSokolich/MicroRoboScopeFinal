
// ================================================================
//  INCLUDES & GLOBAL DEFINITIONS
//  These libraries handle:
//   - AD9850 DDS signal generator (acoustic drive)
//   - I2C (unused but kept for compatibility)
//   - SerialTransfer (high-speed binary packets to/from Python)
// ================================================================
#include <AD9850.h>
#include <Wire.h>
#include "SerialTransfer.h"

SerialTransfer myTransfer;  // Object for sending/receiving binary packets


// ================================================================
//  STORAGE FOR INCOMING PYTHON COMMANDS (10 floats total)
//  Python sends these at ~20–30 Hz using PySerialTransfer.
// ================================================================
float action[16];  // Incoming command array (Bx, By, Bz, alpha, gamma, freq, psi, grad, equal, acoustic)


// Define PI manually (more precision than built-in)
#define PI 3.1415926535897932384626433832795


// ================================================================
//  INDIVIDUAL COMMAND VARIABLES (assigned from 'action[]')
//  These represent:
//   - uniform magnetic field components (Bx, By, Bz)
//   - rolling field orientation (alpha, gamma)
//   - rolling field frequency
//   - perpendicular field tilt (psi)
//   - gradient mode flag
//   - equal-field mode flag
//   - acoustic frequency for AD9850 module
//   - manual set actions
// ================================================================




float Bx;
float By;
float Bz;
float alpha;
float gamma;
float rolling_frequency;
float psi;
float gradient_status;
float equal_field_status;
float acoustic_frequency;

float coil1_manual;
float coil2_manual;
float coil3_manual;
float coil4_manual;
float coil5_manual;
float coil6_manual;


int phase = 0;   // Phase accumulator for the AD9850 (unused but required by DDS library)
static uint32_t lastSend = 0;  // initilize lastsend variable for throttling send rate back to python

// ================================================================
//  FIELD COMPONENT STORAGE (Intermediate computations)
// ================================================================

// Components of the rotating magnetic field
float Bx_roll;
float By_roll;
float Bz_roll;

// Uniform fields (directly given by Python)
float Bx_uniform;
float By_uniform;
float Bz_uniform;

// Perpendicular field components (depends on psi)
float BxPer;
float ByPer;
float BzPer;
float c;          // geometric scaling for perpendicular field
float magnitude;  // normalization term


// ================================================================
//  TIME AND ROTATION VARIABLES
// ================================================================
float tim;     // (unused) — kept for compatibility
float t;       // time in seconds from micros()
float omega;   // angular frequency = 2πf


// These hold the final field (superposition + normalization)
float Bx_final;
float By_final;
float Bz_final;

bool acoustic_on = false;   // Track whether acoustic module is active


// ================================================================
//  DATA STRUCT SENT BACK TO PYTHON
//  Each coil reports sensed current (signed, using dirN)
// ================================================================
struct current_data {
  float Coil1_current;  // +Y coil
  float Coil2_current;  // +X coil
  float Coil3_current;  // -Y coil
  float Coil4_current;  // -X coil
  float Coil5_current;  // +Z coil
  float Coil6_current;  // -Z coil
};

current_data Current_Data;  // Instance used for sending to Python


// ================================================================
//  FAST ACS712 CURRENT SENSOR READING
//  Reads the analog value quickly and returns current in Amps.
//    pin        = ADC pin
//
//    ACS712 5A = 185 mV/A sensativity
//    samples    = number of ADC samples to average
// ================================================================
float readCurrentFast(uint8_t pin, int samples = 5) {

    long sum = 0;  
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);   // read raw ADC value
    }

    float raw = float(sum)/samples;  // average ADC reading
    float voltage = (raw * 4.93) / 1023.0; // convert to volts relative to mid (4.93 V measure with multimeter)
    float current = (voltage - 2.52) / 0.185;  // convert volts to Amps (adjusted with offset = 2.52 V)
    return current;
}


// ================================================================
//  DIRECTION FLAGS FOR EACH COIL
//  These turn ACS712 readings into signed current values.
// ================================================================
int dir1 = 0;
int dir2 = 0;
int dir3 = 0;
int dir4 = 0;
int dir5 = 0;
int dir6 = 0;




// ================================================================
//  COIL PIN DEFINITIONS (PWM + ENABLES)
//  Each coil has:
//    - PWMR  (right H-bridge side)
//    - PWML  (left H-bridge side)
//    - ENR   enable right
//    - ENL   enable left
//
//  The names correspond to spatial coil placement (+X, -X, +Y, -Y, +Z, -Z)
// ================================================================

// -------- Coil 1 : +Y (Brown) --------
const int Coil1_PWMR = 2;
const int Coil1_PWML = 3;
const int Coil1_ENR = 26;
const int Coil1_ENL = 27;

// -------- Coil 2 : +X (Purple) --------
const int Coil2_PWMR = 44;
const int Coil2_PWML = 45;
const int Coil2_ENR = 24;
const int Coil2_ENL = 25;

// -------- Coil 3 : -Y (Green) --------
const int Coil3_PWMR = 6;
const int Coil3_PWML = 7;
const int Coil3_ENR = 22;
const int Coil3_ENL = 23;

// -------- Coil 4 : -X (Blue) --------
const int Coil4_PWMR = 10;
const int Coil4_PWML = 9;
const int Coil4_ENR = 32;
const int Coil4_ENL = 33;

// -------- Coil 5 : +Z (Yellow) --------
const int Coil5_PWMR = 12;
const int Coil5_PWML = 11;
const int Coil5_ENR = 30;
const int Coil5_ENL = 31;

// -------- Coil 6 : -Z (Orange) --------
const int Coil6_PWMR = 8;
const int Coil6_PWML = 46;
const int Coil6_ENR = 28;
const int Coil6_ENL = 29;


// ================================================================
//  AD9850 DDS ACOUSTIC DRIVER PIN DEFINITIONS
// ================================================================
const int W_CLK_PIN = 34;
const int FQ_UD_PIN = 36;
const int DATA_PIN  = 38;
const int RESET_PIN = 40;




  
void setup()
{
  // ================================================================
  //  SET PWM FREQUENCY FOR ALL TIMERS
  //  ---------------------------------------------------------------
  //  Setting prescaler = 1 (0x01) increases PWM frequency to ~31 kHz.
  //  This produces smoother current control for electromagnets and
  //  reduces audible coil noise.
  //
  //  cli() disables interrupts while updating timer registers.
  // ================================================================
  cli();
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;  // Timer1  -> pins 11,12
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;  // Timer2  -> pins 9,10
  TCCR3B = (TCCR3B & 0b11111000) | 0x01;  // Timer3  -> pins 2,3,5
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;  // Timer4  -> pins 6,7,8
  TCCR5B = (TCCR5B & 0b11111000) | 0x01;  // Timer5  -> pins 44,45,46
  sei();  // re-enable interrupts


  // ================================================================
  //  SERIAL & PYTHON COMMUNICATION
  //  ---------------------------------------------------------------
  //  High-speed 500000 baud minimizes latency and maximizes throughput.
  //  SerialTransfer is initialized on this Serial instance.
  // ================================================================
  Serial.begin(500000);
  myTransfer.begin(Serial);


  // ================================================================
  //  INITIALIZE AD9850 ACOUSTIC DRIVER
  //  ---------------------------------------------------------------
  //  DDS.begin configures pins.
  //  DDS.calibrate is used for crystal tuning.
  // ================================================================
  DDS.begin(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);
  DDS.calibrate(124999500);  // Factory recommended calibration value



  // ================================================================
  //  CONFIGURE COIL OUTPUT PINS
  //  ---------------------------------------------------------------
  //  Each coil has two PWM pins (PWMR/PWML) controlling H-bridge legs
  //  and two enable lines (ENR/ENL).
  // ================================================================

  pinMode(49, OUTPUT); // used to measure the loop frequency using osscislscope


  // ---- Coil 1 (+Y) ----
  pinMode(Coil1_PWMR, OUTPUT);
  pinMode(Coil1_PWML, OUTPUT);
  pinMode(Coil1_ENR, OUTPUT);
  pinMode(Coil1_ENL, OUTPUT);

  // ---- Coil 2 (+X) ----
  pinMode(Coil2_PWMR, OUTPUT);
  pinMode(Coil2_PWML, OUTPUT);
  pinMode(Coil2_ENR, OUTPUT);
  pinMode(Coil2_ENL, OUTPUT);

  // ---- Coil 3 (-Y) ----
  pinMode(Coil3_PWMR, OUTPUT);
  pinMode(Coil3_PWML, OUTPUT);
  pinMode(Coil3_ENR, OUTPUT);
  pinMode(Coil3_ENL, OUTPUT);

  // ---- Coil 4 (-X) ----
  pinMode(Coil4_PWMR, OUTPUT);
  pinMode(Coil4_PWML, OUTPUT);
  pinMode(Coil4_ENR, OUTPUT);
  pinMode(Coil4_ENL, OUTPUT);

  // ---- Coil 5 (+Z) ----
  pinMode(Coil5_PWMR, OUTPUT);
  pinMode(Coil5_PWML, OUTPUT);
  pinMode(Coil5_ENR, OUTPUT);
  pinMode(Coil5_ENL, OUTPUT);

  // ---- Coil 6 (-Z) ----
  pinMode(Coil6_PWMR, OUTPUT);
  pinMode(Coil6_PWML, OUTPUT);
  pinMode(Coil6_ENR, OUTPUT);
  pinMode(Coil6_ENL, OUTPUT);


  // ================================================================
  //  ENABLE ALL COIL H-BRIDGES
  //  ---------------------------------------------------------------
  //  Setting ENR/ENL HIGH turns on the H-bridge power stage.
  //  PWM alone will control current direction & amplitude.
  // ================================================================
  digitalWrite(Coil1_ENR, HIGH);  
  digitalWrite(Coil1_ENL, HIGH);
  digitalWrite(Coil2_ENR, HIGH);
  digitalWrite(Coil2_ENL, HIGH);
  digitalWrite(Coil3_ENR, HIGH);
  digitalWrite(Coil3_ENL, HIGH);
  digitalWrite(Coil4_ENR, HIGH);
  digitalWrite(Coil4_ENL, HIGH);
  digitalWrite(Coil5_ENR, HIGH);
  digitalWrite(Coil5_ENL, HIGH);
  digitalWrite(Coil6_ENR, HIGH);
  digitalWrite(Coil6_ENL, HIGH);
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ================================================================
//  set()  — Control Coil PWM and direction
//  ---------------------------------------------------------------
//  DC is a normalized duty cycle input: range [-1, 1].
//     > 0  → PWMR active, PWML off  (positive current)
//     < 0  → PWML active, PWMR off  (negative current)
//     = 0  → both off, but dir is forced to +1 so ACS712 leakage
//            will be interpreted as positive instead of negative.
// ================================================================

void set1(float DC1){
  DC1 = constrain(DC1, -1.0, 1.0);
  if (DC1 > 0){
    dir1 = 1;
    analogWrite(Coil1_PWMR,abs(DC1)*255);
    analogWrite(Coil1_PWML,0);
    
    
  }
  else if (DC1 < 0){
    dir1 = -1;
    analogWrite(Coil1_PWMR,0);
    analogWrite(Coil1_PWML,abs(DC1)*255);
    
    
  }
  else {
    dir1 = 1;
    analogWrite(Coil1_PWMR,0);
    analogWrite(Coil1_PWML,0);
    
    
    
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set2(float DC2){
  DC2 = constrain(DC2, -1.0, 1.0);
  if (DC2 > 0){
    dir2 = 1;
    analogWrite(Coil2_PWMR,abs(DC2)*255);
    analogWrite(Coil2_PWML,0);
    
    
  }
  else if (DC2 < 0){
    dir2 = -1;
    analogWrite(Coil2_PWMR,0);
    analogWrite(Coil2_PWML,abs(DC2)*255);
    
    
  }
  else {
    dir2 = 1;
    analogWrite(Coil2_PWMR,0);
    analogWrite(Coil2_PWML,0);
    
    
  }
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set3(float DC3){
  DC3 = constrain(DC3, -1.0, 1.0);
  if (DC3 > 0){
    dir3 = 1;
    analogWrite(Coil3_PWMR,abs(DC3)*255);
    analogWrite(Coil3_PWML,0);
    
    
  }
  else if (DC3 < 0){
    dir3 = -1;
    analogWrite(Coil3_PWMR,0);
    analogWrite(Coil3_PWML,abs(DC3)*255);
    
    
  }
  else {
    dir3 = 1;
    analogWrite(Coil3_PWMR,0);
    analogWrite(Coil3_PWML,0);
    
    
    
  }

  

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set4(float DC4){
  DC4 = constrain(DC4, -1.0, 1.0);
  if (DC4 > 0){
    dir4 = 1;
    analogWrite(Coil4_PWMR,abs(DC4)*255);
    analogWrite(Coil4_PWML,0);
    

  }
  else if (DC4 < 0){
    dir4 = -1;
    analogWrite(Coil4_PWMR,0);
    analogWrite(Coil4_PWML,abs(DC4)*255);
    
    
  }
  else {
    dir4 = 1;
    analogWrite(Coil4_PWMR,0);
    analogWrite(Coil4_PWML,0);
    
    
   
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set5(float DC5){
  DC5 = constrain(DC5, -1.0, 1.0);
  if (DC5 > 0){
    dir5 = 1;
    analogWrite(Coil5_PWMR,abs(DC5)*255);
    analogWrite(Coil5_PWML,0);
    
    
  }
  else if (DC5 < 0){
    dir5 = -1;
    analogWrite(Coil5_PWMR,0);
    analogWrite(Coil5_PWML,abs(DC5)*255);
    
    
  }
  else {
    dir5 = 1;
    analogWrite(Coil5_PWMR,0);
    analogWrite(Coil5_PWML,0);
    

  }


 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void set6(float DC6){
  DC6 = constrain(DC6, -1.0, 1.0);
  if (DC6 > 0){
    dir6 = 1;
    analogWrite(Coil6_PWMR,abs(DC6)*255);
    analogWrite(Coil6_PWML,0);
    
    
  }
  else if (DC6 < 0){
    dir6 = -1;
    analogWrite(Coil6_PWMR,0);
    analogWrite(Coil6_PWML,abs(DC6)*255);
    
    
  }
  else {
    dir6 = 1;
    analogWrite(Coil6_PWMR,0);
    analogWrite(Coil6_PWML,0);
    
    

  }
 
}




void loop()
{
    // ===============================
    // RECEIVE DATA FROM PYTHON
    // ===============================
    if (myTransfer.available()) {
        uint16_t message = 0;
        message = myTransfer.rxObj(action, message);
    }

    float coil1PWM = action[0];
    float coil2PWM = action[1];
    float coil3PWM = action[2];
    float coil4PWM = action[3];
    float coil5PWM = action[4];
    float coil6PWM = action[5];

    float alpha = action[6];
    float gamma = action[7];
    float psi   = action[8];
    float freq  = action[9];

    // ===============================
    // MANUAL DIRECT COIL MODE
    // ===============================
    if (coil1PWM != 0 || coil2PWM != 0 || coil3PWM != 0 ||
        coil4PWM != 0 || coil5PWM != 0 || coil6PWM != 0)
    {
        set1(coil1PWM);
        set2(coil2PWM);
        set3(coil3PWM);
        set4(coil4PWM);
        set5(coil5PWM);
        set6(coil6PWM);
        return;
    }

    // ===============================
    // BUILD AXIS VECTOR n
    // ===============================
    float nx = cos(gamma) * cos(alpha);
    float ny = cos(gamma) * sin(alpha);
    float nz = sin(gamma);

    // ===============================
    // BUILD ORTHONORMAL BASIS (u, v, n)
    // ===============================
    float hx, hy, hz;

    if (abs(nz) < 0.9) {
        hx = 0; hy = 0; hz = 1;   // use Z axis
    } else {
        hx = 1; hy = 0; hz = 0;   // use X axis
    }

    // u = helper × n
    float ux = hy*nz - hz*ny;
    float uy = hz*nx - hx*nz;
    float uz = hx*ny - hy*nx;

    float umag = sqrt(ux*ux + uy*uy + uz*uz);
    ux /= umag;
    uy /= umag;
    uz /= umag;

    // v = n × u
    float vx = ny*uz - nz*uy;
    float vy = nz*ux - nx*uz;
    float vz = nx*uy - ny*ux;

    // ===============================
    // TIME + ROTATION
    // ===============================
    float Bx, By, Bz;

    if (freq == 0)
    {
        // static uniform field
        Bx = nx;
        By = ny;
        Bz = nz;
    }
    else
    {
        float t = micros() * 1e-6;
        float omega = 2 * PI * freq;

        float c = cos(omega * t);
        float s = sin(omega * t);

        Bx = cos(psi)*nx + sin(psi)*(c*ux + s*vx);
        By = cos(psi)*ny + sin(psi)*(c*uy + s*vy);
        Bz = cos(psi)*nz + sin(psi)*(c*uz + s*vz);
    }

    // ===============================
    // APPLY HELMHOLTZ PAIRS
    // ===============================
    set1(By);
    set3(-By);

    set2(Bx);
    set4(-Bx);

    set5(Bz);
    set6(-Bz);
}