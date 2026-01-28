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
float action[10];  // Incoming command array (Bx, By, Bz, alpha, gamma, freq, psi, grad, equal, acoustic)


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

int phase = 0;   // Phase accumulator for the AD9850 (unused but required by DDS library)


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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
    // ================================================================
    // RECEIVE DATA FROM PYTHON OVER SERIALTRANSFER
    // ================================================================
    if (myTransfer.available()) {
        uint16_t message = 0;

        // Read incoming float array (10 floats) from Python
        message = myTransfer.rxObj(action, message);
    }

    // ================================================================
    // MAP RECEIVED ACTIONS INTO NAMED VARIABLES
    // (These correspond to magnetic field instructions + settings)
    // ================================================================
    Bx_uniform        = action[0];
    By_uniform        = action[1];
    Bz_uniform        = action[2];
    alpha             = action[3];
    gamma             = action[4];
    rolling_frequency = action[5]; 
    psi               = action[6]; 
    gradient_status   = action[7];
    equal_field_status= action[8];
    acoustic_frequency= action[9];

    
    // ================================================================
    // ACOUSTIC DDS MODULE CONTROL
    // Turn acoustic output ON if freq > 0, OFF otherwise
    // DDS.setfreq() only called when switching on
    // ================================================================
    if (acoustic_frequency > 0) {
        if (!acoustic_on)
            DDS.setfreq(acoustic_frequency, phase); // Set freq once on activation
        acoustic_on = true;
    } 
    else {
        if (acoustic_on)
            DDS.down();   // powers down acoustic DDS when stopped
        acoustic_on = false;
    }


    // ================================================================
    // CONVERT ROLLING FREQUENCY TO ANGULAR VELOCITY
    // ω = 2πf
    // ================================================================
    omega = 2 * PI * rolling_frequency;

    // System time in seconds (micros returns µs)
    t = micros() / 1e6;


    // ================================================================
    // ROTATING MAGNETIC FIELD COMPUTATION
    // If ω = 0 → no rotating field
    // Otherwise compute theoretical rotating field vector
    // Using your 7/1/25 derivation
    // ================================================================
    if (omega == 0){
        Bx_roll = 0;
        By_roll = 0;
        Bz_roll = 0;
    }
    else {
        // Base rotating field components
        Bx_roll = -(cos(alpha)*cos(gamma)*cos(omega*t)) + ( sin(alpha)*sin(omega*t) );
        By_roll = -(sin(alpha)*cos(gamma)*cos(omega*t)) - ( cos(alpha)*sin(omega*t) );
        Bz_roll =    sin(gamma)*cos(omega*t);

        // ============================================================
        // Perpendicular component (if ψ != 90°)
        // Adds a static bias perpendicular to the rotation plane
        // ============================================================
        if (psi < PI/2){
            c = 1/tan(psi);
            BxPer = c * cos(alpha) * sin(gamma);
            ByPer = tan(alpha) * BxPer;
            BzPer = BxPer * (1/cos(alpha)) * (1/tan(gamma));
        }
        else {
            // ψ = 90° → perpendicular field undefined → zero it
            c = 0;
            BxPer = 0;
            ByPer = 0;
            BzPer = 0;
        }

        // ============================================================
        // Superimpose rotating field + perpendicular field
        // Normalize such that added perpendicular component blends correctly
        // ============================================================
        Bx_roll = (Bx_roll + BxPer) / (1 + c);
        By_roll = (By_roll + ByPer) / (1 + c);
        Bz_roll = (Bz_roll + BzPer) / (1 + c);
    }


    // ================================================================
    // FINAL SUPERIMPOSED MAGNETIC FIELD (UNIFORM + ROLLING)
    // ================================================================
    Bx = (Bx_uniform + Bx_roll);
    By = (By_uniform + By_roll);
    Bz = (Bz_uniform + Bz_roll);


    // ================================================================
    // PREVENT DIVIDE-BY-ZERO
    // If total B-field is zero → skip normalization
    // ================================================================
    if (Bx == 0 && By == 0 && Bz == 0){
        Bx_final = 0;
        By_final = 0;
        Bz_final = 0;
    }
    else {
        // ============================================================
        // Choose magnitude = max of uniform-field magnitude and rotating-field magnitude
        // This keeps the amplitude consistent if one is zero
        // ============================================================
        magnitude = max(
            sqrt(Bx_uniform*Bx_uniform + By_uniform*By_uniform + Bz_uniform*Bz_uniform),
            sqrt(Bx_roll*Bx_roll + By_roll*By_roll + Bz_roll*Bz_roll)
        );

        // Normalize vector to correct magnitude
        float denom = sqrt(Bx*Bx + By*By + Bz*Bz);
        Bx_final = magnitude * (Bx / denom);
        By_final = magnitude * (By / denom);
        Bz_final = magnitude * (Bz / denom);

        // ============================================================
        // OPTIONAL: equal-field scaling for unequal coil geometry
        // (Helmholtz correction factor)
        // ============================================================
        if (equal_field_status == 1){
            Bx_final = Bx_final;
            By_final = By_final * 0.6;
            Bz_final = Bz_final * 0.3;
        }
    }


    // ================================================================
    // GRADIENT MODE
    // If gradient_status != 0 → use 2-coil differential mode
    // Uniform mode uses opposing pairs in Helmholtz configuration
    // ================================================================
    if (gradient_status != 0){

        // ---------------- Y GRADIENT ----------------
        if (By_final > 0){
            set1(By_final);
        }
        else if (By_final < 0){
            set3(By_final);
        }
        else {
            set1(0);
            set3(0);
        }

        // ---------------- X GRADIENT ----------------
        if (Bx_final > 0){
            set2(Bx_final);
        }
        else if (Bx_final < 0){
            set4(Bx_final);
        }
        else {
            set2(0);
            set4(0);
        }

        // ---------------- Z GRADIENT ----------------
        if (Bz_final > 0){
            set5(Bz_final);
        }
        else if (Bz_final < 0){
            set6(Bz_final);
        }
        else {
            set5(0);
            set6(0);
        }
    }

    // ================================================================
    // UNIFORM MODE (standard Helmholtz coil driving)
    // Opposing pairs get equal & opposite duty signs
    // ================================================================
    else {
        set1(By_final);
        set2(Bx_final);
        set3(-By_final);
        set4(-Bx_final);
        set5(Bz_final);
        set6(-Bz_final);
    }

    // ================================================================
    // READ ALL SIX COIL CURRENTS (ACS712 5A sensors)
    // Multiply each reading by dirN to recover signed current direction
    // ================================================================
    Current_Data.Coil1_current = readCurrentFast(A0) * dir1;
    Current_Data.Coil2_current = readCurrentFast(A1) * dir2;
    Current_Data.Coil3_current = readCurrentFast(A2) * dir3;
    Current_Data.Coil4_current = readCurrentFast(A3) * dir4;
    Current_Data.Coil5_current = readCurrentFast(A4) * dir5;
    Current_Data.Coil6_current = readCurrentFast(A5) * dir6;



    // ================================================================
    // SEND CURRENT DATA BACK TO PYTHON
    // Sent as a SerialTransfer struct (float × 6)
    //throttle the transfer rate so it doesnt overflow python. python can only read the data so fast. must be slower than the pyqt5 timer refresh rate
    // ================================================================
    static uint32_t lastSend = 0;
    if (millis() - lastSend >= 20) {  // 20 ms -> 50 Hz
        lastSend = millis();
    
        uint16_t sendSize = 0;
        sendSize = myTransfer.txObj(Current_Data, sendSize);
        myTransfer.sendData(sendSize);
        //Serial.print("sent to python"); Serial.print(" ");
}
}
