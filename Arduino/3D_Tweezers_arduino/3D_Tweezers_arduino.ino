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
float action[17];  // Incoming command array (Bx, By, Bz, alpha, gamma, freq, psi, grad, equal, acoustic)


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
float amplitude;
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

// Components of the gradient output magnetic field
float C1;
float C2;
float C3;
float C4;
float C5;
float C6;

float MaxVal;

// helper: max over a float array
float findMax(const float *arr, int n) {
  float maxV = arr[0];
  for (int i = 1; i < n; i++) {
    if (arr[i] > maxV) maxV = arr[i];
  }
  return maxV;
}

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


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void loop()
{
  digitalWrite(49, HIGH);  // for testing loop time and frequency of the code. will measure the time to to go low on this pin with an oscilloscope. 
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
    amplitude         = action[0];
    Bx_uniform        = action[1];
    By_uniform        = action[2];
    Bz_uniform        = action[3];
    alpha             = action[4];
    gamma             = action[5];
    rolling_frequency = action[6]; 
    psi               = action[7]; 
    gradient_status   = action[8];
    equal_field_status= action[9];
    acoustic_frequency= action[10];

    coil1_manual      = action[11];
    coil2_manual      = action[12];
    coil3_manual      = action[13];
    coil4_manual      = action[14];
    coil5_manual      = action[15];
    coil6_manual      = action[16];

    
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
        Bx_roll = amplitude * (-(cos(alpha)*cos(gamma)*cos(omega*t)) + ( sin(alpha)*sin(omega*t) ));
        By_roll = amplitude * (-(sin(alpha)*cos(gamma)*cos(omega*t)) - ( cos(alpha)*sin(omega*t) ));
        Bz_roll = amplitude * (   sin(gamma)*cos(omega*t));

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
    //map Bz_final, By_final, and Bx_final to each of the six tweezers. Let's assume that the three top coils correspond to set1-3 and the three bottom are set4-6:
//if we have a Bx_final we need to map that onto two of the top and two of the bottom coils, let's assume it's set1-2 and set4-5. Let's also assume that set1 
// tweezer is along the -y axis and set2 and set3 are 30 degrees above the x and -x axes, respectively. We'll assume that the bottom three tweezers have the 
// exact same arrangement. We'll label the signal going to each of the 6 as C1, C2, C3, C4, C5, C6.

C1 = 0;
C2 = 0;
C3 = 0;
C4 = 0;
C5 = 0;
C6 = 0;

if (coil1_manual != 0 || coil2_manual != 0 || coil3_manual != 0 || 
    coil4_manual != 0 || coil5_manual != 0 || coil6_manual != 0) {
          // Executes if ANY coil variable is non-zero
          set1(coil1_manual);
          set2(coil2_manual);
          set3(coil3_manual);
          set4(coil4_manual);
          set5(coil5_manual);
          set6(coil6_manual);
      //Serial.print("output manual single"); 
    }


else
{
        // if gradient status = 0: output uniform field
          if (gradient_status == 0){

            const float k = 1.0;

            const float Mcam[6][3] = {
              {0.0,            sqrt(2.0)/3.0,  sqrt(2.0)/6.0},
              {sqrt(6.0)/6.0, -sqrt(2.0)/6.0,  sqrt(2.0)/6.0},
              {-sqrt(6.0)/6.0,-sqrt(2.0)/6.0,  sqrt(2.0)/6.0},
              {0.0,            sqrt(2.0)/3.0,   -sqrt(2.0)/6.0},
              {sqrt(6.0)/6.0, -sqrt(2.0)/6.0,   -sqrt(2.0)/6.0},
              {-sqrt(6.0)/6.0,-sqrt(2.0)/6.0,   -sqrt(2.0)/6.0}
            };

            float B[3] = {Bx, By, Bz};
            float I[6];

            for (int i = 0; i < 6; i++)
            {
              I[i] = 0;

              for (int j = 0; j < 3; j++)
              {
                I[i] += Mcam[i][j] * B[j];
              }

              I[i] /= k;
            }

            C1 = I[0];
            C2 = I[1];
            C3 = I[2];
            C4 = I[3];
            C5 = I[4];
            C6 = I[5];

            float vals[6] = { abs(C1), abs(C2), abs(C3), abs(C4), abs(C5), abs(C6) };            
            MaxVal = findMax(vals, 6);

            if (MaxVal>1){
              C1 = C1/MaxVal;
              C2 = C2/MaxVal;
              C3 = C3/MaxVal;
              C4 = C4/MaxVal;
              C5 = C5/MaxVal;
              C6 = C6/MaxVal;
            }

            set1(C1);
            set2(C2);
            set3(C3);
            set4(C4);
            set5(C5);
            set6(C6);

            // set1(C[0]);
            // set2(C[1]);
            // set3(C[2]);
            // set4(C[3]);
            // set5(C[4]);
            // set6(C[5]);
        
          }

          // if gradient status = 1 output gradient field
          else {

              if (By_final <= 0 && Bx_final > 0){//hack change of all signs and <
                C3 = C3 + By_final;
                C2 = C2 - By_final;
                C6 = C6 + By_final;
                C5 = C5 - By_final;

                C2 = C2 + Bx_final;
                C1 = C1 - 0.5*Bx_final;
                C5 = C5 + Bx_final;
                C4 = C4 - 0.5*Bx_final;
              }

            if (By_final > 0 && Bx_final >= 0){//y>0 x>0
                C1 = C1 - By_final;
                C4 = C4 - By_final;

                C2 = C2 + Bx_final;
                C1 = C1 - 0.5*Bx_final;
                C5 = C5 + Bx_final;
                C4 = C4 - 0.5*Bx_final;
              }

            if (By_final <= 0 && Bx_final < 0){
                C3 = C3 - By_final;
                C2 = C2 + By_final;
                C6 = C6 - By_final;
                C5 = C5 + By_final;

                C3 = C3 - Bx_final;
                C1 = C1 + 0.5*Bx_final;
                C6 = C6 - Bx_final;
                C4 = C4 + 0.5*Bx_final;
              }


            if (By_final > 0 && Bx_final <= 0){
                C1 = C1 - By_final;
                C4 = C4 - By_final;

                C3 = C3 - Bx_final;
                C1 = C1 + 0.5*Bx_final;
                C6 = C6 - Bx_final;
                C4 = C4 + 0.5*Bx_final;
              }


            if (Bz_final > 0){
                C1 = C1 + Bz_final;
                C2 = C2 + Bz_final;
                C3 = C3 + Bz_final;
              }

            if (Bz_final < 0){
                C4 = C4 - Bz_final;
                C5 = C5 - Bz_final;
                C6 = C6 - Bz_final;
              }

            set1(C1);
            set2(C2);
            set3(C3);
            set4(C4);
            set5(C5); 
            set6(C6);
// Tried using matrix code but it's not workig for some reason:
//             const float k = 1.0;

//             const float Mcam[6][3] = {
//               {0.0,            sqrt(2.0)/3.0,  sqrt(2.0)/6.0},
//               {sqrt(6.0)/6.0, -sqrt(2.0)/6.0,  sqrt(2.0)/6.0},
//               {-sqrt(6.0)/6.0,-sqrt(2.0)/6.0,  sqrt(2.0)/6.0},
//               {0.0,            sqrt(2.0)/3.0,   -sqrt(2.0)/6.0},
//               {sqrt(6.0)/6.0, -sqrt(2.0)/6.0,   -sqrt(2.0)/6.0},
//               {-sqrt(6.0)/6.0,-sqrt(2.0)/6.0,   -sqrt(2.0)/6.0}
//             };

//             //each row is the vector that points from the origin to the coil

//             float B[3] = {Bx, By, Bz};
//             float I[6];

//             for (int i = 0; i < 6; i++)
//             {
//               I[i] = 0;

//               for (int j = 0; j < 3; j++)
//               {
//                 I[i] += Mcam[i][j] * B[j];
//               }

//               I[i] /= k;
//             }
// // if the coil is in the direction of the field, keep it on, otherwise zero it. This should be the only difference between the gradient and non-gradient fields
//             for (int i = 0; i < 6; i++)
//             {
//                 float alignment =
//                     Mcam[i][0]*Bx +
//                     Mcam[i][1]*By +
//                     Mcam[i][2]*Bz;

//                 if (alignment < 0)
//                     I[i] = 0;
//             }

//             C1 = I[0];
//             C2 = I[1];
//             C3 = I[2];
//             C4 = I[3];
//             C5 = I[4];
//             C6 = I[5];

//             // Normalize (makes sure max current is applied to some coil to optimize strength):

//             float vals[6] = { abs(C1), abs(C2), abs(C3), abs(C4), abs(C5), abs(C6) };            
//             MaxVal = findMax(vals, 6);

//             C1 = C1/MaxVal;
//             C2 = C2/MaxVal;
//             C3 = C3/MaxVal;
//             C4 = C4/MaxVal;
//             C5 = C5/MaxVal;
//             C6 = C6/MaxVal;


//             set1(C1);
//             set2(-C2);
//             set3(C3);
//             set4(C4);
//             set5(C5); 
//             set6(C6);


          


           
          }
    }

      // set1(0);
      // set2(0);
      // set3(0);
      // set4(0);
      // set5(0);
      // set6(0);

 
  
    // ================================================================
    // READ AND SEND CURRENT DATA BACK TO PYTHON (ACS712 5A sensors)
    // Sent as a SerialTransfer struct (float × 6)
    // throttle the transfer rate so it doesnt overflow python. python can only read the data so fast. must be slower than the pyqt5 timer refresh rate
    // also realizing there is no point reading the current data every loop if im not sending it to python AKA using it. its too slow otherwise
    // ================================================================
    
    if (millis() - lastSend >= 20) {  // 20 ms -> 50 Hz
        lastSend = millis();

        // Multiply each reading by dirN to recover signed current direction
        Current_Data.Coil1_current = readCurrentFast(A0) * dir1;
        Current_Data.Coil2_current = readCurrentFast(A1) * dir2;
        Current_Data.Coil3_current = readCurrentFast(A2) * dir3;
        Current_Data.Coil4_current = readCurrentFast(A3) * dir4;
        Current_Data.Coil5_current = readCurrentFast(A4) * dir5;
        Current_Data.Coil6_current = readCurrentFast(A5) * dir6;


    
        uint16_t sendSize = 0;
        sendSize = myTransfer.txObj(Current_Data, sendSize);
        myTransfer.sendData(sendSize);
        //Serial.print("sent to python"); Serial.print(" ");
}

digitalWrite(49, LOW);  // for testing loop time and frequency of the code. will measure the time between Low and High on this pin with an oscilloscope. 
}
