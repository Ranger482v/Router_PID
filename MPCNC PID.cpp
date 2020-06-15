/*
 * PWM PID control using an optical sensor, a PWM enable AC triac power control, LCD panel....more info soon.
 * 
 * Details for now, https://www.v1engineering.com/forum/topic/hardware-needed-for-a-software-fix/
 * 
 * V1 Engineering
 */

/*
    // WIRING INSTRUCTIONS //
- Use PIN A2 as Input for Potentiometer
- Use PIN 11 as Input for Mode Button
- Use PIN 12 as Input for Potentiometer Read Button
*/

#include <Arduino.h>
#include <PID_v1.h>                                  // Arduino library PID Library by Brett Beauregard
#include <LiquidCrystal_I2C.h>                       // LCD Library
#include <Wire.h>



// Wiring: SDA pin is connected to A4 and SCL pin to A5.
// Connect to LCD via I2C, default address 0x27 (A0-A2 not jumpered)
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4); // Change to (0x27,16,2) for 16x2 LCD.
int lcdClear = 0;       // One Time LCD Clear to Refresh Screen

// PINS
#define POT_PWR A1          // Power for POT ( For Ease of Connecions on Perf Board )
#define POT_PIN A2          // Input for POT
#define POT_GND A3          // Ground for POT ( For Ease of Connecions on Perf Board )

const int PHOTO_PIN = 2;                             // Spindle IR Sensor input pin
const int PWM_PIN = 3;                               // Marlin PWMms Input pin
const int PHOTOPWR = 4;                              // Power for Photo Sensor ( For Ease of Connections on Perf Board )
const int ROUTER_PWM_OUTPIN = 9;                     // Triac output to router pin 10, 11 do not work
const int buttonGnd = 10;                            // Ground for Buttons ( For Ease of Connecions on Perf Board )
const int switchInput = 11;                          // Button for Input Switching
const int readPot = 12;                              // Button for Reading POT Input

int currentMillis;        // Values for Timing
int lastMillis;           // Values for Timing
int lastMillislcd1;       // Value for LCD Print Timing
int lastMillislcd2;       // Value for LCD PRint Timing
int counter = 0;          // Counter for Displaying Temp or Current Setpoint LCD
int counter2 = 0;         // Counter for RPM Reset to 0

int interruptMode = 0;    // Value to Change Marlin INPUT Interrupt Mode

int routerRPMSet = 0;         // Setpoint Sent through PID
int lastRouterRPMSet = 0;     // Store RPM Setpoint when switching from Marlin to POT
int tempRouterRPMSet = 0;     // Setpoint from POT
int lastTempRPMSet;           // last Setpoint from POT
int routerRPM;                // Actual RPM Value for Serial and LCD

int potVal;                   // Value from POT
int potMath;                  // Averaged Value of POT
int potValMapped;             // Averaged POT Value Scaled to RPM Setpoints
int totalPotVal;              // POT Values added together to be Averaged
int totalPotRead = 0;         // Number of POT Reads
int maxPotReads = 15;         // Max Number of POT Reads before Averaging Takes Place

int inputMode = 0;
int switchInputStat = 1;
int lastSwitchInputStat = 1;
int switchInputUpdateCnt = 0;

unsigned int prev_optical_pwm;
const long US_PERIOD_TO_RPM = 60000000;              // Convert from us of one rotation to RPM.
volatile unsigned long current_rpm_time = 0;         // Spindle RPM PWM input calc
volatile unsigned long prev_rpm_time = 0;            // Spindle RPM PWM input calc
volatile unsigned long rpm_value = US_PERIOD_TO_RPM; // Spindle RPM input in microseconds
volatile unsigned long prev_rpm_value;
volatile unsigned long last_rpm_value = 0;           // Stored Value for Signal Peak Filtering
volatile unsigned long pwm_value = 0;                // Marlin PWMms value in microseconds
volatile unsigned long prev_time = 0;                // Marlin PWMms Math
int triac_scaled;                                    // Traic is not 0-255

// PID variables
double Setpoint = 0.0;                           
double Input = 0.0;
double Output = 0.0;

//PID Gains
double Kp=2.1;
double Ki=7.0;
double Kd=0.025;

/*// Original //
Kp=2.1;
Ki=7;
Kd=0.025;
*/

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);                   // PID library

const int MAX_TOOL_RPM = 30000;                                              // SETTINGS per "spindle" ?1k headroom needed?
const int MAX_PWM_INPUT_US = 2024;                                           // Settings the microseconds of the max PWM from Marlin.


////////////////////////////////////////////////////////////////////////////////////////////

// spindleRPM gets called on a falling edge of the PHOTO_PIN, and records the amount of time
// between edges in the rpm_value field (in microsecinds). ISr.
void spindleRPM() {
   // Capture the time at "now"
   current_rpm_time = micros();

   // Store the microseconds since the last sample.
   rpm_value = current_rpm_time - prev_rpm_time;

  /////// Filter Incoming Values to Exclude Outliers ///////
  // Made Filtering more Precise by Splitting it into Ranges // 
   if(Setpoint > 200) {
     if(rpm_value < 1800) {
      rpm_value = last_rpm_value;
     }
   }
   else if (Setpoint > 125 && Setpoint < 201) {
    if(rpm_value < 2500) {
      rpm_value = last_rpm_value;
    }
   }
   else if (Setpoint < 126) {
    if(rpm_value < 3200) {
      rpm_value = last_rpm_value;
    }
   }
   // Start a new "timer" by setting the previous to "now".
   prev_rpm_time = micros();
   }


void rising() {
  // Capture when this is rising.
   prev_time = micros();

   // Set the next interrupt.
   interruptMode = 1;             // Set Value to Change to "Falling" in Loop
}

void falling() {
  // Measure the time since the last rising edge, in microseconds.
   pwm_value = micros()-prev_time;

   // Set the next interrupt.
   attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);
}
///////////////////////////////////////////////////////////////////////

void setup()
{
    pinMode(PHOTO_PIN, INPUT);                                               // Spindle RPM input pin
    attachInterrupt(digitalPinToInterrupt(PHOTO_PIN), spindleRPM, FALLING);  // Spindle RPM interrupt
    
    pinMode(PHOTOPWR, OUTPUT);
    digitalWrite(PHOTOPWR, HIGH);

    //pinMode(PWM_PIN, INPUT);                                                 // Marlin INPUT Pin
    //attachInterrupt(digitalPinToInterrupt(PWM_PIN), rising, RISING);         // Marlin INPUT
    
    pinMode(POT_PIN, INPUT_PULLUP);     // Pot INPUT
    pinMode(readPot, INPUT_PULLUP);
    pinMode(buttonGnd, OUTPUT);
    digitalWrite(buttonGnd, LOW);

    pinMode(POT_PWR, OUTPUT);    // POT 5V Source
    pinMode(POT_GND, OUTPUT);    // POT GND Source
    digitalWrite(POT_PWR, HIGH);
    digitalWrite(POT_GND, LOW);

    pinMode(readPot, INPUT_PULLUP);       // Set A2 to Read POT
    pinMode(switchInput, INPUT_PULLUP);   // Set Input Mode Button PIN
    
    analogWrite(ROUTER_PWM_OUTPIN,0);     // Triac initial state = off safety? 
    
    myPID.SetMode(AUTOMATIC);             // PID auto/manual *TODO change with lcd button
    myPID.SetSampleTime(8.2);             // PID Loop time this is faster than 60Hz Double check 

    //Serial.begin(9600);

  // Initiate the LCD:
  lcd.init();
  lcd.backlight();  
}

void loop() {  
///////////////////////////////////////////////////////////////////////////////////////
                                //  INPUT MODE SECTION  //
///////////////////////////////////////////////////////////////////////////////////////

   ////////////// Set Input Mode ////////////////////////
   // 0 = Potentiometer, 1 = Marlin
  switchInputStat = digitalRead(switchInput);
  if(switchInputUpdateCnt > 40) {
   if(switchInputStat != lastSwitchInputStat) {
     if(switchInputStat == 0) {
       inputMode++;
       lcd.clear();
       switchInputUpdateCnt = 0;
     }
     if(inputMode > 1) {
       inputMode = 0;
     }
   }
  }
  lastSwitchInputStat = switchInputStat;
  if(switchInputUpdateCnt < 41) {
    switchInputUpdateCnt++; // Increment Counter for Update Button Debounce
  }

///////////////////////// INPUT MODE FOR POTENTIOMETER ////////////////////
  if(inputMode == 0) {
   if(totalPotRead == 15) {           //
    totalPotRead = 0;                 // Average the Pot Reading
    potMath = totalPotVal / 15;       //
    potValMapped = map(potMath, 20, 1020, 0, 21);  // Convert Pot Reading
    if(potValMapped == 0) {
      tempRouterRPMSet = 0;               // Set RPM to 0 if POT in 0 range
    }
    if(potValMapped > 0) {
      tempRouterRPMSet = map(potValMapped, 1, 21, 10000, 30000);  // Map Pot Ranges into 10k Increments
    }
    totalPotVal = 0;
   }

   potVal = analogRead(POT_PIN);        // Update Pot Reading
   totalPotVal = totalPotVal + potVal;  // Add to Total Pot Reading
   totalPotRead++;                      // Increment Pot Reading for Averaging
    // Set Running RPM With PushButton
   if(tempRouterRPMSet != routerRPMSet) {
    int readPotStat = (digitalRead(readPot));
    if(readPotStat == 0) {
      routerRPMSet = tempRouterRPMSet;
      counter = 6;
      lcdClear = 1;                     // Set 1 Time LCD Clear After Changing Setpoint
    }
   }
  }

  /////////////////////////// INPUT MODE FOR MARLIN PWM //////////////////////////////////
  if(inputMode == 1) {
    // Sets Interrupt to Falling after capturing Rising (Due to PlatformIO Not Recognizing VOIDS below other VOIDS)
    if(interruptMode == 1) {
      attachInterrupt(digitalPinToInterrupt(PWM_PIN), falling, FALLING);
      interruptMode = 0;
    }
    routerRPMSet = map(pwm_value, 0, MAX_PWM_INPUT_US, 0, 255);   // PID setpoit from Marlin
  }

      



//////////////////////////////////////////////////////////////////////////////////////////
                                //  PWM OUTPUT SECTION  //
//////////////////////////////////////////////////////////////////////////////////////////

   last_rpm_value = rpm_value;                               // Value for RPM Filtering

   // Compute the spindle's RPM value.
   unsigned long rpm_math = (US_PERIOD_TO_RPM / rpm_value)-1; // Spindle, interrupt microseconds to RPM
   unsigned int optical_pwm = rpm_math * 255 / MAX_TOOL_RPM; // Spindle, RPM to PWM
             
   // One iteration of the PID.
   Input = optical_pwm;                                      // PID Input from router
   //int routerRPMSet = 30000;                               // Set RPM Manually in Code
   Setpoint = map(routerRPMSet, 0, 30000, 0, 255);           // Set RPM With Potentiometer
   myPID.Compute();                                          // PID Run the Loop

   // Set the output of the speed controller.
   if (routerRPMSet == 0){                                   // Setpoint is 0
       pwm_value = 0;                                        // Reset PID
       Output = 0;                                           // Reset PID
       analogWrite(ROUTER_PWM_OUTPIN, 0);                    // Turn off Spindle AC
       rpm_value = US_PERIOD_TO_RPM;                         // Disregards the spindown but clears the PID
       }
    else {                                                   // If spindle is enabled write PID value to triac
       //triac_scaled = map(Output, 0, 255, 0, 255);         // Traic scaling 20-240 not needed? test/tune
       analogWrite(ROUTER_PWM_OUTPIN, Output);               // Out to AC control Triac
       routerRPM = map(optical_pwm, 0, 255, 0, 30000);       // Convert Optical PWM to Actual RPM
       }




/////////////////////////////////////////////////////////////////////////////////
                              //  LCD Section  //
/////////////////////////////////////////////////////////////////////////////////
  // Show New Setpoint Selection for 2 seconds
  if(lastTempRPMSet != tempRouterRPMSet) {
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("New Set ");
    lcd.setCursor(11, 0);
    lcd.print(tempRouterRPMSet);
    lastTempRPMSet = tempRouterRPMSet;
    counter = 0;
    lcdClear = 1;
  }
  // Show Current Setpoint After 2 seconds or Update Button Pressed
  else if(counter >= 2) { // Clears Screen Once After New Temp Setpoint
    if(lcdClear == 1) {
      lcd.clear();
      lcdClear = 0;
    }
    if(currentMillis >= lastMillislcd1 + 1000) {  // Updates Setpoint 1 time a second
      //lcd.clear();
      if(routerRPMSet < 1000) {
          routerRPM = 0;
          lcd.clear();
      }
      lcd.setCursor(2, 0);
      lcd.print("Setpoint ");
      lcd.setCursor(11, 0);
      lcd.print(routerRPMSet);
      lcd.setCursor(17, 0);
      lcd.print("RPM");
      lastMillislcd1 = currentMillis;
    }
  }
  // Show Current Router RPM
  if(currentMillis >= lastMillislcd2 + 200) { // Updates Current RPM 5 times a second
    lcd.setCursor(3, 1);
    lcd.print("Current ");
    lcd.setCursor(11, 1);
    lcd.print(routerRPM);
    lcd.setCursor(17, 1);
    lcd.print("RPM");
    lastMillislcd2 = currentMillis;

    // Show Current Mode
    lcd.setCursor(12, 3);
    lcd.print("Mode ");
    lcd.setCursor(17, 3);
    if(inputMode == 0) {
      lcd.print("POT");
    }
    if(inputMode == 1) {
      lcd.print("MAR");
    }
  }




///////////////////////////////////////////////////////////////////////////
  // Timing Update for LCD Refresh (Limits Number of Refreshes per Second)
  currentMillis = millis();
  if(currentMillis >= lastMillis + 1000) {
    if(counter < 3) {
      counter++;
    }
    lastMillis = currentMillis;
  }
}


