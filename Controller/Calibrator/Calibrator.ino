#include <Constants.h>

/**
 * This sketch is used to find calibration values for the controller.
 * It averages calibration values over a series of time.
 */

int rollOffset, pitchOffset, yawOffset, throttleOffset;
// The number of samples to average
const int AVG_LENGTH = 50;

void setup() {
  Serial.begin(9600);
  
  pinMode(THR_JOYSTICK, INPUT);
  pinMode(YAW_JOYSTICK, INPUT);
  pinMode(ROLL_JOYSTICK, INPUT);
  pinMode(PITCH_JOYSTICK, INPUT);
  pinMode(POT, INPUT);
  pinMode(TOGGLE, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  
  // delay, why? who knows
  delay(1000);
  
  // tone while calibrating
  tone(SPEAKER_PIN, 220);
  
  average(yawOffset, YAW_JOYSTICK);
  average(rollOffset, ROLL_JOYSTICK);
  average(pitchOffset, PITCH_JOYSTICK);
  
  Serial.println("-------------- Calibration --------------");
  Serial.print("YAW: " + yawOffset);
  Serial.print("\tROLL: " + rollOffset);
  Serial.println("\tPITCH: " + pitchOffset);
  Serial.println("------------ End Calibration ------------");
  Serial.println();
  
  noTone(SPEAKER_PIN);
  delay(1000);
}

void loop() {
  Serial.print("ROLL:" + analogRead(ROLL_JOYSTICK));
  Serial.print("\tPITCH: " + analogRead(PITCH_JOYSTICK));
  Serial.print("\tYAW: " + analogRead(YAW_JOYSTICK));
  Serial.print("\tTHR: " + analogRead(THR_JOYSTICK));
  Serial.print("\tPOT: " + analogRead(POT));
  Serial.println("\tTOG: " + digitalRead(TOGGLE));
  delay(100);
}

/**
 * Comutes the average value of a pin
 * for AVG_LENGTH samples.
 * 
 * @param &avgValue  The average value of the pin
 * @param pin        The pin to sample.
 */
int average(int &avgValue, int pin) {
  int sum = 0;
  for(int i = 0; i < AVG_LENGTH; i++) {
    sum += waitForTen(pin); 
  }
  avgValue = sum / AVG_LENGTH;
}

/**
 * Samples a pin and returns a value only
 * when it has been the same for 10 consecutive
 * samples.
 */ 
int waitForTen(int pin) {
  int count = 0;
  //get first val
  int last = 0;
  
  while(count < 10) {
    int current = analogRead(pin);

    if(current == last) {
      count++;
    } else {
      count = 0;
    }
    
    last = current;
  }
}
