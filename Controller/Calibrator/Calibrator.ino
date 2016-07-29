//This sketch is used to find calibration values for the controller
//it averages calibration values over a series of time

#include <Constants.h>

int rightLeftOffset, frontBackOffset, yawOffset, throttleOffset;

void setup() {
  Serial.begin(9600);
  
  //pin modes
  pinMode(THR_JOYSTICK,INPUT);
  pinMode(YAW_JOYSTICK,INPUT);
  pinMode(RL_JOYSTICK,INPUT);
  pinMode(FB_JOYSTICK,INPUT);
  pinMode(POT,INPUT);
  pinMode(TOGGLE,INPUT);
  pinMode(SPEAKER_PIN,OUTPUT);
  
  delay(1000);
  
  tone(SPEAKER_PIN, 220);
  
  //YAW
  int sum = 0;
  for(int x = 0; x < 50; x ++) {
    sum += calibrate(YAW_JOYSTICK); 
  }
  yawOffset = sum / 50;
  
  //RL
  sum = 0;
  for(int x = 0; x < 50; x ++) {
    sum += calibrate(RL_JOYSTICK); 
  }
  rightLeftOffset = sum / 50;
  
  //FB
  sum = 0;
  for(int x = 0; x < 50; x ++) {
    sum += calibrate(FB_JOYSTICK); 
  }
  frontBackOffset = sum / 50;
  
  Serial.print(" YAW: ");
  Serial.print(yawOffset);
  Serial.print(" RL: ");
  Serial.print(rightLeftOffset);
  Serial.print(" FB: ");
  Serial.println(frontBackOffset);
  delay(1000);
  
  noTone(SPEAKER_PIN);
}

void loop() {
  int RL = analogRead(RL_JOYSTICK);
  int FB = analogRead(FB_JOYSTICK);
  int YAW = analogRead(YAW_JOYSTICK);
  int THROTTLE = analogRead(THR_JOYSTICK);
  int POTReading = analogRead(POT);
  int toggle = digitalRead(TOGGLE);
  Serial.print("RL:");
  Serial.print(RL);
  Serial.print(" FB:");
  Serial.print(FB);
  Serial.print(" YAW:");
  Serial.print(YAW);
  Serial.print(" THR:");
  Serial.print(THROTTLE);
  Serial.print(" POT:");
  Serial.print(POTReading);
  Serial.print(" TOG:");
  Serial.println(toggle);
  delay(50);
}

int calibrate(int pin) {
  int count = 0;
  int sum = 0;
  //get first val
  int last = analogRead(pin);
  delay(10);
  
  //stop if count is 10 or more
  while(count < 10) {
    //get current val
    int current = analogRead(pin);
    //increment count if they're the same
    if(current == last) {
      count++;
      sum+=current;
    }
    
    last = current;
  }
  
  return sum / count;
}
