#include <XBee.h>
#include <SoftwareSerial.h>
#include <Constants.h>

const int CALIBRATE_TIME = 200;

XBee xbee = XBee();
SoftwareSerial nss(8, 9);
uint8_t payload[8];

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40c67584);
ZBTxRequest packet = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

boolean mode = RATE_MODE;
boolean changed = false;

void setup() {
  // start serials
  Serial.begin(9600);
  nss.begin(9600);
  xbee.setSerial(nss);

  // pin modes
  pinMode(THR_JOYSTICK, INPUT);
  pinMode(YAW_JOYSTICK, INPUT);
  pinMode(ROLL_JOYSTICK, INPUT);
  pinMode(PITCH_JOYSTICK, INPUT);
  pinMode(POT, INPUT);
  pinMode(TOGGLE, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);

  // delay to allow inputs to settle
  delay(1000);

  // the speaker will need a total resistance of at least 125 Ohm
  // so if it's an 8 Ohm speaker, an additional 117 is needed etc...

  // Wait for inputs to settle. Wait for each input to be within +-5 of 
  // its offset value as defined in Constants.h, calculated with Calibrator.ino
  waitUntilSettled(THR_JOYSTICK, THR_OFFSET, NOTE_A3);
  waitUntilSettled(YAW_JOYSTICK, YAW_OFFSET, NOTE_B3);
  waitUntilSettled(PITCH_JOYSTICK, PITCH_OFFSET, NOTE_C4);
  waitUntilSettled(ROLL_JOYSTICK, ROLL_OFFSET, NOTE_D4);

  //done
  tone(SPEAKER_PIN, NOTE_E4);
  delay(200);
  noTone(SPEAKER_PIN);
  delay(15);
  tone(SPEAKER_PIN, NOTE_E4);
  delay(200);
  noTone(SPEAKER_PIN);
  delay(15);
  tone(SPEAKER_PIN, NOTE_E4);
  delay(350);
  noTone(SPEAKER_PIN);

  payload[0] = 'T';
}

void loop() {
  unsigned long initial = millis();

  int toggleReading = digitalRead(TOGGLE);
  if (toggleReading == 1 && !changed) {
    mode = !mode;
    changed = true;
  } else if (toggleReading == 0 && changed) {
    changed = false;
  }

  int potValue = analogRead(POT);
  int throttleValue = mapLimit(analogRead(THR_JOYSTICK) - THR_OFFSET, THR_MIN, THR_OFFSET, THR_MAX, ESC_MIN, ESC_OFFSET);
  //disallow throttles less than min
  throttleValue = min(ESC_MIN, throttleValue);

  int yawValue = mapLimit(analogRead(YAW_JOYSTICK) - YAW_OFFSET, YAW_MIN, YAW_OFFSET, YAW_MAX, 0, map(potValue, 0, 1023, (mode == RATE_MODE) ? RATE_MIN : ANGLE_MIN, (mode == RATE_MODE) ? RATE_MAX : ANGLE_MAX));
  int rollValue = -mapLimit(analogRead(ROLL_JOYSTICK) - ROLL_OFFSET, ROLL_MIN, ROLL_OFFSET, ROLL_MAX, 0, map(potValue, 0, 1023, (mode == RATE_MODE) ? RATE_MIN : ANGLE_MIN, (mode == RATE_MODE) ? RATE_MAX : ANGLE_MAX));
  int pitchValue = mapLimit(analogRead(PITCH_JOYSTICK) - PITCH_OFFSET, PITCH_MIN, PITCH_OFFSET, PITCH_MAX, 0, map(potValue, 0, 1023, (mode == RATE_MODE) ? RATE_MIN : ANGLE_MIN, (mode == RATE_MODE) ? RATE_MAX : ANGLE_MAX));

  /*Serial.print("THR:");
  Serial.print(throttleValue);
  Serial.print(" YAW:");
  Serial.print(yawValue);
  Serial.print(" ROLL:");
  Serial.print(rollValue);
  Serial.print(" PITCH:");
  Serial.print(pitchValue);
  Serial.print(" POT:");
  Serial.print(potValue);
  Serial.print(" TOG:");
  Serial.print(digitalRead(TOGGLE));*/

  //THESE CAN ALL GO ON ONE BYTE BECAUSE THEY ONLY TAKE UP 6BITS EACH 2^6=64
  //yaw
  payload[1] = yawValue >> 8 & 0b00000001;
  payload[2] = yawValue & 0xff;
  //roll
  payload[1] |= rollValue >> 7 & 0b00000010;
  payload[3] = rollValue & 0xff;
  //pitch
  payload[1] |= pitchValue >> 6 & 0b00000100;
  payload[4] = pitchValue & 0xff;

  //Throttle, pot, and mode thrown in
  payload[1] |= potValue >> 5 & 0xF8 | throttleValue >> 3 & 0xE0;
  payload[5] = potValue & 0xff;
  payload[6] = throttleValue & 0xff;
  payload[7] = (byte) mode;

  /*Serial.print(" THR/POT/MODE:");
  Serial.print(payload[3],BIN);
  Serial.print(" ");
  Serial.print(payload[4],BIN);
  Serial.print(" ");
  Serial.print(payload[5],BIN);
  Serial.print(" YAW:");
  Serial.print(payload[0],BIN);
  Serial.print(" ROLL:");
  Serial.print(payload[1],BIN);
  Serial.print(" PITCH:");
  Serial.print(payload[2],BIN);*/


  xbee.send(packet);
  if (xbee.readPacket(125)) {
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getDeliveryStatus() != SUCCESS) {
        //the remote XBee didn't ACK
        tone(SPEAKER_PIN, NOTE_A3);
        delay(50);
        noTone(SPEAKER_PIN);
      }
    }
  } else {
    //error or something
    tone(SPEAKER_PIN, 600);
    delay(50);
    noTone(SPEAKER_PIN);
  }

  int diff = 125 - millis() + initial;
  if (diff > 0) {
    delay(125 - diff);
  }
}

/**
 * Waits for the value on pin to settle within 5 of offeset.
 * Plays note during delay and will never take less than
 * CALIBRATE_TIME to run.
 */
void waitUntilSettled(int pin, int offset, int note) {
  int start = millis();
  tone(SPEAKER_PIN, note);
  
  int reading = analogRead(pin);
  while (abs(reading - offset) > 5) {
    delay(20);
    reading = analogRead(pin);
  }
  
  int dif = CALIBRATE_TIME - millis() + start;
  if (dif <= CALIBRATE_TIME && dif >= 0) {
    delay(dif);
  }
}

/**
 * Maps a value, val, on a scale of fromLow to fromHigh, with an offset, to
 * a range of toLow to toHigh, returning a maximum value of toHigh
 */
int mapLimit(int val, int fromLow, int fromOffset, int fromHigh, int toLow, int toHigh) {
  int result;

  if (val > 0) {
    result = map(val, fromLow, fromHigh - fromOffset, toLow, toHigh);
  } else {
    result = map(val, fromLow, fromOffset - fromLow, toLow, toHigh);
  }

  if (result > toHigh) {
    result = toHigh;
  } else if (result < -toHigh) {
    result = -toHigh;
  }

  return result;
}
