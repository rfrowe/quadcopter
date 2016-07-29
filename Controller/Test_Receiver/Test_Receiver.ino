#include <XBee.h>
#include <Constants.h>
#include <ServoTimer2.h>
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SoftwareSerial.h>

#define OFFSET 0.14

XBee xbee = XBee();
//SoftwareSerial nss(2, 3);
XBeeResponse response = XBeeResponse();
XBeeAddress64 broadcast = XBeeAddress64(0x0013A200, 0x40C67566);
uint8_t payload[4];
ZBTxRequest packet = ZBTxRequest(broadcast, payload, sizeof(payload));
// create reusable response objects for responses we expect to handle
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();

double yawReading, rollReading, pitchReading;
double nextYaw, nextRoll, nextPitch;
double roll_output, yaw_output, pitch_output;
double rollAngle, pitchAngle, nextPitchAngle, nextRollAngle;
//#define RKp 0.8465298142717498
#define RKp 0.4
#define RKi 0
#define RKd 0
#define PKp 0.4
#define PKi 0
#define PKd 0
#define YKp 2
#define YKi 0
#define YKd 0
#define RKp_STAB 4.5
#define RKi_STAB 0
#define RKd_STAB 0
#define PKp_STAB 4.5
#define PKi_STAB 0
#define PKd_STAB 0
PID rollPID(&rollReading, &roll_output, &nextRoll, RKp, RKi, RKd, DIRECT);
PID pitchPID(&pitchReading, &pitch_output, &nextPitch, PKp, PKi, PKd, DIRECT);
PID rollStabPID(&rollAngle, &nextRoll, &nextRollAngle, RKp_STAB, RKi_STAB, RKd_STAB, DIRECT);
PID pitchStabPID(&pitchAngle, &nextPitch, &nextPitchAngle, PKp_STAB, PKi_STAB, PKd_STAB, DIRECT);
PID yawPID(&yawReading, &yaw_output, &nextYaw, YKp, YKi, YKd, DIRECT);

boolean failsafe = true;
int count = 0;
int throttleReading = ESC_OFF;
boolean mode = RATE_MODE;

ServoTimer2 FL, FR, BL, BR;
int lastFL, lastFR, lastBL, lastBR;

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
#define GYRO_X_OFFSET -0.0451872205
#define GYRO_Y_OFFSET 0.0425009727
#define GYRO_Z_OFFSET 0.0940304565

void setup() {
  FL.attach(4);
  FR.attach(5);
  BL.attach(6);
  BR.attach(7);

  accel.begin();
  mag.begin();
  gyro.begin();

  /*pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);*/

  // start serial
  Serial.begin(9600);
  //nss.begin(9600);
  xbee.begin(Serial);

  writeServo(ESC_OFF);

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-167.0, 167.0);
  pitchPID.SetMode(AUTOMATIC);
  pitchPID.SetOutputLimits(-167.0, 167.0);
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-167.0, 167.0);
  pitchStabPID.SetMode(AUTOMATIC);
  pitchStabPID.SetOutputLimits(-167.0, 167.0);
  rollStabPID.SetMode(AUTOMATIC);
  rollStabPID.SetOutputLimits(-167.0, 167.0);

  delay(8000);
}

// continuously reads packets, looking for ZB Receive or Modem Status
void loop() {
  unsigned long initial = millis();

  dtostrf(rollPID.GetKd(), 4, 3, (char*) payload);
  //payload[0] = (byte) getBatteryVoltage();
  //xbee.send(packet);

  int nextFL = lastFL;
  int nextFR = lastFR;
  int nextBL = lastBL;
  int nextBR = lastBR;

  if (getData(rx)) {
    char data[rx.getDataLength()];
    for (int x = 0; x < sizeof(data); x++) {
      data[x] = rx.getData(x);
    }
    String blah = String(data);

    if (blah.charAt(0) == 'C') {
      float p = blah.substring(1, blah.indexOf(' ')).toFloat();
      blah.setCharAt(blah.indexOf(' '), ',');
      float i = blah.substring(blah.indexOf(',') + 1, blah.indexOf(' ')).toFloat();
      blah.setCharAt(blah.indexOf(' '), '|');
      float d = blah.substring(blah.indexOf('|') + 1).toFloat();

      pitchPID.SetTunings(p, i, d);
      rollPID.SetTunings(p, i, d);
    } else if (blah.charAt(0) == 'T') {
      mode = getMode(rx);
      double p = mapDouble(getPOT(rx), 0.0, 1023.0, 0.0, 4);
      if (p != yawPID.GetKd()) {
        yawPID.SetTunings(p, yawPID.GetKi(), yawPID.GetKd());
      }
      throttleReading = getTHR(rx);
      nextYaw = getYAW(rx);

      /*Serial.print("THR:");
      Serial.print(getTHR(rx));
      Serial.print(" YAW:");
      Serial.print(getYAW(rx));
      Serial.print(" ROLL:");
      Serial.print(getROLL(rx));
      Serial.print(" PITCH:");
      Serial.print(getPITCH(rx));
      Serial.print(" POT:");
      Serial.print(getPOT(rx));
      Serial.print(" TOG:");
      Serial.println(mode);*/

      if (mode == RATE_MODE) {
        nextRoll = getROLL(rx);
        nextPitch = getPITCH(rx);
      } else {
        sensors_event_t event;
        sensors_vec_t orientation;

        accel.getEvent(&event);
        mag.getEvent(&event);
        dof.fusionGetOrientation(&event, &event, &orientation);

        nextPitchAngle = getPITCH(rx);
        nextRollAngle = getROLL(rx);

        pitchAngle = orientation.pitch;
        rollAngle = orientation.roll;

        rollStabPID.Compute();
        pitchStabPID.Compute();
      }
    }
  } else {
    if (failsafe) {
      if (throttleReading >= ESC_MIN  + 3) {
        throttleReading -= 3;

        nextRoll = 0.0;
        nextPitch = 0.0;
        nextYaw = 0.0;
      }
    } else if (count == 10) {
      failsafe = true;
    } else if (count < 10) {
      count++;
    }
  }

  nextFL = throttleReading;
  nextFR = throttleReading;
  nextBL = throttleReading;
  nextBR = throttleReading;

  if (throttleReading > ESC_MIN + 50) {
    sensors_event_t event;
    gyro.getEvent(&event);

    yawReading = getGyroZ(event);
    rollReading = getGyroY(event);
    pitchReading = getGyroX(event);

    rollPID.Compute();
    pitchPID.Compute();
    //yawPID.SetOutputLimits((roll_output + pitch_output) - 500, 500 - (roll_output + pitch_output));
    yawPID.Compute();

    /*Serial.print("YAW:"); Serial.print(yawReading);
    Serial.print("\tNYAW:"); Serial.print(nextYaw);
    Serial.print("\tYO:"); Serial.print(yaw_output);

    Serial.print(" \tROLL:"); Serial.print(rollReading);
    Serial.print("\tNROLL:"); Serial.print(nextRoll);
    Serial.print("\tRO:"); Serial.print(roll_output);

    Serial.print(" \tPITCH:"); Serial.print(pitchReading);
    Serial.print("\tNPITCH:"); Serial.print(nextPitch);
    Serial.print("\tPO:"); Serial.print(pitch_output);*/

    nextFL = limit(nextFL + roll_output - pitch_output - yaw_output, ESC_MIN, ESC_MAX);
    nextFR = limit(nextFR - roll_output - pitch_output + yaw_output, ESC_MIN, ESC_MAX);
    nextBL = limit(nextBL + roll_output + pitch_output + yaw_output, ESC_MIN, ESC_MAX);
    nextBR = limit(nextBR - roll_output + pitch_output - yaw_output, ESC_MIN, ESC_MAX);
  }

  writeServo(nextFL, nextFR, nextBL, nextBR);

  unsigned long final = millis();
  //Serial.print("  \tT:");
  //Serial.println(final - initial);
  delay(50 - final + initial);
}

boolean getData(ZBRxResponse &rx) {
  xbee.readPacket();

  if (xbee.getResponse().isAvailable() && xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
    xbee.getResponse().getZBRxResponse(rx);
    count = 0;
    failsafe = false;
    return true;
  }

  return false;
}

void writeServo(int nextFL, int nextFR, int nextBL, int nextBR) {
  if (nextFL != lastFL) {
    FL.write(nextFL);
    lastFL = nextFL;
  }
  if (nextFR != lastFR) {
    FR.write(nextFR);
    lastFR = nextFR;
  }
  if (nextBL != lastBL) {
    BL.write(nextBL);
    lastBL = nextBL;
  }
  if (nextBR != lastBR) {
    BR.write(nextBR);
    lastBR = nextBR;
  }
}

boolean getBatteryVoltage() {
  double reading1 = analogRead(A0);
  double reading2 = analogRead(A1);
  double reading3 = analogRead(A2);

  double cell1 = mapDouble(analogRead(A0), 0.0, 1023.0, 0.0, 5.0) - OFFSET;
  double cell2 = mapDouble(analogRead(A1), 0.0, 1023.0, 0.0, 5.0) / (10000.0 / (10000.0 + 10000.0)) - cell1 - 2 * OFFSET;
  double cell3 = mapDouble(analogRead(A2), 0.0, 1023.0, 0.0, 5.0) / (10000.0 / (20000.0 + 10000.0)) - cell2 - cell1 - 3 * OFFSET;

  return cell1 > 3.3 && cell2 > 3.3 && cell3 > 3.3;
}

void writeServo(int newValue) {
  writeServo(newValue, newValue, newValue, newValue);
}

int getTHR(ZBRxResponse &rx) {
  return (rx.getData(1) & 0xE0) << 3 | rx.getData(6);
}
int getYAW(ZBRxResponse &rx) {
  return twos_comp(rx.getData(1) << 8 & 0x100 | rx.getData(2), 9);
}
int getROLL(ZBRxResponse &rx) {
  return twos_comp(rx.getData(1) << 7 & 0x100 | rx.getData(3), 9);
}
int getPITCH(ZBRxResponse &rx) {
  return twos_comp(rx.getData(1) << 6 & 0x100 | rx.getData(4), 9);
}
int getPOT(ZBRxResponse &rx) {
  return (rx.getData(1) & 0x18) << 5 | rx.getData(5);
}
boolean getMode(ZBRxResponse &rx) {
  if (rx.getData(7) == 1) {
    return true;
  } else {
    return false;
  }
}

float mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double limit(double number, double min, double max) {
  if (number > max)
    return max;
  if (number < min)
    return min;
  return number;
}

double radToDeg(double reading) {
  return (reading * 180.0) / PI;
}

double getGyroX(sensors_event_t &event) {
  return radToDeg(event.gyro.x - GYRO_X_OFFSET);
}
double getGyroY(sensors_event_t &event) {
  return radToDeg(event.gyro.y - GYRO_Y_OFFSET);
}
double getGyroZ(sensors_event_t &event) {
  return radToDeg(event.gyro.z - GYRO_Z_OFFSET);
}

int twos_comp(int val, int bits) {
  if ((val & (1 << (bits - 1))) != 0)
    val = val - (1 << bits);
  return val;
}
