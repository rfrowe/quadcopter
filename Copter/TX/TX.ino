/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <XBee.h>
#include <SoftwareSerial.h>

/*
This example is for Series 2 XBee
 Sends a ZB TX request with the value of analogRead(pin5) and checks the status response for success
*/

// create the XBee object
XBee xbee = XBee();
SoftwareSerial nss(8, 9);

uint8_t payload[] = { 0 };

// SH + SL Address of receiving XBee
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40c67584);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();

int pin5 = 0;

boolean start = false;

int statusLed = 13;
int errorLed = 13;
int wake = 10;

void flashLed(int pin, int times, int wait) {

  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}

void setup() {
  pinMode(statusLed, OUTPUT);
  pinMode(errorLed, OUTPUT);
  pinMode(wake, OUTPUT);
  
  Serial.begin(9600);
  nss.begin(9600);

  xbee.setSerial(nss);
}

void loop() {
  // break down 10-bit reading into two bytes and place in payload
  if (Serial.available() > 0) {
    String str = Serial.readString();
    if(str == "1") {
      Serial.println("Going HIGH");
      payload[0] = 1;
    } else if(str == "0") {
      Serial.println("Going LOW");
      payload[0] = 0;
    } else if(str == "Start") {
      Serial.println("Starting");
      start = true;
    } else if(str == "Stop") {
      Serial.println("Stopping");
      start = false;
    }
  }
  
  if(!start) {
    Serial.println("Waiting");
    delay(50);
    return;
  }
  
  //wake arduino to send
  //digitalWrite(wake,LOW);
  xbee.send(zbTx);
  
  Serial.print("Transmitting ");
  Serial.println(payload[0]);

  // flash TX indicator
  //flashLed(statusLed, 1, 100);

  // after sending a tx request, we expect a status response
  // wait up to half second for the status response
  if (xbee.readPacket(500)) {
    // got a response!

    // should be a znet tx status
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        // success.  time to celebrate
        //flashLed(statusLed, 5, 50);
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        flashLed(errorLed, 3, 500);
      }
    }
  } else if (xbee.getResponse().isError()) {
    //nss.print("Error reading packet.  Error code: ");
    //nss.println(xbee.getResponse().getErrorCode());
  } else {
    // local XBee did not provide a timely TX Status Response -- should not happen
    flashLed(errorLed, 2, 50);
  }
  
  //sleep arduino
  //digitalWrite(wake,HIGH);
  delay(50);
}

