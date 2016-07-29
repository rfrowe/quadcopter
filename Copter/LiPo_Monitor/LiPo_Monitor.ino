#define OFFSET 0.14

void setup() {
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
}

void loop() {
 getBatteryVoltage();
}

void getBatteryVoltage() { 
  double reading1 = analogRead(A0);
  double reading2 = analogRead(A1);
  double reading3 = analogRead(A2);  
  
  double cell1 = mapDouble(reading1,0.0,1023.0,0.0,5.0) - OFFSET;
  double cell2 = mapDouble(reading2,0.0,1023.0,0.0,5.0) / (10000.0/(10000.0+10000.0)) - cell1 - 2*OFFSET;
  double cell3 = mapDouble(reading3,0.0,1023.0,0.0,5.0) / (10000.0/(20000.0+10000.0)) - cell2 - cell1 - 3*OFFSET;
  
  Serial.print("C1:");
  Serial.print(cell1);
  Serial.print(" C2:");
  Serial.print(cell2);
  Serial.print(" C3:");
  Serial.print(cell3);
  Serial.print(" Total:");
  Serial.println(cell1+cell2+cell3);
}

float mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result / 1000.0;
}
