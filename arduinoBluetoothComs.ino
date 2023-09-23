#include <SoftwareSerial.h>
#include <HCSR04.h>

HCSR04 hcFront(3,2); // (Trig, Echo)
HCSR04 hcLeft(8,7);
HCSR04 hcRight(12,13); 

SoftwareSerial bluetoothSerial(0, 1); // RX, TX pins on Arduino

void setup() {
  Serial.begin(9600);      // Serial monitor for debugging
  bluetoothSerial.begin(9600); // Bluetooth module communication
}

void loop() {
  int rightProx = hcRight.dist();
  int leftProx = hcLeft.dist();
  int frontProx = hcFront.dist();

  String values = "values: " + String(rightProx) + "," + String(leftProx) + "," + String(frontProx);
  
  // Send the 'values' string over Bluetooth
  bluetoothSerial.println(values);

  Serial.println(values); // For debugging on the serial monitor
  
  delay(60);
}
