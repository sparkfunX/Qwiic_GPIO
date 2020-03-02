/************************************************************************
 * Qwiic GPIO test sketch
 * 
 * Priyanka Makin @ SparkFun Electronics
 * Original Creation Date: February 28, 2020 
 ***********************************************************************/
#include <Wire.h>

byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2
const int LED = 13;

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic GPIO test sketch");

  Wire.begin();

  pinMode(LED, OUTPUT);
}

void loop()
{
 if (testForConnectivity() == true)
 {
  Serial.println("Qwiic GPIO connected.");
  digitalWrite(LED, HIGH); 
 }
 else
 {
  Serial.println("Nothing detected.");
  digitalWrite(LED, LOW);
 }
 delay(10);
}

boolean testForConnectivity() {
  Wire.beginTransmission(qwiicGpioAddress);
  //check here for an ACK from the slave
  if (Wire.endTransmission() != 0){
    return false;
  }
  return true;
}
