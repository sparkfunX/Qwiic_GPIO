/******************************************************************************
  Qwiic Relay Example 1 - Write GPIO
  Kevin Kuwata @ SparkX
  April 17, 2018
  https://github.com/sparkfunX/Qwiic_GPIO
  Arduino 1.8.5

  This sketch sets up a GPIO 0 as an output and toggles
  the output HIGH and LOW. 
  
  This device sinks current. When an output is LOW then current will flow. 
  This logic has been taken care of for the user, thus the function 
  setPinOutput will behave similar digitalWrite.
   

******************************************************************************/

#include <Wire.h>
/* I2C Command Byte Defines  */
#define REGISTER_INPUT_PORT          	 	0x00    //register 0 
#define REGISTER_OUTPUT_PORT          		0X01    //register 1
#define REGISTER_CONFIGURATION        	 	0X03    //register 3

byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2 

void setup(){
	Serial.begin(9600);
	Serial.println("Qwiic GPIO Example 1- Write GPIO");
	Wire.begin();
if(testForConnectivity() == false){
	Serial.println("Check Connections. No Qwiic GPIO detected.");
	while(1);
}
	
	setPinMode(qwiicGpioAddress, 1, OUTPUT);
	setPinOutput(qwiicGpioAddress, 1, LOW);
}

void loop(){
	setPinOutput(qwiicGpioAddress, 1, HIGH);
	delay(1000);
	setPinOutput(qwiicGpioAddress, 1, LOW);
	delay(4000);
}


/* ============================================================ */


/*
	setPinMode(byte address, byte pin, byte direction) sets a single pin
	as an input or an output and does not affect the other pins. The direction
	input parameter is either INPUT or OUTPUT.
	
		similar to pinMode()
*/
void setPinMode(byte address, byte pin, byte direction){
	byte currentPinDirection = readRegister(address, REGISTER_CONFIGURATION);
		pin = 1 <<pin;
		
  if(direction == INPUT || direction == INPUT_PULLUP ){
		currentPinDirection |= pin; // pin will come in correctly masked because of a define. 
  }
  else if(direction == OUTPUT){
		currentPinDirection &= ~(pin);  
  }
  		writeRegister(address, REGISTER_CONFIGURATION, currentPinDirection );
}



/*
	setPinOutput(byte address, byte pin, byte state) will set a pin as high or low
	
			similar to digitalWrite(pin, state)

*/
void setPinOutput(byte address, byte pin, byte state){	
	pin =  1<<pin;
	
	byte currentRegisterValue = readRegister(address, REGISTER_OUTPUT_PORT);
	
	if(state == LOW){ //INVERSE OF ARDUINO, but this will result in the same effect a user would expect. 
		currentRegisterValue |= pin;
	}
	else if(state == HIGH){
		currentRegisterValue &= ~pin;
	}
	writeRegister(address, REGISTER_OUTPUT_PORT, currentRegisterValue);
}


/* 
	testForConnectivity() Checks for an ACK from the Qwiic GPIO
	Returns true if an Qwiic GPIO is connected. 
*/
boolean testForConnectivity() {
  Wire.beginTransmission(qwiicGpioAddress);
  //check here for an ACK from the slave, if no ack don't allow change?
  if (Wire.endTransmission() != 0) {
    return false;
  }
  return true;
}


/*
	readRegister(byte address, byte registerToRead) will read the register 
	values of a given register at a given address.

	Inputs: address, register values
*/
byte readRegister(byte address, byte registerToRead){
	Wire.beginTransmission(address);
	Wire.write(registerToRead); 
	Wire.endTransmission(false);
	Wire.requestFrom(address, 1);
	
	byte currentRegisterValue = 0;
	
	byte count = 0;
	while(Wire.available() >0){
	if(count ==0){
		currentRegisterValue = Wire.read();
	}
		Wire.read(); // don't collect
		count++;
	}
	return currentRegisterValue;
}

/*
	writeRegister(byte address, byte registerToWrite, byte valueToWrite) writes the 
	value passed to a specific register at the Qwiic GPIO address.
	
	Note: when writing to a register a full 8 bits is written- overwriting any 
	previous state.
	
	Inputs: Qwiic Gpio Address, Register to write to, and Byte to write
*/

void writeRegister(byte address, byte registerToWrite, byte valueToWrite){
	Wire.beginTransmission(address);
	Wire.write(registerToWrite); 
	Wire.write(valueToWrite);
	Wire.endTransmission();
}

