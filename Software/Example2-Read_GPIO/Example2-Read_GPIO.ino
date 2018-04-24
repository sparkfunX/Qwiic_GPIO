/******************************************************************************
  Qwiic Relay Example 2 - Read GPIO
  Kevin Kuwata @ SparkX
  April 17, 2018
  https://github.com/sparkfunX/Qwiic_GPIO
  Arduino 1.8.5

  This sketch sets up a GPIO 5 as an input and makes GPIO 1 an output. 
  
  When the GPIO 5 is read LOW, GPIO 1 is set to HIGH and when GPIO5 is read 
  HIGH GPIO 1 is set LOW. 
  
******************************************************************************/

#include <Wire.h>


/* I2C Command Byte Defines  */
#define REGISTER_INPUT_PORT           	0x00    //register 0 
#define REGISTER_OUTPUT_PORT          	0X01    //register 1
#define REGISTER_CONFIGURATION         	0X03    //register 3

byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2 

void setup(){
	  // put your setup code here, to run once:
	Serial.begin(9600);
	Serial.println("Qwiic GPIO Example 2- Read GPIO");
	Wire.begin();

	testForConnectivity();
	
	setPinMode(qwiicGpioAddress, 1, OUTPUT);
	setPinMode(qwiicGpioAddress, 7, INPUT);
	
	setPinOutput(qwiicGpioAddress, 1, HIGH);
}

byte pinStateChangeOccurred = 0;
void loop(){	
	while(readPin(qwiicGpioAddress, 7) == LOW){
		setPinOutput(qwiicGpioAddress, 1, HIGH);
		Serial.println("Button Pressed");
		delay(100);
		pinStateChangeOccurred = 1;
	}	
	
	if(pinStateChangeOccurred == 1){
	setPinOutput(qwiicGpioAddress, 1, LOW);
		pinStateChangeOccurred = 0;
	}
}

/* ============================================================ */


/*
	readPin(byte address, byte pin) reads the input state of the specified pin
	and returns the corresponding pin's state. 
	
	similar to digitalRead()
*/
byte readPin(byte address, byte pin){
	byte currentRegisterValue = readRegister( address, REGISTER_INPUT_PORT);
	if(currentRegisterValue & (1 << pin)){
	return HIGH;
	}
	return LOW;
}

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
