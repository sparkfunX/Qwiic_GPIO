/******************************************************************************
  Qwiic Relay Example 3 - Interrupts
  Kevin Kuwata @ SparkX
  April 17, 2018
  https://github.com/sparkfunX/Qwiic_GPIO
  Arduino 1.8.5

  This example notifies the user when an interrupt occurs. Interrupts
  can only occur on pins that are configured as inputs. 

  The interrupt pin is active low.
   
  An internal pull up is required for the pin connected to the Interrupt.  
******************************************************************************/

#include <Wire.h>

/* I2C Command Byte Defines  */
#define REGISTER_INPUT_PORT           0x00    //register 0 
#define REGISTER_OUTPUT_PORT          0X01    //register 1
#define REGISTER_CONFIGURATION         0X03    //register 3

byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2 
volatile byte oldInputState = 0x00; //Global variable used to keep track of pin change to determine interrupts.

void setup(){
	Serial.begin(9600);
	Serial.println("Qwiic GPIO Example 3- Interrupts");
	Wire.begin();
	
if(testForConnectivity() == false){
	Serial.println("Check Connections. No Qwiic GPIO detected.");
	while(1);
}

	setPinMode(qwiicGpioAddress, 0, INPUT);
	setPinMode(qwiicGpioAddress, 7, INPUT);
	
	pinMode(13, OUTPUT); //using this to figure out loop speed
	digitalWrite(13, HIGH);
	
	pinMode(8, INPUT);
	digitalWrite(8, HIGH); // Internal pull up
	
	//record the input pin states to allow identifcation of state change
	oldInputState = readRegister(qwiicGpioAddress, REGISTER_INPUT_PORT); 	
}

void loop(){

	digitalWrite(13, HIGH);
	
	if(digitalRead(8) == LOW){
		Serial.println("interrupt occurred");
	//Serial.println(readRegister(qwiicGpioAddress, REGISTER_CONFIURATION), 2);	
	//Serial.println(readRegister(qwiicGpioAddress, REGISTER_INPUT_PORT), 2);
		Serial.println(interruptedPin(qwiicGpioAddress), BIN);
		delay(250);
		digitalWrite(8, HIGH);
	}
	digitalWrite(13, LOW);
}


/* ============================================================ */
/*
	interruptedPin(byte address)returns a byte, with the pins that have shown a change.
	For example if pin 0 and pin 7 triggered interrupts the return byte would be
	0b10000001
	If only pin 3 triggered then the return byte would be
	0b00001000
	
	Note: The interrupt Pin will trigger twice due to rising and falling edges. It is 
	up to the user to construct logic around this. 
*/
byte interruptedPin(byte address){
	byte inputRegister = readRegister(address, REGISTER_INPUT_PORT);
	byte returnValue = 0;
	
	if(inputRegister != oldInputState){
		//mask byte to find pin
		//left shift a 1 by the number of pins
		// or the result
		// return
		//what if you did a XOR and just returned the value and reset
		
	returnValue = oldInputState ^ inputRegister; //Bits that changed will be a 1.
	oldInputState = inputRegister;
	return (returnValue);
	}
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
	setPinOutput() will set a pin as high or low
	input: address of the Qwiic GPIO 
		   pin, the pin you wish to control
		   state, the logic level HIGH or LOW.
		   	
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
	readRegister(byte address, byte registerToRead)
	
	This function will read the register values of a given register at a 
	given address.

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
	writeRegister(byte address, byte registerToWrite, byte valueToWrite)	
	This function takes the Qwiic GPIO address, the register to write to, 
	and the value to write
	
	Note when writing to a register a full 8 bits is written- overwriting any 
	previous state.
	
	Inputs: Qwiic Gpio Address, Register to write to, and Byte to write
*/

void writeRegister(byte address, byte registerToWrite, byte valueToWrite){
	Wire.beginTransmission(address);
	Wire.write(registerToWrite); 
	Wire.write(valueToWrite);
	Wire.endTransmission();
}