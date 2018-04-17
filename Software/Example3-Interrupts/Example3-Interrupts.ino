/******************************************************************************
  Qwiic Relay Example 3 - Interrupts
  Kevin Kuwata @ SparkX
  April 17, 2018
  https://github.com/sparkfunX/Qwiic_GPIO
  Arduino 1.8.5

  This example notifies the user when an interrupt occurs. Interrupts
  can only occur on pins that are configured as inputs. 

  The interrupt pin is active low - when an interrupt occurs the pin 
  goes to LOW.
   
  An external pull up may be required. 
******************************************************************************/

#include <Wire.h>
byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2 

/* I2C Command Byte Defines  */
#define REGISTER_INPUT_PORT           0x00    //register 0 
#define REGISTER_OUTPUT_PORT          0X01    //register 1
#define REGISTER_POLARITY_INVERSION   0x02    //register 2
#define REGISTER_CONFIURATION         0X03    //register 3

//bit masks
#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

#define SET_OUTPUT 		0x00
#define SET_INPUT 		0x01

//these should probably be pin0 through 7 because of the way the board is labeled. 
#define PIN0			0b00000001
#define PIN1			0b00000010
#define PIN2			0b00000100 
#define PIN3			0b00001000
#define PIN4 			0b00010000
#define PIN5 			0b00100000
#define PIN6 			0b01000000
#define PIN7 			0b10000000

#define ALL_PINS_OUTPTUS		0x00
#define ALL_PINS_INPUTS			0xFF

#define ALL_PINS_LOW			0x00
#define ALL_PINS_HIGH			0xFF

void setup(){
	  // put your setup code here, to run once:
	Serial.begin(9600);
	Serial.println("Qwiic GPIO Example 2- Read GPIO");
	Wire.begin();

	testForConnectivity();
	
	setPinMode(qwiicGpioAddress, 7, SET_INPUT);

	pinMode(8, INPUT);
	digitalWrite(8, HIGH); // Internal pull up
	
	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);
}

void loop(){
	while(digitalRead(8) == LOW)){
		Serial.println("interrupt occurred");
		delay(250);
	}
}





/*
	setPinMode(byte address, byte pin, byte direction)
	
	This function will set a single pin on the Qwiic GPIO as input or output. 
	Other pins remained unchanged. 
	
	pin directions: SET_OUTPUT 0x00 and SET_INPUT 0X01
	
	Inputs: Qwiic Gpio Address, pin: 0 - 7, and pin direction
	
		similar to pinMode()

*/
void setPinMode(byte address, byte pin, byte direction){
	Wire.beginTransmission(qwiicGpioAddress); //required to be global, or pass in so if you had multiple expanders
	Wire.write(REGISTER_CONFIURATION);
	byte currentPinDirection = readRegister(address, REGISTER_CONFIURATION);

		pin =  actualPinBits(pin);
	
  if(direction == SET_INPUT){
		currentPinDirection |= pin; // pin will come in correctly masked because of a define. 
  }
  else if(direction == SET_OUTPUT){
		currentPinDirection &= ~(pin);  
  }
  
  		writeRegister(address, REGISTER_CONFIURATION, currentPinDirection );
}

/*
	readPin(byte address, byte pin);
	The pin is the corresponding pin on the Qwiic GPIO: 0 through 7.
	read a specific pin of the GPIO, returns pin state. 
	
	Inputs: address of the Qwiic GPIO and the pin to read: 0 through 7
	
	similar to digitalRead()
*/
byte readPin(byte address, byte pin){
	byte currentRegisterValue = readRegister( address, REGISTER_INPUT_PORT);

	switch(pin){
		case 0: 
		currentRegisterValue &= BIT0; 
		currentRegisterValue >>= 0;
		return currentRegisterValue; 
		
		case 1: 
		currentRegisterValue &= BIT1; 
		currentRegisterValue >>= 1;
		return currentRegisterValue; 
		
		case 2: 
		currentRegisterValue &= BIT2; 
		currentRegisterValue >>= 2;
		return currentRegisterValue; 
		
		case 3: 
		currentRegisterValue &= BIT3;
		currentRegisterValue >>= 3;
		return currentRegisterValue; 
		
		case 4: 
		currentRegisterValue &= BIT4; 
		currentRegisterValue >>= 4;
		return currentRegisterValue; 

		case 5: 
		currentRegisterValue &= BIT5; 
		currentRegisterValue >>= 5;
		return currentRegisterValue; 
		
		case 6: 
		currentRegisterValue &= BIT6; 
		currentRegisterValue >>= 6;
		return currentRegisterValue; 
		
		case 7: 
		currentRegisterValue &= BIT7; 
		currentRegisterValue >>= 7;
		return currentRegisterValue; 
	
		default:
		return 53; //INVALID
		break;
	
	}	
}

/* testForConnectivity() Checks for an ACK from the Qwiic GPIO
*/
void testForConnectivity() {
  Wire.beginTransmission(qwiicGpioAddress);
  //check here for an ACK from the slave, if no ack don't allow change?
  if (Wire.endTransmission() != 0) {
    Serial.println("Check Connections. No Qwiic GPIO found.");
    while (1);
  }
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


/*
	setPinOutput() will set a pin as high or low
	input: address of the Qwiic GPIO 
		   pin, the pin you wish to control
		   state, the logic level HIGH or LOW.
		   	
			similar to digitalWrite(pin, state)

*/
void setPinOutput(byte address, byte pin, byte state){	
	Wire.beginTransmission(address);
	Wire.write(REGISTER_OUTPUT_PORT);
	Wire.endTransmission(false);
	Wire.requestFrom(address, 1);
	
	pin =  actualPinBits(pin);
	
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
	actualPinBits takes in the desired pin in decimal and returns the bit
	pattern used for masking
*/
byte actualPinBits(byte desiredPin){
	switch(desiredPin){
		case 0:
		return PIN0;
		
		case 1:
		return PIN1;
		
		case 2:
		return PIN2;
		
		case 3:
		return PIN3;
		
		case 4:
		return PIN4;
		
		case 5:
		return PIN5;
		
		case 6:
		return PIN6;
		
		case 7:
		return PIN7;
	}
}