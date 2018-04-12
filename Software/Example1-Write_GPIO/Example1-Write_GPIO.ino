/*
  This is the first attempt at communicating with Qwiic Gpio.

  Created May, 4th 2018

  Author: Kevin Kuwata
*/

#include <Wire.h>
byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2 


/*================================= DEFINES FOR DEV =================================*/
/*===================================================================================*/
//#define GPIO_OUTPUT_ON_OFF			//MANUALLY TRIGGER OUTPUT.
//#define READINPUT						//read the pin state register
//#define TEST_SET_PIN_DIRECTION			//tests ability to set a pin as output or input. reads the values off, capture with Logic Analyzer 
//#define TEST_SET_OUTPUT					//test the ability to set a pin as high or low. 
#define TEST_READ_INPUT
/*===================================================================================*/


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
#define PIN1			0b00000001
#define PIN2			0b00000010
#define PIN3			0b00000100 
#define PIN4			0b00001000
#define PIN5 			0b00010000
#define PIN6 			0b00100000
#define PIN7 			0b01000000
#define PIN8 			0b10000000

#define ALL_PINS_OUTPTUS		0x00
#define ALL_PINS_INPUTS			0xFF

#define ALL_PINS_LOW			0x00
#define ALL_PINS_HIGH			0xFF

/*====================================================================================*//*====================================================================================*/
// Setup
/*====================================================================================*//*====================================================================================*/




/*====================================================================================*//*====================================================================================*/

/*
	This function will set a pin on the Expander as input or output. 
	Takes in a pin number and a direction
	uses defines SET_OUTPUT 0x00 and SET_INPUT 0X01
	pin1 are also defines
	
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
	actualPinBits takes in the desired pin in decimal and returns the bit
	pattern used for masking
*/
byte actualPinBits(byte desiredPin){
	switch(desiredPin){
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
		
		case 8:
		return PIN8;
	}
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
	
	//interpret pin number like 5 as 0b00010000 and not 0b0101
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
	readPin(byte address, byte pin);
	read a specific pin of the GPIO, returns pin read
	similar to digitalRead()
*/
byte readPin(byte address, byte pin){
	byte currentRegisterValue = readRegister( address, REGISTER_INPUT_PORT);

	switch(pin){
		case 1: 
		currentRegisterValue &= BIT0; 
		currentRegisterValue >>= 0;
		return currentRegisterValue; //FINAL RESULT 0 OR 1
		
		case 2: 
		currentRegisterValue &= BIT1; 
		currentRegisterValue >>= 1;
		return currentRegisterValue; 
		
		case 3: 
		currentRegisterValue &= BIT2; 
		currentRegisterValue >>= 2;
		return currentRegisterValue; 
		
		case 4: 
		currentRegisterValue &= BIT3;
		currentRegisterValue >>= 3;
		return currentRegisterValue; 
		
		case 5: 
		currentRegisterValue &= BIT4; 
		currentRegisterValue >>= 4;
		return currentRegisterValue; 

		case 6: 
		currentRegisterValue &= BIT5; 
		currentRegisterValue >>= 5;
		return currentRegisterValue; 
		
		case 7: 
		currentRegisterValue &= BIT6; 
		currentRegisterValue >>= 6;
		return currentRegisterValue; 
		
		case 8: 
		currentRegisterValue &= BIT7; 
		currentRegisterValue >>= 7;
		return currentRegisterValue; 
	
		default:
		return 53; //JUST FOR ERROR CHECKING RN
		break;
	
	}	
}

//read all the inputs
	/*
		* write to the address
		* write the register address:  REGISTER_INPUT_PORT
		* *** repeated start, end transmission(false)
		* Read from the address , get an ack back, 
		* then we can do a read with while wire.available();
	*/
byte readInputs(){
  byte currentRegisterValue = 0;
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_INPUT_PORT);
  Wire.endTransmission(false);
  Wire.requestFrom(qwiicGpioAddress, 1);
  
  byte count = 0;
  while(Wire.available()){
  if(count == 0){
	currentRegisterValue = Wire.read();
  }
  Wire.read(); //IGNORE remaining bytes, we only asked for 1.
  count ++;
	}
	
	return(currentRegisterValue); //use this later for bit masking.
}


#ifdef TEST_READ_INPUT
void setup(){
	  // put your setup code here, to run once:
	Serial.begin(9600);
	Serial.println("Start");
	Wire.begin();

	testForConnectivity();
	
	//set all outputs as off, inputs are unaffected
	setAllPinsState(qwiicGpioAddress, LOW);
	
	setPinMode(qwiicGpioAddress, 7, SET_OUTPUT);
	setPinMode(qwiicGpioAddress, 5, SET_OUTPUT);
	setPinMode(qwiicGpioAddress, 3, SET_OUTPUT);
	setPinMode(qwiicGpioAddress, 4, SET_OUTPUT);
	setPinMode(qwiicGpioAddress, 2, SET_OUTPUT);
	
	//set pins 6 and 8 as input
	setPinMode(qwiicGpioAddress, 6, SET_INPUT);
	setPinMode(qwiicGpioAddress, 8, SET_INPUT);
	
	//Set outputs as low
	setPinOutput(qwiicGpioAddress, 7, LOW);
	setPinOutput(qwiicGpioAddress, 5, LOW);
	setPinOutput(qwiicGpioAddress, 4, LOW);
	setPinOutput(qwiicGpioAddress, 3, LOW);
	setPinOutput(qwiicGpioAddress, 2, LOW);
}

void loop(){
	
	
	
	
	while(readPin(qwiicGpioAddress, 6) == 0){
		setPinOutput(qwiicGpioAddress, 2, HIGH);
		setPinOutput(qwiicGpioAddress, 5, HIGH);
		delay(50);
		setPinOutput(qwiicGpioAddress, 2, LOW);
		delay(50);
	}	
	
	setPinOutput(qwiicGpioAddress, 3, HIGH);
	setPinOutput(qwiicGpioAddress, 4, HIGH);
	delay(100);
	setPinOutput(qwiicGpioAddress, 3, LOW);
	setPinOutput(qwiicGpioAddress, 4, LOW);
	delay(100);
	
	while(readPin(qwiicGpioAddress, 8) == 0){
		setPinOutput(qwiicGpioAddress, 2, LOW);
		setPinOutput(qwiicGpioAddress, 5, LOW);
		setPinOutput(qwiicGpioAddress, 7, HIGH);
		delay(75);
		setPinOutput(qwiicGpioAddress, 7, LOW);
		delay(75);
	}	

	
		/*setPinOutput(qwiicGpioAddress, 4, HIGH);
		delay(200);
		setPinOutput(qwiicGpioAddress, 4, LOW);
		delay(200);
*/
/*
	Serial.print("original read register: ");
	Serial.print(currentRegisterValue, BIN);
	Serial.print("  ---- masked -----      ");
	currentRegisterValue = currentRegisterValue & PIN6;
	Serial.print(currentRegisterValue, BIN);
	Serial.print(">>>>>> shifted >>>>>       ");
	currentRegisterValue >>= 5; //one less than the pin number
	Serial.print(currentRegisterValue, BIN);
	Serial.print("  final ------>   ");
	Serial.println(currentRegisterValue, BIN);
	
	*/
	
	

}
#endif


#ifdef TEST_SET_OUTPUT

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");
  Wire.begin();

  testForConnectivity();

  //configure port as output, so inverted logic. 
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_CONFIURATION);
  Wire.write(0xf0); //arbitrarily chosen just so effect of setPinMode can be seen.
  Wire.endTransmission();
}

void loop(){
	//blink led with setPinOutput
	setPinOutput(qwiicGpioAddress, PIN1, HIGH);
	delay(100);
	
	setPinOutput(qwiicGpioAddress, PIN1, LOW);
	setPinOutput(qwiicGpioAddress, PIN2, HIGH);
	delay(100);
	
	setPinOutput(qwiicGpioAddress, PIN3, HIGH);
	setPinOutput(qwiicGpioAddress, PIN2, LOW);
	delay(100);
	
	setPinOutput(qwiicGpioAddress, PIN3, LOW);
	delay(100);
}
#endif


#ifdef GPIO_OUTPUT_ON_OFF
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");
  Wire.begin();

  testForConnectivity();

  //configure port as output, so inverted logic. 
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_CONFIURATION);
  Wire.write(0xf0); //arbitrarily chosen just so effect of setPinMode can be seen.
  Wire.endTransmission();
}

void loop(){
	delay(100);

	Wire.beginTransmission(qwiicGpioAddress);
	Wire.write(REGISTER_OUTPUT_PORT);
	Wire.write(0x00); //LOW
	Wire.endTransmission();
  
	delay(100);
	
	Wire.beginTransmission(qwiicGpioAddress);
	Wire.write(REGISTER_OUTPUT_PORT);
	Wire.write(0xFF); //HIGH
	Wire.endTransmission();
	delay(100);
}
#endif

  
#ifdef READINPUT //read input register thats it.

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");
  Wire.begin();

  testForConnectivity();

  //configure port as output, so inverted logic. 
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_CONFIURATION);
  Wire.write(0xf0); //arbitrarily chosen just so effect of setPinMode can be seen.
  Wire.endTransmission();
}


void loop() {
  byte inputs = readInputs();
  
  Serial.println("test >> ");
  Serial.print(readInputs());
  
  //if(inputs >>7 == 0){ //worked but eh
	 if(~inputs & BIT7){ //negate the read and anything thats a 1 is correct, so this is probably
	 //when you use the configuration to set up inverting so its normal like a button read
	 //or set up the button differently. 
	 delay(100);

  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_OUTPUT_PORT);
  Wire.write(0x00); //LOW
  Wire.endTransmission();
  
  delay(100);
  
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_OUTPUT_PORT);
  Wire.write(0xff); //HIGH
  Wire.endTransmission();
 
  delay(100);
  }	 
}
#endif


#ifdef TEST_SET_PIN_DIRECTION
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");
  Wire.begin();

  testForConnectivity();

  //configure port as output, so inverted logic. 
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_CONFIURATION);
  Wire.write(0xf0); //arbitrarily chosen just so effect of setPinMode can be seen.
  Wire.endTransmission();
}
void loop(){
	
	byte value;
	
	//SET
	setPinMode(qwiicGpioAddress, PIN1, SET_OUTPUT);
	//READ
	value = readRegister(qwiicGpioAddress, REGISTER_CONFIURATION);
	Serial.println( value, HEX);
	
	//SET
	setPinMode(qwiicGpioAddress, PIN8, SET_INPUT);
	//READ
	value = readRegister(qwiicGpioAddress, REGISTER_CONFIURATION);
	Serial.println(value, HEX);
	
	//force all 0
		Wire.beginTransmission(qwiicGpioAddress);
		Wire.write(REGISTER_CONFIURATION); 
		Wire.write(0XFF);
		Wire.endTransmission();
			delay(100);
}
#endif
/*====================================================================================*//*====================================================================================*/
/*================================= HELPER FUNCTION =================================*/
/*===================================================================================*/
/*===================================================================================*/
/*===================================================================================*/

// testForConnectivity() checks for an ACK from an Relay. If no ACK
// program freezes and notifies user.
void testForConnectivity() {
  Wire.beginTransmission(qwiicGpioAddress);
  //check here for an ACK from the slave, if no ack don't allow change?
  if (Wire.endTransmission() != 0) {
    Serial.println("Check Connections. No slave attached.");
    while (1);
  }
}

/*
	this function will read the register values of a given register. 
	clearly it returns a byte. because 8 bit registers
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
	writeRegister() takes the Qwiic GPIO address, the register to write to, and the value to write
*/
void writeRegister(byte address, byte registerToWrite, byte valueToWrite){
	Wire.beginTransmission(address);
	Wire.write(registerToWrite); 
	Wire.write(valueToWrite);
	Wire.endTransmission();
}

/*
	setAllPinsDirection set all as output or input
	use #define for direction: SET_INPUT or SET_OUTPUT
*/
void setAllPinsDirection(byte address, byte direction){
	if(direction == SET_OUTPUT){
		writeRegister(address, REGISTER_CONFIURATION, ALL_PINS_OUTPTUS);
	}
	else{
		writeRegister(address, REGISTER_CONFIURATION, ALL_PINS_INPUTS);
	}
}

/*
	setAllPinsState set all as HIGH or LOW	
*/
void setAllPinsState(byte address, byte state){
	if(state == HIGH){
		//writeRegister(address, REGISTER_OUTPUT_PORT, ALL_PINS_HIGH);
				writeRegister(address, REGISTER_OUTPUT_PORT, 0x00);

	}
	else{
		//writeRegister(address, REGISTER_OUTPUT_PORT, ALL_PINS_LOW);
						writeRegister(address, REGISTER_OUTPUT_PORT, 0XFF);

	}
}

/*
  Send data to slave
    - Start and address
    - Send data
    - Stop conddition

  Read data from slave
    - Start and address
    - send requested register
    - mast rx (this would be the request # of bytes)..
        - like the getFirmwareVersion() function below


  //getFirmwareVersion() returns the firmware version as a float.
  float getFirmwareVersion() {
  Wire.beginTransmission(address);
  Wire.write(__REGISTER___);
  Wire.endTransmission();

  Wire.requestFrom(qwiicRelayAddress, 2); //2 BYTES for the Version Number

//collect response.
  if (Wire.available() == 0) return 0;
  float versionNumber = Wire.read();
  versionNumber += (float)Wire.read() / 10.0;

  return (versionNumber);
  }




*/

