/*
  This is the first attempt at communicating with Qwiic Gpio.

  Created May, 4th 2018

  Author: Kevin Kuwat
*/

#include <Wire.h>
byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2 


/*================================= DEFINES FOR DEV =================================*/
/*===================================================================================*/
//#define GPIO_OUTPUT_ON_OFF			//MANUALLY TRIGGER OUTPUT.
//#define READINPUT						//read the pin state register
//#define TEST_SET_PIN_DIRECTION			//tests ability to set a pin as output or input. reads the values off, capture with Logic Analyzer 
#define TEST_SET_OUTPUT					//test the ability to set a pin as high or low. 

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
#define PIN1			0b00000001
#define PIN2			0b00000010
#define PIN3			0b00000100 //Remember everything has to be offset by 1 zero indexed. 
#define PIN4			00b0001000
#define PIN5 			0b00010000
#define PIN6 			0b00100000
#define PIN7 			0b01000000
#define PIN8 			0b10000000



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




//digitalWrite(pin, and high low value)
//pinMode(pin , DIRECTION);
/*
	This function will set a pin on the Expander as input or output. 
	Takes in a pin number and a direction
	uses defines SET_OUTPUT 0x00 and SET_INPUT 0X01
	pin1 are also defines
*/
void setPinMode(byte address, byte pin, byte direction){
	Wire.beginTransmission(qwiicGpioAddress); //required to be global, or pass in so if you had multiple expanders
	Wire.write(REGISTER_CONFIURATION);
  
	byte currentPinDirection = readRegister(address, REGISTER_CONFIURATION);

  //don't want to change all the other pin settings so do an or equals. 
  //but what if you wanted to change a pin direction? or equal wouldn't work. 
  // make 2 cases, 1 for inputs which is an or equal
		// second case is for outputs, which is a bit flip and then an or equal?
		
  //case 1 set as input: which is an or equal. 
  if(direction == SET_INPUT){
	  //get current settings and save it to local variable
	  //do appropriate bit masking
		currentPinDirection |= pin; // pin will come in correctly masked because of a define. 
	  //or equal
	  // set register.
	  
	  	writeRegister(address, REGISTER_CONFIURATION, currentPinDirection );
		/* this should work still, taken care of the function above.
		Wire.beginTransmission(address);
		Wire.write(REGISTER_CONFIURATION); 
		Wire.write(currentPinDirection);
		Wire.endTransmission();
		*/
	  //done.
  }
  else if(direction == SET_OUTPUT){
	  //this is case 2: we want to set as output
	  //need to clear just that one bit thats a clear bit  &= ~BIT0 for example
		currentPinDirection &= ~(pin);
		writeRegister(address, REGISTER_CONFIURATION, currentPinDirection );
		/*
		Wire.beginTransmission(address);
		Wire.write(REGISTER_CONFIURATION); 
		Wire.write(currentPinDirection);
		Wire.endTransmission();
		*/
		
	  
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
		   level, the logic level HIGH or LOW.
*/
void setPinOutput(byte address, byte pin, byte level){
	byte currentRegisterValue = 0;
	Wire.beginTransmission(address);
	Wire.write(REGISTER_OUTPUT_PORT);
	Wire.endTransmission(false);
	Wire.requestFrom(address, 1);
	
	currentRegisterValue = readRegister(address, REGISTER_OUTPUT_PORT);
	
	
	if(level == HIGH){
		currentRegisterValue |= pin;
	}
	else if(level == LOW){
		currentRegisterValue &= ~pin;
	}
	
	writeRegister(address, REGISTER_OUTPUT_PORT, currentRegisterValue);
}


//read the inputs
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

#ifdef TEST_SET_OUTPUT
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

  
#ifdef READINPUT
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

