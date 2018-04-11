/*
  This is the first attempt at communicating with Qwiic Gpio.

  Created May, 4th 2018

  Author: Kevin Kuwat
*/

#include <Wire.h>

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


/* Configuration Register:
  set to 0 for output
  set to 1 for input (default).
*/


byte const qwiicGpioAddress = 0x27; //When all jumpers are closed A0, A1, A2 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Start");
  Wire.begin();

  testForConnectivity();

  //configure port as output, so inverted logic. 
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_CONFIURATION);
  Wire.write(0x80); //first 7 are outputs, 8th is an input. (1 == input, 0 ==output) like the letters i=1, o = 0
  Wire.endTransmission();
}


//#define GPIO_OUTPUT_ON_OFF

//read the inputs
  /*
    * write to the address
    * write the register address:  REGISTER_INPUT_PORT
    * *** repeated start, end transmission(false)
    * Read from the address , get an ack back, 
    * then we can do a read with while wire.available();
  */
byte readInputs(){
  byte inputValues = 0;
  Wire.beginTransmission(qwiicGpioAddress);
  Wire.write(REGISTER_INPUT_PORT);
  Wire.endTransmission(false);
  Wire.requestFrom(qwiicGpioAddress, 1);
  
  byte count = 0;
  while(Wire.available()){
  if(count == 0){
  inputValues = Wire.read();
  }
  Wire.read(); //IGNORE remaining bytes, we only asked for 1.
  count ++;
  }
  
  return(inputValues); //use this later for bit masking.
}


void loop() {
  
#ifdef GPIO_OUTPUT_ON_OFF
  // put your main code here, to run repeatedly:
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
  #endif
  delay(100);
  
  
  

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
  Wire.write(0x7F); //HIGH
  Wire.endTransmission();
 
  delay(100);
  }  
  
}



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


