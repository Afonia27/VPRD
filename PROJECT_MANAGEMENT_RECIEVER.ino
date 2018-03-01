/*   This sketch is used on the robot side of VPRD (Variable Pressure Retrivial Device) system. 
 *    It uses the bidirection 2.4 GHz radio communication to recieve comand from the controller and send data to it.
 *    The program takes encrypted value of recieved variable from the array and sends pulses to the servos.
 *    Flag is used for connected and unconnected mode. 
 *    Clean channel search algorythm is introduced in this code.
 *    Two speed modes of the robot.
 *    Solenoid valve is cotrolled using the BJT transistor
 *    Two resistors are used to workout the battery level of the robot
 *    Auto synchonisation system of the controller and the roobot(connected/unconnected modes).
 *    The operating channel can be reseted by holding reset button for 2 seconds
    by Afanasi Chihaioglo 2018
*/
#include <EEPROM.h> // Include all of the required libraries
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Servo.h>

RF24 radio(9, 10); // Create radio module on pin 9 and 10
//RF24 radio(9,53); // For Arduino Mega

byte recieved_data[2]; // Array for recieved data
byte transmit_data[2]; // Array for transmitted data

int z ; // Variable used for robot's directioning
byte k = 0; // Byte used to acknowledge the connection
int l; // This line will be deleted
int channel; // Variable to store the operating channel
const int buzzer = 8; // This line will be deleted
boolean flag; // Connectivity flag
int button; // Variable to store button state for reseting the channel
int t1; // Initial time after pressing the button
int t2; // Time difference
Servo myservo1; // Servo for driving wheel 
Servo myservo2; // Servo for driving wheel
Servo myservo3; // Servo for lifting arm

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //Pipe Numbers ( If you want to use more than one reciever/transmitter)


void setup() {
  Serial.begin(9600); //Open Serial port
 pinMode(buzzer, OUTPUT); // This line will be deleted
  myservo1.attach(A0); // Attach servos to the analog output pins
  myservo2.attach(A1);
  myservo3.attach(A2);
  myservo1.writeMicroseconds(1500); // Set all of the servos to stationary positions
  myservo2.writeMicroseconds(1500);
  myservo3.writeMicroseconds(1500);
  pinMode(6, INPUT_PULLUP); // Attach button for channel reset

 
//----------------------------------------- Radio preferences setup ------------------------------------------------
  
  radio.begin(); //activate the module
  radio.setAutoAck(0);         //Recieved data acknowledgement mode (1 for ON, 0 for OFF)
  radio.enableAckPayload();    //Allow data to be sent if data is recieved (is used if top line os ON)
  radio.setPayloadSize(32);     //Size of the packet in bytes (is used if top mode is ON)

  radio.openReadingPipe(1, address[0]); // Set the adress to Recieve the data 
  radio.openWritingPipe(address[1]); //Set the address to Transmit the data
  
  channel=EEPROM.read(1); // Retrieve the last channel used from internal memory of the Arduino
  radio.setChannel(channel);  // Set the communication channel number ( Have to be same on both transmitter and reciever)

  radio.setPALevel (RF24_PA_LOW); //Power level of the module. Choose from RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX . The higher power level means the larger distance range where the robot can opperate.
  radio.setDataRate (RF24_250KBPS); //Data rate. Choose from RF24_2MBPS, RF24_1MBPS, RF24_250KBPS . Use higher data rate if you are using longer transmit/recieve array.
                                   // Top two settings have to be same on the transmitter and reciever.

  radio.powerUp(); //Start operation flow
  flag = 1; // Set Connection flag to 1
  
}

void loop() {
  
   //--------------------Programmable LISTEN ----------------------
    delay(5);
    if (flag == 1){
    radio.startListening(); // To recieve the data module needs to start listening
    
    
  byte pipeNo;// Create the byte for a pipe
  
  if ( radio.available(&pipeNo)) {  //Listen all pipes
    radio.read( &recieved_data, sizeof(recieved_data) ); //Read the data from the address
    if (recieved_data[1] == 1){ // If controller sent the command to enter connected mode, flag can be set to 1
      flag = 0;
      
    }
    }
    if (recieved_data[1] > 90){ // Used for the Clear Channel Algorythm, to recieve the clear channel from the cotroller
    channel = recieved_data[1]; 
    radio.setChannel(channel); // Set channel to new value
    EEPROM.write(1,channel); // Write new channel value to the internal memory of Arduino
 }
    } 
  //----------------------------MAIN WRITE------------------------
    delay(5);
    if (flag == 0){ // If connected mode is active
    radio.stopListening(); // To transmit the data, firstly module needs to stop listening.
    
    
    l =!digitalRead(6); // This line will be deleted
    if (l == HIGH){ // This line will be deleted
      k=1; // This line will be deleted
      
    }      // This line will be deleted
    else { // This line will be deleted
      k=0;
    }      // This line will be deleted
    
    transmit_data[0]=k; 
    transmit_data[1] = k+2; // Acknowledge the connection
    radio.write(&transmit_data, sizeof(transmit_data)); //Write data to the address
    
    }
  
  //-----------------------MAIN LISTEN-------------------------
  
  if (flag == 0){
  radio.startListening(); 
    
  byte pipeNo; 
  
  if ( radio.available(&pipeNo)) {  
    radio.read( &recieved_data, sizeof(recieved_data) );        
    
    z=recieved_data[0]; // Store command for driving as variable z
    
    // Three main options for servo control: writeMicroseconds(1100)- Servo fully clockwise
    //                                       writeMicroseconds(1500)- Servo stationary
    //                                       writeMicroseconds(1900)- Servo fully anti-clockwise
    // Values in between can be used to vary the speed
    
    if (z == 1){   // Forward
      
      myservo1.writeMicroseconds(1100);
      myservo2.writeMicroseconds(1900);
      
     
    }
    else if (z == 2){  // Back
      
      myservo1.writeMicroseconds(1900);
      myservo2.writeMicroseconds(1100);
      
    }
    else if (z == 3){ // Right
     myservo1.writeMicroseconds(1900);
     myservo2.writeMicroseconds(1900); 
      
    }
    else if (z == 4){ // Left
     myservo1.writeMicroseconds(1100);
     myservo2.writeMicroseconds(1100); 
      
    }
     else if (z == 5){ // Slight Right
     myservo1.writeMicroseconds(1700);
     myservo2.writeMicroseconds(1900); 
    }
     else if (z == 6){ // Slight Left
     myservo1.writeMicroseconds(1100);
     myservo2.writeMicroseconds(1300); 
    }
    else if (z == 8) { // Lifting Arm Up
       myservo3.writeMicroseconds(1100);
    }
    else if (z == 9){ // Lifting Arm Down
      myservo3.writeMicroseconds(1900);
    }
    else if (z == 12){ // Slow Forwards
      myservo1.writeMicroseconds(1470);
      myservo2.writeMicroseconds(1530);
      
    }
    else if (z == 13){ // Slow Backwards
      myservo1.writeMicroseconds(1530);
      myservo2.writeMicroseconds(1470);
    }
    else if (z == 14){ // Slow Right
     myservo1.writeMicroseconds(1530);
     myservo2.writeMicroseconds(1530); 
      
    }
    else if (z == 15){ // Slow Left
     myservo1.writeMicroseconds(1470);
     myservo2.writeMicroseconds(1470); 
    }
     else if (z == 7){  // Clear
      
      myservo1.writeMicroseconds(1500);
      myservo2.writeMicroseconds(1500);
      myservo3.writeMicroseconds(1500);
    }
    }
    if (recieved_data[1] == 0){ // If controller sent the command to exit connected mode, raise the flag
      flag = 1;
    }
      
    }
  button = !digitalRead(6); // If reset button is pressed
  t1=millis(); // Take the initial time ( millis() function returns the time past after Arduino initialisation)
 while (button == HIGH){
  t2=millis()-t1; // Find the time differense
  if (t2>2000){ // If time differense is more than 2 seconds
  channel = 96; // Reset the channel
  radio.setChannel(channel);
  EEPROM.write(1,channel); 
  button = LOW;
  flag = 1; // Raise the flag to set robot to unconnected mode
    
  }
 }
  

 
 }
 

  



