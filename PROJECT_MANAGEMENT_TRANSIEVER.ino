/*   This sketch is used on the controller side of VPRD (Variable Pressure Retrivial Device) system. 
 *    It uses the bidirection 2.4 GHz radio communication to control the robot and get the infomration from the robot. 
 *    The program takes values from the joystic and buttons, encrypts it to the one variable and sends the array to the reciever. 
 *    Flag is used for connected and unconnected mode. 
 *    Clean channel search algorythm is introduced in this code.
 *    Contains Programming mode which is accesable by holdong joystick and right buttons for 2 seconds.
 *    Uses HMI system to interact with user. It is possible to change the image to prefered one.
 *    Joystick button in retrivial mode is used to control the self-designed air system.
 *    Two speed modes of the robot.
 *    Auto synchonisation system of the controller and the roobot.
    by Afanasi Chihaioglo 2018
*/
#include <EEPROM.h> // Include all of the required libraries
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,20,4);
#if defined(ARDUINO) && ARDUINO >= 100 //Liquid Crystal Library set up
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif
byte logo0[8] = {0b00000,0b00000,0b00000,0b00011,0b00010,0b00011,0b00111,0b01001}; // Arrays for storing logotype 
byte logo1[8] = {0b00000,0b10000,0b10000,0b11000,0b11000,0b11000,0b11100,0b11110};
byte logo2[8] = {0b11010,0b11001,0b11010,0b11001,0b11111,0b01111,0b00111,0b00011};
byte logo3[8] = {0b11111,0b10011,0b10101,0b10011,0b10101,0b10010,0b11100,0b11000};
byte ArrowUp[8] = {B00100,B01110,B11111, B01110,B01110,B01110,B01110,B01110}; // Arrays for arrows
byte ArrowDown[8] = {B01110,B01110,B01110,B01110,B01110,B11111,B01110,B00100};

RF24 radio(9, 10); // "Create radio module on pin 9 and 10
//RF24 radio(9,53); // For Arduino Mega

byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //Pipe Numbers ( If you want to use more than one reciever/transmitter)

byte forward = 8; // Set it to the pin 8
byte back = 5;  
byte right = 6; 
byte left = 4; 
byte z = 7; // Variable used for robot's directioning (Is set to 7 to be stationary initially)
int x; // 
int y;
int f; // forward
int b; // back
int l; // left
int r;//right
int t1; //initial time for programming mode entrance
int t2; //time differense programming mode entrance
int error = 5; // used for checking connection with the robot
int counter = 1;
int counter2 = 1;
byte k ; //Variable used for recieved data byte 0
int Cchannel; //Current Channel
int Ochannel; //Old Channel
const int X_pin = 0; // analog joystic X potentiometer
const int Y_pin = 1; // analog joystic Y potentiometer
byte recieved_data[2]; // Array for recieved data
byte transmit_data[2]; // Array for transmitted data
byte latest_data[2]; // Array for the latest data (Used in Clear Channel algorythm) 
boolean flag; // Connectivity flag
boolean flag1; // Clean channel flag
int screenflag; // Flag for HMI
int SW; // Joystick switch
int vacflag; // Flag for air system control.
int FuncButton = 0; //Variable to store Function button state.

void setup() {
  Serial.begin(9600); //Open Serial port

  pinMode(forward, INPUT_PULLUP);
  pinMode(back, INPUT_PULLUP);
  pinMode(right, INPUT_PULLUP);
  pinMode(left, INPUT_PULLUP);
  pinMode(7,INPUT_PULLUP); //  Attach Function button
  pinMode(3,INPUT_PULLUP); // Attach Button on the joystic
  
 //----------------------------------------- Radio preferences setup ------------------------------------------------
  
  radio.begin(); //activate the module
  radio.setAutoAck(0);         //Recieved data acknowledgement mode (1 for ON, 0 for OFF)
  radio.enableAckPayload();    //Allow data to be sent if data is recieved (is used if top line os ON)
  radio.setPayloadSize(32);     //size of the packet in bytes (is used if top mode is ON)

  radio.openWritingPipe(address[0]); //Set the address to Transmit the data 
  radio.openReadingPipe(1, address[1]); // Set the adress to Recieve the data    

  
  Cchannel = EEPROM.read(1); // Retrieve the last channel used from internal memory of the Arduino
   radio.setChannel(Cchannel); // Set the communication channel number ( Have to be same on both transmitter and reciever)
  

  radio.setPALevel (RF24_PA_LOW); //Power level of the module. Choose from RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX . The higher power level means the larger distance range where the robot can opperate.
  radio.setDataRate (RF24_250KBPS); //Data rate. Choose from RF24_2MBPS, RF24_1MBPS, RF24_250KBPS . Use higher data rate if you are using longer transmit/recieve array.
                                   // Top two settings have to be same on the transmitter and reciever.

  radio.powerUp(); //Start operation flow
  flag = 1; // Set Connection flag to 1
  flag1 = 0; // Set Clean channel flag to 0
  screenflag = 0; // Set screen flag to 0
    
lcd.init(); // Activate the LCD screen 
  lcd.backlight(); 
   lcd.createChar(0, logo0); // Convert logotype and arrow arrays into chars which will be displayed.
  lcd.createChar(1, logo1);
  lcd.createChar(2, logo2);
  lcd.createChar(3, logo3);
  lcd.createChar(4, ArrowUp);
  lcd.createChar(5, ArrowDown);
  
}

void loop() {
  
  
  //----------------------------UNCONNECTED MODE WRITE -----------------------
  
  delay(5); 
  if (flag == 1){    
    
    radio.stopListening(); // To transmit the data, firstly module needs to stop listening.
    transmit_data[1] = 1; // byte used to initiate the robot
    transmit_data[0] = 7; // Tell robot to remain stationary
    
    radio.write(&transmit_data, sizeof(transmit_data)); // Write the data to the address
    show_logo(); // Function used to display the loading bar
  }

  //------------------------------- CONNECTED MODE LISTEN----------------------------
   delay(5);
  
  if (flag == 0){
  
  
  radio.startListening();  // To recieve the data module needs to start listening
  byte pipeNo; // Create the byte for a pipe
  
  //Serial.println(radio.available(&pipeNo)); 
  //if (!radio.available(&pipeNo)){
  //counter = counter + 1; }
  
  
if ( radio.available(&pipeNo)) {  //Listen all pipes
    
    radio.read( &recieved_data, sizeof(recieved_data) ); //Read the data from the address
    k=recieved_data[0];
    
    
    if (recieved_data[1] >0){ // If robot responded after we sent him command to start, flag can be changed to the connected mode
      flag = 0;
      counter = 0;   
    }    
  }
  }

  
//------------------------------------------ UNCONNECTED MODE LISTEN ---------------------------------
  if (flag == 1){
   radio.startListening(); 
  byte pipeNo;
  //Serial.println(radio.available(&pipeNo));
  if ( radio.available(&pipeNo)) { 
    
    radio.read(&recieved_data, sizeof(recieved_data) ); // 
    if (recieved_data[1] >0){
      flag = 0;
      counter = 0;
    } 
  }
  }
  
  
  
  
  if (counter > 100 ){ // Counter used to identify if robot is not responding
  flag = 1;
  counter = 0;
}


  //-------------------------CONNECTED MODE WRITE---------------------
  if (flag == 0){  
  
  
  radio.stopListening();
  FuncButton=!digitalRead(7); //Read the Function button, joystic button and directional buttons state. Additionally read the analog values from the joystick
  SW = !digitalRead(3);
  x=analogRead(X_pin);
  y=analogRead(Y_pin);
  f = !digitalRead(forward); 
  b = !digitalRead(back);
  r = !digitalRead(right);
  l = !digitalRead(left);
  while (FuncButton == HIGH){ // Flag for screen Change
    switch(screenflag){
    case 1:
    screenflag = 0; // Main driving mode
    lcd.clear();
    FuncButton = LOW;
    break;
    case 0:
    screenflag = 1; // Retrivial device mode
    lcd.clear();
    FuncButton = LOW;
    break;
  }
   
  }
  if (screenflag == 0){ //Main driving mode
  
  // Values are interpreted from analog values of the joystck and encrypted in one variable z
  
  if (x == 1023 && y == 502){ // Forward
    z = 1;
    if (b == HIGH){
      z = 12; // Slow Forward
    }
  }
   else if ( x > 530 && y == 502){ // Forward
    z = 1;
    if (b == HIGH){
      z = 12; // Slow Forward
    }
   }
   else if (x >530 && y == 503){ // Forward
    z = 1;
    if (b == HIGH){
      z = 12; // Slow Forward
    }
   }
   else if (x >530 && y > 350 && y <850){ // Forward
    z = 1;
    if (b == HIGH){
      z = 12; // Slow Forward
    }
   }
   else if( x < 490 && y > 350 && y <850){ // Back
      z=2;
      if (b == HIGH){
      z = 13; // Slow Back
    }
    }
    else if (x > 510 && y > 503){ // Slight Right
    z = 5;
    if (b == HIGH){
      z = 14; // Slow Right
    }
  }
  else if (x < 490 && y > 503){ // Slight Right
    z = 5;
    if (b == HIGH){
      z = 14; // Slow Right
    }
  }
  else if (x > 510 && y < 500){ // Slight Left
    z = 6;
    if (b == HIGH){
      z = 15; // Slow Left
    }
  }
  else if (x < 490 && y < 500){ // Slight Left
    z = 6;
    if (b == HIGH){
      z = 15; // Slow Left
    }
  }


   // Values are interpreted from digital inputs and encrypted in one variable z
   
    else if (f == HIGH){ // Forward
    z=1;
    }
    else if (b == HIGH){ // Back
     z=2;
    }
    
    else if (r == HIGH){ // Right
     z=3;
    }
    else if (l == HIGH){ // Left
     z=4;
    }
    
   
  else { // Stationary
    z = 7;
  }
 
  lcd.setCursor(4,1); // Print the infromation on the screen
  lcd.print("Main Driving");
  lcd.setCursor(4,3);
  lcd.print(Cchannel); 
  
 
  }
  
  
  
  if (screenflag == 1){ // Retrivial Device mode
   
    if (x > 530){ // Lifting arm up
    z = 8;
  }
  else if (x < 490){ // Lifting arm down
    z = 9;
  }
  
    else if (f == HIGH){ // Forward
    z=1;
    }
    else if (b == HIGH){ // Back
     z=2;
    }
    
    else if (r == HIGH){ // Right
     z=3;
    }
    else if (l == HIGH){ // Left
     z=4;
    }
    
   
  else { // Stationary
    z = 7;
  }
   if (SW == HIGH){ // If Joystick button is pressed
    switch(vacflag){// Flag for Retrivial device control 
    case 0:
    vacflag = 1;
    lcd.setCursor(6,1);
    lcd.print(" On ");
    z=10; // Turn on the Pump
    SW = LOW;
    break;
    case 1:
    vacflag = 0;
    lcd.setCursor(6,1);
    lcd.print(" Off");
    z=11; // Turn off the Pump
    SW = LOW;
    break;
    }
   
  }
  lcd.setCursor(0,0); // Print the infromation on the screen
  lcd.printByte(4); lcd.print(" Lifting Arm Up");
  lcd.setCursor(0,1);
  lcd.printByte(219);lcd.print(" Pump");
  lcd.setCursor(0,2);
  lcd.printByte(5);lcd.print(" Lifting Arm Down");
   
  }
  
  //if(!digitalRead(A2)) //button for clean channel algorythm
  //error = 3;
  
 //if (error != 5){  // Channel Interuption Check
  //  flag1 = 1;


if (flag1 == 1){ // Flag used to initiate the clean channel search
    flag = 1;
    for(int x=0; x<=3; x++){
    transmit_data[1] = 0;   // Sends the robot command to enter Unconneted mode ( to not inrefere the search)
    transmit_data[0] = 7;
    radio.write(&transmit_data, sizeof(transmit_data));
    delay(50);
    
  }
       CleanChan(); // Clean Channel Search
       flag1 = 0; // Set flag back to 0
    }
 transmit_data[0] = z; 
radio.write(&transmit_data, sizeof(transmit_data)); 
    
  }
 
 //--------------------------------------Programming mode enterance conditions---------------------------
 t1=millis(); // Take the initial time ( millis() function returns the time past after Arduino initialisation)
 SW = !digitalRead(3); 
 r = !digitalRead(right);
 while (r == HIGH && SW == HIGH ){ // If joystick button and right buttons are pressed go to while loop
 radio.stopListening();
 transmit_data[0] = 7; 
radio.write(&transmit_data, sizeof(transmit_data)); // отправить по радио
  
  t2=millis()-t1; // Find difference between initial time and time for which button is being pressed
  if (t2>2000){ // If time is more than 2 sec exit the while loop
    r = LOW;
    lcd.clear();
  }
 }
while (t2 > 2000){ // If time difference is more than two seconds then display the Program mode options
  lcd.setCursor(2,0);
  lcd.print("Programming Mode:");
  lcd.setCursor(2,1);lcd.print("Find Clear Chan ");lcd.printByte(4);
  lcd.setCursor(7,2);lcd.print("Reset Chan ");lcd.printByte(5);
  lcd.setCursor(3,3);lcd.print("To Exit press FB");
  f = !digitalRead(forward); 
  b = !digitalRead(back);
  FuncButton=!digitalRead(7);
  if (f == HIGH && b == LOW){ // If Forward button is pressed, then do the Clear Channel Search
    flag1 = 1;
    flag = 0;
    t2 = 1;
  }
  if (b == HIGH && f == LOW){ // If Back button is pressed, then reset channel to the initial one
  Cchannel = 96;
  radio.setChannel(Cchannel);
  EEPROM.write(1,Cchannel); // Save it to the internal memory
  t2 = 1;
  flag = 1; // Set the Unconnected mode
  }
  if (FuncButton == HIGH){ // If Function button is pressed, exit the Program mode
    t2 = 1;
  }
}

}


   
 

 

//   ------------------------------FUNCTIONS--------------------------
void CleanChan(){ 
  Ochannel=Cchannel; // Assign Old channel variable with current channel value
  
  for (int i=Cchannel+1; i <=127 ; i++){ // for the 96-127 range ( This range is chosen because it is the most clear band of channels)
      if (i == 127){ // If counter equals Channel 127, then start from the channel 96
        i=96;
      }
      radio.setChannel(i); // Set channel to the counter
      radio.startListening(); // Start listening
      delayMicroseconds(128);
      radio.stopListening(); // Stop listening
      if ( !radio.testCarrier() ) { //Test if a signal greater than -64dBm is present on the channel
       Cchannel=i; // If there is no signal present, then set Current channel to the counter
       break;
       }    
  delay(50);
      }
      
  
  
  radio.setChannel(Ochannel); // Set channel to the old channel
  
  radio.stopListening();
  for(int x=0; x<=100; x++){ // Transmit new channel to the robot
    transmit_data[1] = Cchannel;
    transmit_data[0] = 7;
    radio.write(&transmit_data, sizeof(transmit_data));
    delay(10);
    
  }

  radio.setChannel(Cchannel); // Set the new channel
  error = 5;
  recieved_data[1]=0; // Unconnected mode
  EEPROM.write(1,Cchannel); // Write channel number to the internal memory
   
}
      
        
void show_logo(){  // Display the loading bar consist of small logo images
   lcd.clear();
  for(int i=0;i<20;i=i+2){
   lcd.setCursor(4,0);
  lcd.print("Connecting..."); 
  lcd.setCursor(i,2);
  lcd.printByte(0);
  lcd.printByte(1);

  lcd.setCursor(i,3);
  lcd.printByte(2);
  lcd.printByte(3);
  delay(100);
  } 
    lcd.clear();
}
  



  


