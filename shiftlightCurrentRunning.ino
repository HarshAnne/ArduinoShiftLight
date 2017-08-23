#include <SoftwareSerial.h>
#include <Timer.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel.h>

//current draw 35 + 35 + 15 + 12(32)

#define PIN 12 //arduino pin 12 used for ws2812b led strip

#define RxD 10 //arduino pin 7 to HC-05 Tx pin
#define TxD 9//arduino pin 8 to HC-05 Rx pin
#define Key 11 //key pin on hc-05, high for at mode
#define BT_CMD_RETRIES 10
#define OBD_CMD_RETRIES 6
#define RPM_CMD_RETRIES 10
#define SPD_CMD_RETRIES 10

#define buttonIn 13 //arduino pin used to control Eco or Sport versions

//button initialization
int state = HIGH;
int reading;
int previous = LOW;
//long time = 0;
long debounce = 200;

const int analogPin = A4; //potentiometer pin
int potValue = 0;
int pwmValue = 0;
const int pwmPin = 2;
long time = 0;
int dtime = 5000;
int timeFlag = 0;
int ledVersion = 0;

double rpm_to_disp;
int shift_light_rpm, delta, gear;
int start_rpm = 15;
double spd, rpm;
boolean bt_error_flag;
boolean obd_error_flag;
boolean rpm_error_flag;
boolean spd_error_flag;
boolean ecoOrSport;

SoftwareSerial hcSerial(RxD, TxD);

Timer t;

// NEOPIXEL SETUP
// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number
// Parameter 3 = pixel type flags, add them as needed
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
int pixNum = 12; //Number of pixels

Adafruit_NeoPixel strip = Adafruit_NeoPixel(pixNum, PIN, NEO_GRB + NEO_KHZ800);

                          //7,8,9,10,11,12,13 (arduino pins)
int sevenSegArray[7][7] = {{0,0,1,0,1,0,1},//n
                            {0,1,1,0,0,0,0},//1
                            {1,1,0,1,1,0,1},//2
                            {1,1,1,1,0,0,1},//3
                            {0,1,1,0,0,1,1},//4
                            {1,0,1,1,0,1,1},//5
                            {1,0,1,1,1,1,1},//6
                           };

void setup() {
  strip.begin();
  strip.show();       //makes sure all led's are off

  time = millis();
  pinMode(buttonIn, INPUT);
  pinMode(ledVersion,OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  //Serial.begin(9600);
  Serial.print("Setup");
  pinMode(TxD, OUTPUT);
  pinMode(RxD, INPUT);
  pinMode(Key, OUTPUT);
  digitalWrite(Key, LOW);
  
  ecoVersionMidOut();

  t.every(100, spd_rpm_calc);
  Serial.println("Setting up Bluetooth...");
  
  setupBlueToothConnection();
  Serial.println("OBD init");
  
  obd_init();
  Serial.println("its go time!");
  pixelClear();
}

void spd_rpm_calc(){
  char input[30];
  char rpminput[15];
  char recvChar;
  boolean prompt = false;
  boolean rpmprompt = false;
  int i,j;
 
  hcSerial.print("010d1");
  hcSerial.print("\r");
  delay(200);
  hcSerial.print("010c1");
  hcSerial.print("\r");
  
  i = 0;
  
  while (hcSerial.available() <= 0);
  while (hcSerial.available() > 0 && !(prompt)){
   
    recvChar = hcSerial.read();
    
    if ((i < 30)&& (recvChar != 32)){
      input[i]=recvChar;
      i=i+1;
      /*if(recvChar == 62){
       
      }*/
    }
    
  }
  
  delay(125);
  spd = 45;
    input[9] = 'x';
    char *ptr1 = &input[8];
    char *ptrblank = &input[13];
    char **ptr2 = &ptrblank;
    input[26] = 'x';
    char *ptr3 = &input[25];
    char *ptrblank2 = &input[28];
    char **ptr4 = &ptrblank2;
    rpm = strtol(ptr1, ptr2, 16)/4;
    spd = strtol(ptr3, ptr4, 16) / 1.61 + 1;
    gearIndicator();
}

void send_OBD_cmd(char *obd_cmd){
  char recvChar;
  boolean prompt;
  int retries;
  long startTime, currTime, changeInTime;
  changeInTime = 5000;
  if (!(obd_error_flag)){

    prompt = false;
    retries = 0;
    while ((!prompt) && (retries < OBD_CMD_RETRIES)){
      hcSerial.print(obd_cmd);
      hcSerial.print("\r");
      startTime = millis();
      while (hcSerial.available() <= 0){
        currTime = millis();
        if(currTime - startTime > changeInTime){
            hcSerial.print("\r");
            startTime = millis();
            Serial.println("wait timeout sending \r");
            
        }
        Serial.println("waiting for input");
        delay(500);
      }

      while ((hcSerial.available() > 0) && (!prompt)){
        recvChar = hcSerial.read();
        Serial.print("Received: " + recvChar);
        if (recvChar == 62) prompt = true;
      }
      Serial.println("");
      retries = retries + 1;
      
      delay(1000);
    }
    if (retries>=OBD_CMD_RETRIES) {
      Serial.println("send failed...retrying:");
      pixelError();
      delay(500);
      setupBlueToothConnection();
    }
  }
}

void obd_init(){
  pixelObdSetup();
  obd_error_flag = false;
  Serial.println("SENT ATZ");
  send_OBD_cmd("ATZ"); //Reset
  delay(1000);
  send_OBD_cmd("ATSP0"); //automatic protocol detection
  Serial.println("SENT ATSP0");
  delay(1000);
  send_OBD_cmd("0100"); //establishes communication
  Serial.println("SENT 0100");
  delay(1000);
  send_OBD_cmd("0120");
  Serial.println("SENT 0120");
  delay(1000);
  send_OBD_cmd("0140");
  Serial.println("SENT 0140");
  delay(1000);
  send_OBD_cmd("010C1");
  Serial.println("SENT 010C1");
  delay(1000);
  
}

void setupBlueToothConnection()
{
  pixelBlueToothSetup();
  bt_error_flag = false;

  ATmode();
  Serial.println("ATmode");
  delay(500);

  sendATCommand("RESET");
  Serial.println("RESET");
  delay(1000);
  sendATCommand("ORGL");
  Serial.println("ORGL");
  sendATCommand("ROLE=1");
  Serial.println("ROLE=1");
  sendATCommand("CMODE=0");
  Serial.println("CMODE=0");
  sendATCommand("BIND=000D,18,000001");
  Serial.println("BIND");
  sendATCommand("INIT");
  Serial.println("INIT");
  delay(1000);
  sendATCommand("PAIR=000D,18,000001,20");
  Serial.println("PAIR");
  delay(1000);
  Commode();
  Serial.println("Commode");
  delay(1000);
  Serial.println("setup bluettoth connection finished");
}

void ATmode(){
  hcSerial.flush();
  delay(1000);
  digitalWrite(Key, HIGH);
  delay(1000);
  hcSerial.begin(38400);
}

void Commode(){
  hcSerial.flush();
  delay(1000);
  digitalWrite(Key, LOW);
  delay(1000);
  hcSerial.begin(38400);
}

void sendATCommand(char *command)
{
  char recvChar;
  char str[2];
  int i,retries;
  boolean OK;

  if (!(bt_error_flag)){
    OK = false;

    while ((retries<BT_CMD_RETRIES) && (!(OK))){
      hcSerial.print("AT");
      if (strlen(command) > 1){
        hcSerial.print("+");
        hcSerial.print(command);
      }
      hcSerial.print("\r\n");

      while(hcSerial.available() <= 0 );

      i=0;
      while (hcSerial.available() > 0){
        recvChar = hcSerial.read();
        Serial.print(recvChar);
        if (i < 2){
          str[i] = recvChar;
          i=i+1;
        }
      }
      Serial.println("");
      retries = retries + 1;
      if ((str[0]=='O') && (str[1]=='K')) OK = true;
      delay (1000);
    }
    if (retries >= BT_CMD_RETRIES){
      bt_error_flag=true;
      Serial.println("Send Failed");
    }
  }
}

//  All pixels are blue which bluetooth is being setup
void pixelBlueToothSetup(){
  Serial.println("PIXEL BLUETOOTH SETUP");
  for (int i = 0; i < pixNum; ++i){
    strip.setPixelColor(i, 0, 0, 16);
  }
  strip.show();
}

void pixelObdSetup(){
  Serial.println("PIXEL OBD SETUP");
  for (int i = 0; i < pixNum; ++i){
    strip.setPixelColor(i, 16, 0, 0);
  }
  strip.show();
  Serial.println("PIXEL OBD SETUP COMPLETE");
}

//  Pixels flash red if an error is detected
void pixelError(){
  Serial.println("PIXEL ERROR");
  int j = 0;
  while (j < 5){
    for (int i = 0; i < pixNum; ++i){
      strip.setPixelColor(i, 16, 0, 0);
    }
    strip.show();
    delay(25);
    for (int i = 0; i < pixNum; ++i){
      strip.setPixelColor(i, 0, 0, 0);
    }
    strip.show();
    j = j+1;
  }
}
void pixelClear(){
  for (int i = 0; i < pixNum; ++i){
    strip.setPixelColor(i, 0, 0, 0);
  }
  strip.show();
}

void ecoVersion(){
  shift_light_rpm = 28;
  start_rpm = 15;
  delta = (shift_light_rpm-start_rpm)/pixNum;               //range is 1500-2800 rpm over 12 pixels
}

void ecoVersionMidOut() {
    digitalWrite(ledVersion,LOW);
    shift_light_rpm = 28;
    start_rpm = 15;
    delta = (shift_light_rpm-start_rpm)/6;
}

void sportVersion(){
  shift_light_rpm = 50;
  start_rpm = 25;
  delta = (shift_light_rpm-start_rpm)/pixNum;                //range is 2500-5000 rpm over 12 pixels
}

void sportVersionMidOut(){
    digitalWrite(ledVersion, HIGH);
    shift_light_rpm = 50;
    start_rpm = 25;
    delta = (shift_light_rpm-start_rpm)/6;                  //mid out syncs 2 pixels together, so 12/2 = 6
}

void sevenSegDisplay() {
  int pin = 3;
  for (int i = 0; i < 7; ++i) {
    if (pin == 9){
        digitalWrite(13, sevenSegArray[gear][i]);
    }  
    else {
        digitalWrite(pin, sevenSegArray[gear][i]);
    }
    pin++;
  }
}

void gearIndicator(){
  //base case when car is stopped in neutral
  if ((spd < 2)) {
    //car is in neutral
    gear = 0;
    sevenSegDisplay();
    //Serial.println("1st if");
  }
  else if (rpm < 1000 && (spd > 1)) {
    //car is in neutral
    gear = 0;
    sevenSegDisplay();
    //Serial.println("2nd if");
  }
  else if (rpm > 1000 && (spd > 1)) {
    //car is in gear
    if (((spd/rpm)*1000) >= 2.5 && ((spd/rpm)*1000 <= 7.5)) {
      //car is in 1st gear
      gear = 1;
      sevenSegDisplay();
      //Serial.println("gear = 1");
    }
    else if (((spd/rpm)*1000 > 7.5) && ((spd/rpm)*1000 <= 12.5)) {
      //car is in 2nd gear
      gear = 2;
      sevenSegDisplay();
    }
    else if (((spd/rpm)*1000 > 12.5) && ((spd/rpm)*1000 <= 17.5)) {
      //car is in 3rd gear
      gear = 3;
      sevenSegDisplay();
    }
    else if (((spd/rpm)*1000 > 17.5) && ((spd/rpm)*1000 <= 23)) {
      //car is in 4th gear
      gear = 4;
      sevenSegDisplay();
    }
    else if (((spd/rpm)*1000 > 23) && ((spd/rpm)*1000 <= 28.5)) {
      //car is in 5th gear
      gear = 5;
      sevenSegDisplay();
    }
    else if (((spd/rpm)*1000 > 28.5) && ((spd/rpm)*1000 <= 35)) {
      //car is in 6th gear
      gear = 6;
      sevenSegDisplay();
    }
    else {
      //if not in any of these ranges, car is in neutral
      gear = 0;
      sevenSegDisplay();
    }
  }
}

//middle out displays the leds increasing from the middle out, where 2 symmetric pixels on 
// either side are synced together. 
void middleOut() {
    if (rpm_to_disp >= shift_light_rpm-(6*delta)) {
          strip.setPixelColor(5, 0,pwmValue,0);
          strip.setPixelColor(6, 0,pwmValue,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(5, 0, 0, 0);
          strip.setPixelColor(6,0,0,0);
          strip.show();
        }
    if (rpm_to_disp >= shift_light_rpm-(5*delta)) {
          strip.setPixelColor(4, (int)(pwmValue/3),pwmValue,0);
          strip.setPixelColor(7,(int)(pwmValue/3),pwmValue,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(4, 0, 0, 0);
          strip.setPixelColor(7,0,0,0);
          strip.show();
        }
    if (rpm_to_disp >= shift_light_rpm-(4*delta)) {
          strip.setPixelColor(3, (int)(pwmValue*2/3),pwmValue,0);
          strip.setPixelColor(8,(int)(pwmValue*2/3),pwmValue,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(3, 0, 0, 0);
          strip.setPixelColor(8,0,0,0);
          strip.show();
        }
    if (rpm_to_disp >= shift_light_rpm-(3*delta)) {
          strip.setPixelColor(2, pwmValue,pwmValue,0);
          strip.setPixelColor(9,pwmValue,pwmValue,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(2, 0, 0, 0);
          strip.setPixelColor(9,0,0,0);
          strip.show();
        }
    if (rpm_to_disp >= shift_light_rpm-(2*delta)) {
          strip.setPixelColor(1, pwmValue,(int)(pwmValue/2),0);
          strip.setPixelColor(10,pwmValue,(int)(pwmValue/2),0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(1, 0, 0, 0);
          strip.setPixelColor(10,0,0,0);
          strip.show();
        }
    if (rpm_to_disp >= (shift_light_rpm-delta)) {
          strip.setPixelColor(0, pwmValue,0,0);
          strip.setPixelColor(11,pwmValue,0,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(0, 0, 0, 0);
          strip.setPixelColor(11,0,0,0);
          strip.show();
        }
    if (rpm_to_disp >= shift_light_rpm) {
        strip.setPixelColor(0,0,0,0);
        strip.setPixelColor(11,0,0,0);
        strip.show();
        delay(25);
        strip.setPixelColor(0,pwmValue,0,0);
        strip.setPixelColor(11,pwmValue,0,0);
        strip.show();
        delay(10);
    }
    else{
        strip.setPixelColor(0,0,0,0);
        strip.setPixelColor(11,0,0,0);
        strip.show();
    }
}


void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("looping");
  while (!(obd_error_flag)){
    if ((rpm>=0) && (rpm<10000)){

      gearIndicator();
      /*
      Serial.print(millis()/1000);
      Serial.print(", ");
      Serial.print(rpm);
      Serial.print(", ");
      Serial.print(spd);
      Serial.print(", ");
      Serial.println(gear);
      */
      rpm_to_disp = int (rpm/100);
      
      if (shift_light_rpm>0){
        
        //Experimental middle out shift pattern
        middleOut();
        
        /*
        
        if (rpm_to_disp >= shift_light_rpm-(12*delta)) {
          //barLength = 1;
          strip.setPixelColor(0, 0,255,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(0, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-(11*delta)) {
          //barLength = 1;
          strip.setPixelColor(1,42.5,255,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(1, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-(10*delta)) {
          //barLength = 1;
          strip.setPixelColor(2,85,255,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(2, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-(9*delta)) {
          //barLength = 1;
          strip.setPixelColor(3,127.5,255,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(3, 0, 0, 0);
          strip.show();
        }
        
        if (rpm_to_disp >= shift_light_rpm-(8*delta)) {
          //barLength = 1;
          strip.setPixelColor(4,170,255,0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(4, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-(7*delta)) {
          //barLength = 2;
          strip.setPixelColor(5,212.5,255,0);
          strip.show();
        }
        else {
          strip.setPixelColor(5, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-(6*delta)) {
          //barLength = 3;
          strip.setPixelColor(6,255,255,0);
          strip.show();
        }
        else {
          strip.setPixelColor(6, 0, 0, 0);
          strip.show();
        }

        
        if (rpm_to_disp >= shift_light_rpm-(5*delta)) {
          //barLength = 4;
          strip.setPixelColor(7,255,204,0);
          strip.show();
        }
        else {
          strip.setPixelColor(7, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-4*delta) {
          //barLength = 5;
          strip.setPixelColor(8,255,153,0);
          strip.show();
        }
        else {
          strip.setPixelColor(8, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-3*delta) {
          //barLength = 6;
          strip.setPixelColor(9,255,102,0);
          strip.show();
        }
        else {
          strip.setPixelColor(9, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-2*delta) {
          //barLength = 7;
          strip.setPixelColor(10,255,51,0);
          strip.show();
        }
        else {
          strip.setPixelColor(10, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-delta) {
          //barLength = 8;
          strip.setPixelColor(11,255,0,0);
          strip.show();
        }
        else {
          strip.setPixelColor(11, 0, 0, 0);
          strip.show();
        }
        */
        
      }
      
      t.update();
      
      
      potValue = analogRead(analogPin);
      pwmValue = map(potValue, 0, 1023, 0, 255);
      analogWrite(pwmPin, pwmValue);
      
      if (potValue < 3) {
          if (timeFlag == 0){
            time = millis();
            timeFlag++;
          }
          if (millis() - time > dtime && timeFlag != 0) {
              if (state == LOW) {
                  ecoVersionMidOut();
                  state = HIGH;
              }
              else {
                  sportVersionMidOut();
                  state = LOW;
              }
              timeFlag = 0;
          }
      }
      else {
          timeFlag = 0;
      }
      
      /*
      reading = digitalRead(buttonIn);
      if (reading == HIGH && previous == LOW && millis() - time > debounce) {
        if (state == HIGH) {
          ecoVersion();
          Serial.println("ECO VERSION");
          delay(50);
          state = LOW;
        }
        else {
          sportVersion();
          Serial.println("SPORT VERSION");
          delay(50);
          state = HIGH;
        }
        time = millis();
      }
      previous = reading;
      */
    }
  }
}/
    }
  }
}