#include <SoftwareSerial.h>
#include <Timer.h>
#include <stdlib.h>
#include <Adafruit_NeoPixel.h>

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
long time = 0;
long debounce = 200;


double rpm_to_disp;
int shift_light_rpm, delta, gear;
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

Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);

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

  pinMode(buttonIn, INPUT);  //initialize 
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  Serial.begin(9600);
  Serial.print("Setup");
  pinMode(TxD, OUTPUT);
  pinMode(RxD, INPUT);
  pinMode(Key, OUTPUT);
  digitalWrite(Key, LOW);
  
  ecoVersion();  //calls function that initializes shift_light_rpm and delta variables to ecofriendly values

  t.every(100, spd_rpm_calc);  //timer object function that calls spd_rpm_calc every 100ms. 
  Serial.println("Setting up Bluetooth...");  //debug code
  
  setupBlueToothConnection();
  Serial.println("OBD init");
  
  obd_init();
  Serial.println("its go time!");
  pixelClear();
}

void spd_rpm_calc(){  //function that gets the rpm and speed from odbII
  char input[30];
  char rpminput[15];
  char recvChar;
  boolean prompt = false;
  boolean rpmprompt = false;
  int i,j;
 
  hcSerial.print("010d1");  //obd code for speed
  hcSerial.print("\r");
  delay(200);
  hcSerial.print("010c1");  //obd code for rpm
  hcSerial.print("\r");
  
  i = 0;
  
  while (hcSerial.available() <= 0);
  while (hcSerial.available() > 0 && !(prompt)){
   
    recvChar = hcSerial.read();      //read the response from the obd and put it in recvChar array
    
    if ((i < 30)&& (recvChar != 32)){  //ascii char 32 is a space character
      input[i]=recvChar;
      i=i+1;
    }
    
  }
  
  delay(125);
    input[9] = 'x';  //when converting from base16 to base 10, formatting '0x00' allows easy conversion with function strtol
    char *ptr1 = &input[8];
    char *ptrblank = &input[13];
    char **ptr2 = &ptrblank;
    input[26] = 'x';
    char *ptr3 = &input[25];
    char *ptrblank2 = &input[28];
    char **ptr4 = &ptrblank2;
    rpm = strtol(ptr1, ptr2, 16)/4;  //converts 'AxAA' from base 16 to base 10, and divides by 4 to get actual rpm value
    spd = strtol(ptr3, ptr4, 16) / 1.61 + 1; //onverts 'AxAA' from base 16 to base 10, and divides by 1.61km/mi to get mph.
    gearIndicator();
}

void send_OBD_cmd(char *obd_cmd){  //function used to initialize the obd using specific codes
  char recvChar;
  boolean prompt;
  int retries;

  if (!(obd_error_flag)){

    prompt = false;
    retries = 0;
    while ((!prompt) && (retries < OBD_CMD_RETRIES)){
      hcSerial.print(obd_cmd);
      hcSerial.print("\r");

      while (hcSerial.available() <= 0);

      while ((hcSerial.available() > 0) && (!prompt)){
        recvChar = hcSerial.read();
        Serial.print(recvChar);
        if (recvChar == 62) prompt = true;  //ascii char 62 is '>', meaning the obd is waiting for a request
      }
      Serial.println("");
      retries = retries + 1;
      
      delay(1000);
    }
    if (retries>=OBD_CMD_RETRIES) {  //if the obd isnt responding/not connected, call bluetooth setup again
      Serial.println("send failed...retrying:");
      pixelError();
      delay(500);
      setupBlueToothConnection();
    }
  }
}

void obd_init(){  //specific commands sent to obd for initialization
  pixelObdSetup();
  obd_error_flag = false;
  send_OBD_cmd("ATZ");
  delay(1000);
  send_OBD_cmd("ATSP0");
  delay(1000);
  send_OBD_cmd("0100");
  delay(1000);
  send_OBD_cmd("0120");
  delay(1000);
  send_OBD_cmd("0140");
  delay(1000);
  send_OBD_cmd("010C1");
  delay(1000);
  
}

void setupBlueToothConnection()  //hc-05 setup and connecting to obdII
{
  pixelBlueToothSetup();
  bt_error_flag = false;

  ATmode();  //allows at commands to be sent to hc-05
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

void ATmode(){  //when key pin is high, at commands are allowed to be sent. 
  hcSerial.flush();
  delay(1000);
  digitalWrite(Key, HIGH);
  delay(1000);
  hcSerial.begin(38400);
}

void Commode(){  //key low, allowing communication between hc-05 and obd
  hcSerial.flush();
  delay(1000);
  digitalWrite(Key, LOW);
  delay(1000);
  hcSerial.begin(38400);
}

void sendATCommand(char *command)  //AT command function
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

//  All pixels are blue when bluetooth is being setup
void pixelBlueToothSetup(){  
  Serial.println("PIXEL BLUETOOTH SETUP");
  for (int i = 0; i < 8; ++i){
    strip.setPixelColor(i, 0, 0, 128);
  }
  strip.show();
}

//  All pixels are red when obd is being setup
void pixelObdSetup(){
  Serial.println("PIXEL OBD SETUP");
  for (int i = 0; i < 8; ++i){
    strip.setPixelColor(i, 128, 0, 0);
  }
  strip.show();
}

//  Pixels flash red if an error is detected
void pixelError(){
  Serial.println("PIXEL ERROR");
  int j = 0;
  while (j < 5){
    for (int i = 0; i < 8; ++i){
      strip.setPixelColor(i, 128, 0, 0);
    }
    strip.show();
    delay(25);
    for (int i = 0; i < 8; ++i){
      strip.setPixelColor(i, 0, 0, 0);
    }
    strip.show();
    j = j+1;
  }
}
//  Clears all pixels of any color after a successful connection
void pixelClear(){
  for (int i = 0; i < 8; ++i){
    strip.setPixelColor(i, 0, 0, 0);
  }
  strip.show();
}

void ecoVersion(){
  shift_light_rpm = 25;
  delta = 1.25;               //range is 1500-2500 rpm over 8 pixels
}

void sportVersion(){
  shift_light_rpm = 40;
  delta = 2.5;                //range is 2000-4000 rpm over 8 pixels
}

//  function that displays gear selection.
void sevenSegDisplay() {
  int pin = 2;
  for (int i = 0; i < 7; ++i) {
    digitalWrite(pin, sevenSegArray[gear][i]);
    ++pin;
  }
}

void gearIndicator(){
  /*
   **Gear indicator code omitted until further testing** 
   */
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!(obd_error_flag)){
    if ((rpm>=0) && (rpm<10000)){

      /*
      Serial.print(millis()/1000);      <---------Serial data for debugging
      Serial.print(", ");                        |
      Serial.print(rpm);                         |
      Serial.print(", ");                        |
      Serial.print(spd);                         |
      Serial.print(", ");                        |
      Serial.println(gear);             <---------
      */
      rpm_to_disp = int (rpm/100);  //allows us to work in a range of 0-99, instead of 0-10000
      
      if (shift_light_rpm>0){
        //GREEN PIXELS
        if (rpm_to_disp >= shift_light_rpm-(8*delta)) {
          strip.setPixelColor(0, 0, 128, 0);
          strip.show();
        }
        else {
          
          strip.setPixelColor(0, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-(7*delta)) {
          strip.setPixelColor(1, 0, 128, 0);
          strip.show();
        }
        else {
          strip.setPixelColor(1, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-(6*delta)) {
          strip.setPixelColor(2, 0, 128, 0);
          strip.show();
        }
        else {
          strip.setPixelColor(2, 0, 0, 0);
          strip.show();
        }

        //BLUE PIXELS
        if (rpm_to_disp >= shift_light_rpm-(5*delta)) {
          strip.setPixelColor(3, 0, 0, 128);
          strip.show();
        }
        else {
          strip.setPixelColor(3, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-4*delta) {
          strip.setPixelColor(4, 0, 0, 128);
          strip.show();
        }
        else {
          strip.setPixelColor(4, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-3*delta) {
          strip.setPixelColor(5, 0, 0, 128);
          strip.show();
        }
        else {
          strip.setPixelColor(5, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm-2*delta) {
          strip.setPixelColor(6, 128, 0, 0);
          strip.show();
        }
        else {
          strip.setPixelColor(6, 0, 0, 0);
          strip.show();
        }
        if (rpm_to_disp >= shift_light_rpm) {
          strip.setPixelColor(7, 128, 0, 0);
          strip.show();
        }
        else {
          strip.setPixelColor(7, 0, 0, 0);
          strip.show();
        }
        
      }
      
      t.update();
      
      reading = digitalRead(buttonIn);  //button toggles between eco or sport version
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
    }
  }
}

