
#include <Wire.h>
//#include <SPIMemory.h>
//#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <FS.h>
#include "SPIFFS.h"

//W25Q64 read/write location
#define data_add_sector  (uint32_t)0x1F0000U
#define delay_add_sector (uint32_t)0x01F000U
#define init_add_sector  (uint32_t)0x000000U
#define fade_add_sector (uint32_t)0x02F000U
#define SERVICE_UUID        "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID "0000FFE1-0000-1000-8000-00805F9B34FB"

//Global variable define
bool serFlag = 0, ackFlag = 0;
bool fadeFlag = 0;

uint8_t read_byte;
uint8_t prog;
uint8_t row, column, totalIC;


//char json_read[25];
//(36 * 3) = 108 output
Adafruit_NeoPixel slaveOutput(36, 4, NEO_RGB + NEO_KHZ800); //36 WS2811 IC, D3 Pin Number
//SoftwareSerial BLE_Serial(6, 7); // RX, TX
//SPIFlash flash;
//***************************************************************************************************************************
//char json_read[100];
uint8_t  uart_read_count, neo = 0, bit_rotate = 0;
uint8_t bytes = 4, Row = 36, Colm = 36, Lines = 5, No_IC = 9, fade, val, de_val,STH,srh,STM,stm,CUH,cuh,CMI,cmi,ENH,ENM,enh,enm;
uint8_t Buff_neo[108];
uint8_t pattern_sel, iner_pattern_sel, _R_G_B[3] = {255, 0, 0};
uint8_t _second , _minute, _hour;
uint8_t ST_MIN = 50, ST_HUR = 15, ET_MIN = 52, ET_HUR = 15, fade_count = 0;
int o,n,SP;
uint16_t valuue,vaalue;
char DT1,DT2,DT11,DT12,DT13,DT14,row1,row2,col1,col2,line1,line2,SH1,SH2,SM1,SM2,EH1,EH2,EM1,EM2;
uint8_t row_data,col_data,line_data,SH_data,SM_data,EH_data,EM_data;
int DataA1[100];
int DataA[100];
bool presentData[110] = {0}; 

bool start_flag, _erase_flag = 0, end_read, RTC_EN = 0, fade_end_flag = 0;
bool start_read = 0, flag, SYSTEM = 0, SYSTEM_ACT = 0, pre_fade_flag[108] = {0};
bool Buff_neo_check[108] = {0};
bool Buff_[108] = {0};

uint16_t  j, k, i, count;
uint16_t Rn_delay = 500, Ch_delay = 1000, data_len = 0, fade_delay = 0;
uint16_t add_delay = 0, add_inc = 0, Timer, _add;
int P=0;
/*********** Variable Def *************/
/***********Function declaration *********/
void data_extract(void);
/***********Function declaration *********/
//Adafruit_NeoPixel shift_motor(50, 3, NEO_GRB + NEO_KHZ800);
/****************************************************BLEESP******************************************************/
//bool flag = 0;
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
int z,timerr=100000;
void json_extract(void);
std::string valu;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic)
    {
      if (millis()<100000)
      {
      valu = pCharacteristic->getValue();
      json_extract();
      if (valu.length() > 0) 
      {
        for (z = 0; z < valu.length(); z++)
        Serial.print(valu[z]);
        Serial.println();
       
      }
      }
    else
      {
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->stop();
    BLEDevice::getAdvertising()->stop();
      }
      }
     
    
};
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
/****************************************************SPIFFS******************************************************/
const char* initi = "/file1.txt";
const char* Data = "/file2.txt";
const char* Delay = "/file3.txt";

//*****************************************************************************************************************
void conversion1(void);
void conversion2(void);
void conversion(void);

void setup() {
 Serial.begin(115200);
 /************************************************SPIFFS********************************************************/
  if(SPIFFS.begin())
  {
    Serial.println("SPIFFS Initialize....ok");
  }
  else
  {
    Serial.println("SPIFFS Initialization...failed");
  }
 /*************************************************ESPBLE****************************************************/
//  // Create the BLE Device
//    // Serial.println("setup ok1");
//    if(SPIFFS.format())
//    Serial.print("success");
//    else
//    {Serial.print("f");}
  BLEDevice::init("ESP32");
    // Serial.println("check");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service

  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
                       // Serial.println("setup ok2");

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  // Start the service

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  
  Serial.println("Waiting a client connection to notify...");
  /*****************************************************************************************************************/
  //shift_motor.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  //for (uint8_t neo_loop = 0; neo_loop < No_IC; neo_loop++) shift_motor.setPixelColor(neo_loop, shift_motor.Color(0, 0, 0));
  //shift_motor.show();   // Send the updated pixel colors to the hardware.
  /*********************************************************************/
  // pinMode(2, INPUT); //PIR or Microwave sensor pin
  slaveOutput.begin();
  slaveOutput.show();
  delay(500);
  //flash.begin();
  //// Serial.println(flash.getCapacity());
  //Serial.println("Ready...");
   // Serial.println("setup ok4");

  //If data available from BLE within 30 seconds,
  //then counter will reset and extract the json data,
  //otherwise LED fading function will call...
  uint8_t ackCount = 0, loopCount = 0, uart_read_count = 0;
  uint16_t delaySerial = 0;
//  while (delaySerial <= 3000) {
//    loopCount++;
//    delaySerial++;
//    while (BLE_Serial.available()) {
//      json_read[uart_read_count] = BLE_Serial.read();
//      uart_read_count++;
//      loopCount = 0;
//      if (json_read[uart_read_count - 1] == '}') {
//        delaySerial = 0;
//        Serial.println(json_read);
//        json_extract();
//        clear_buff();
//        ackCount = 0;
//        uart_read_count = 0;
//      }
//    }
//    //Sending ack to BLE if we will not get response from website
//    if (loopCount >= 50 && ackFlag == 1) {
//      Serial.println("sending ack");
//      loopCount = 0;
//      ackCount++;
//      if (ackCount >= 10) {
//        ackFlag = 0;
//        ackCount = 0;
//      }
//      BLE_Serial.write("{\"ACK\":\"OK""\"}");
//    }
//    delay(10);
//  }
  Wire.begin();
  conversion1();
  conversion2();
  conversion(); 
  //Suppose we have 105 column then total byte store in flash would be divide by 8
  if (column % 8 == 0) bytes = column / 8;
  else bytes = (column / 8) + 1;
  //we have 3 outputs in one IC so totalIC would be divide by 3
  if (column % 3) totalIC = (column / 3) + 1;
  else totalIC = (column / 3);
   //conv1(1,2,1,2)
   // Serial.println("setup ok5");
}
uint8_t conv(char msb,char lsb)
{
      uint8_t X=0,I=0;
      if(msb<90){X=msb-48;}
      else{X=msb-87;}
      if(lsb<90){I=lsb-48;}
      else{I=lsb-87;}
      return( X << 4 | I);
      // Serial.print("conv ok"); 
}
uint16_t conv1(char msb1,char lsb1,char msb2, char lsb2)
{
      uint8_t xx=0,cc=0,vv=0,bb=0;
      if(msb1<90){xx=msb1-48;}
      else{xx=msb1-87;}
      if(lsb1<90){cc=lsb1-48;}
      else{cc=lsb1-87;}
    if(msb2<90){vv=msb2-48;}
      else{vv=msb2-87;}
    if(lsb2<90){bb=lsb2-48;}
      else{bb=lsb2-87;}
      return( xx << 12 | cc << 8 | vv << 4 | bb ); 
      // Serial.print("conv1 ok");
}
void json_extract() {

  // Serial.println("not here");
 StaticJsonDocument<400> Doc;
  deserializeJson(Doc, valu);
  if (Doc["CON"] == 0) {
    pCharacteristic->setValue("{\"CON\":1}");
    pCharacteristic->notify();
  }
  else if (Doc["ERASE"] == 1) 
  {
    pCharacteristic->setValue("{\"ACK\":\"OK\"}");
    pCharacteristic->notify();
        if(SPIFFS.format())
    Serial.print("format_success");
    else Serial.print("format_failed");
    delay(10);
  }
   else if ( (Doc["EOL"] > 0 ) || (Doc["SOL"] > 0) || (Doc["SOP"] == "1") || 
           (Doc["EOP"] > 0)) 
  {
    pCharacteristic->setValue("{\"ACK\":\"OK\"}");
    pCharacteristic->notify();

  }
  else if ( (Doc["BT"] > 0) || (Doc["DT"] > 0))
  {
    File Q = SPIFFS.open(Data,"a");
    uint8_t daata= Doc["DT"];
    Q.seek(n);
    if(daata<16)
    {
      Q.print("0");
    }
    String dataa=String(daata,HEX);
    Q.print(dataa);
    //Serial.print(daata);
    n+=2;
    Q.close();
    pCharacteristic->setValue("{\"ACK\":\"OK\"}");
    pCharacteristic->notify();
  }
   
  else if (Doc["ROW"] > 0)
  { 
    File f = SPIFFS.open(initi, "w");
    File Q = SPIFFS.open(Data,"w");
    Q.close();
    Row = Doc["ROW"];
    //f.seek(P);
    String row= String(Row, HEX);
         if(Row<16)
             {
                f.print("0");
             }
   f.print(row); 
      // Serial.print(Row);

   Colm = Doc["COL"];
   f.seek(2);
   if(Colm<16)
      {
        f.write(0x30);
      }
  String colm= String(Colm,HEX);
  f.print(colm); 
     // Serial.print(Colm);

  pCharacteristic->setValue("{\"ACK\":\"OK\"}");
   pCharacteristic->notify();
 f.close();
  }
  else if(Doc["CHAR"] > 0)
  {
      File f = SPIFFS.open(initi, "a");

    Lines = Doc["CHAR"];
    f.seek(4);
     if(Lines<16)
      {
        f.write(0x30);
      }
  String lines= String(Lines,HEX);
  f.print(lines); 
 // Serial.print(Lines);
  pCharacteristic->setValue("{\"ACK\":\"OK\"}");
   pCharacteristic->notify();
   f.close();
  }
  else if(Doc["SH"] >= 0)
  {
      File f = SPIFFS.open(initi, "a");

    STH = Doc["SH"];
    f.seek(6);
     if(STH<16)
      {
        f.write(0x30);
      }

  String sth= String(STH,HEX);
  f.print(sth); 
 // Serial.print(STH);
  //Serial.print(sth);
  //Serial.print(sth);
    STM = Doc["SM"];
     f.seek(8);
     if(STM<16)
      {
        f.write(0x30);
      }
  String stm= String(STM,HEX);
  f.print(stm); 
  pCharacteristic->setValue("{\"ACK\":\"OK\"}");
   pCharacteristic->notify();
   f.close();
  }
  else if(Doc["EH"] >= 0)
  {
      File f = SPIFFS.open(initi, "a");

    ENH = Doc["EH"];
    f.seek(10);
     if(ENH<16)
      {
        f.write(0x30);
      }
  String enh= String(ENH,HEX);
  f.print(enh); 
    ENM = Doc["EM"];
    f.seek(12);
     if(ENM<16)
      {
        f.write(0x30);
      }
  String enm= String(ENM,HEX);
  f.print(enm); 
  pCharacteristic->setValue("{\"ACK\":\"OK\"}");
   pCharacteristic->notify();
   f.close();
  }
  else if(Doc["SC"] >= 0)
  {
    File d = SPIFFS.open(Delay, "a");
//   char soc = Doc["SC"];
//    d.seek(o);
//     if(soc<16)
//      {
//        d.write(0x30);
//      }
//  String SOC= String(soc,HEX);
//  d.print(SOC); 
// // Serial.print(soc);
//  o+=2;
   uint16_t rd = Doc["RD"];
    d.seek(o);
     if(rd<16)
      {
        d.write(0x30);
        d.write(0x30);
        d.write(0x30);
      }
      else if(rd<256)
      {
        d.write(0x30);
        d.write(0x30); 
      }
      else if(rd<4096)
      {
        d.write(0x30);
      }
  String rn= String(rd,HEX);
  d.print(rn); 
  //Serial.print(rd);
  o+=4;
  d.close();
    pCharacteristic->setValue("{\"ACK\":\"OK\"}");
    pCharacteristic->notify();
  }
    else if(Doc["PD"] >= 0)
  {
    File d = SPIFFS.open(Delay, "a");
    uint16_t pd = Doc["PD"];
    d.seek(o);
     if(pd<16)
      {
        d.write(0x30);
        d.write(0x30);
        d.write(0x30);
      }
      else if(pd<256)
      {
        d.write(0x30);
        d.write(0x30); 
      }
      else if(pd<4096)
      {
        d.write(0x30);
      }
  String Pd= String(pd,HEX);
  d.print(Pd); 
  o+=4;
   char fd = Doc["FD"];
    d.seek(o);
     if(fd<16)
      {
        d.write(0x30);
        d.write(0x30);
        d.write(0x30);
      }
      else if(fd<256)
      {
        d.write(0x30);
        d.write(0x30); 
      }
      else if(fd<4096)
      {
        d.write(0x30);
      }
  String Fd= String(fd,HEX);
  d.print(Fd);
  o+=4; 
  d.close();
    pCharacteristic->setValue("{\"ACK\":\"OK\"}");
    pCharacteristic->notify();
  }
  else if(Doc["FDM"] >= 0)
  {
    File d = SPIFFS.open(Delay, "a");
   uint16_t fdm = Doc["FDM"];
    d.seek(o);
     if(fdm<16)
      {
        d.write(0x30);
        d.write(0x30);
        d.write(0x30);
      }
      else if(fdm<256)
      {
        d.write(0x30);
        d.write(0x30); 
      }
      else if(fdm<4096)
      {
        d.write(0x30);
      }
  String Fdm= String(fdm,HEX);
  d.print(Fdm);
  o+=4;
  //Serial.print("fdm");
  //Serial.println(Fdm);
  d.close();
  pCharacteristic->setValue("{\"ACK\":\"OK\"}");
    pCharacteristic->notify();
  }
 
  //   File d=SPIFFS.open(Data, "r");
    //    for(i=0;i<d.size();i++) //Read upto complete file size
    //      {
    //         Serial.print((char)d.read());
    //      }
    //    Serial.println();
    //    delay(500);
    //   d.close(); 
    //  File d1=SPIFFS.open(Delay, "r");
    //    for(i=0;i<d1.size();i++) //Read upto complete file size
    //      {
    //        Serial.print((char)d1.read());
    //      }
    //    Serial.println();
    //    delay(500);
    //  d1.close(); 
    //  File d2=SPIFFS.open(initi, "r");
    //      for(i=0;i<d2.size();i++) //Read upto complete file size
    //      {
    //          Serial.print((char)d2.read());
    //      }
    //      Serial.println();
    //      delay(500);
    //  d2.close(); 

   else if ((Doc["CH"] >= 0) || (Doc["CM"] >= 0)) {
    //Writing current time from website to DS1307
    Wire.begin();
    Wire.beginTransmission(0x68);
    Wire.write(0x00);                 //reset register pointer
    Wire.write(0x7F & 0x00);          //0x7F to turn RTC ON
    Wire.write(decToBcd(Doc["CM"]));  //writing current minute
    Wire.write(decToBcd(Doc["CH"]));  //writing current hour
    Wire.endTransmission();
    flag = 1;
    pCharacteristic->setValue("{\"ACK\":\"OK\"}");
    pCharacteristic->notify();
  } 
  // Serial.println("json ok");
}

//void clear_buff() {
  //Clearing serial buffer after extract one json
//  for (uint8_t clr = 0; clr < 25; clr++) json_read[clr] = 0;
//}
void conversion1(void)
{
//File f1 = SPIFFS.open("/file21.txt","w");
//f1.print("hello");
//f1.close();
  File f = SPIFFS.open(Data,"r");
 // Serial.print((char)f.read());
  int SI=0,add_inc=0,SPACE=0;
  for(SI=0;SI<f.size();SI+=2)
  {
    f.seek(SPACE);
    DT1=f.read();
    SPACE++;
    f.seek(SPACE);
    DT2=f.read();
    SPACE++;
    DataA[add_inc]=conv(DT1,DT2);
    add_inc++;
  }     
  f.close();
  //Serial.println(DataA[3]);
}

void conversion2(void)
{
  File f = SPIFFS.open(Delay,"r");
  int SI=0,SPACE=0;
  for(SI=0;SI<f.size();SI+=4)
  {
    f.seek(SPACE);
    DT11=f.read();
    SPACE++;
    f.seek(SPACE);
    DT12=f.read();
  SPACE++;
    f.seek(SPACE);
    DT13=f.read();
  SPACE++;
    f.seek(SPACE);
    DT14=f.read();
    SPACE++;
//    Serial.println(DT11);
//    Serial.println(DT12);
//    Serial.println(DT13);
//    Serial.println(DT14);
    DataA1[SP]=conv1(DT11,DT12,DT13,DT14);
            // Serial.print(DataA1[SP]);
    SP++;
    
  }
    f.close();
    //Serial.println(DataA1[3]);
}
void conversion(void)
{
 File f=SPIFFS.open(initi,"r");
  row1=f.read();
 f.seek(1);
  row2=f.read();
  row_data=conv(row1,row2); 
 //Serial.print(row_data);
 row = row_data;
 //Serial.print(row);
  // ***********************************************
 f.seek(2);
  col1=f.read();
 f.seek(3);
  col2=f.read();
  col_data=conv(col1,col2); 
  column = col_data;
// Serial.print(column);
// ***********************************************
 f.seek(4);
  line1=f.read();
 f.seek(5);
  line2=f.read();
  line_data=conv(line1,line2);
  prog = line_data; 
// Serial.print(prog);
// ***********************************************
 f.seek(6);
  SH1=f.read();
 f.seek(7);
 SH2=f.read();
  SH_data=conv(SH1,SH2); 
// Serial.print(SH_data);
 //***********************************************
 f.seek(8);
  SM1=f.read();
 f.seek(9);
  SM2=f.read();
  SM_data=conv(SM1,SM2); 
// Serial.print(SM_data);
// ***********************************************
 f.seek(10);
  EH1=f.read();
 f.seek(11);
  EH2=f.read();
  EH_data=conv(EH1,EH2); 
// Serial.print(EH_data);
// ***********************************************
 f.seek(12);
 EM1=f.read();
 f.seek(13);
 EM2=f.read();
 EM_data=conv(EM1,EM2); 
// Serial.print(EM_data);
 // ***********************************************
 //f.seek(14);
 //char CH1=f.read();
 //f.seek(15);
 //char CH2=f.read();
 //uint8_t CH_data=conv(CH1,CH2); 
 //Serial.print(CH_data);
 // ***********************************************
 //f.seek(16);
 //char CM1=f.read();
 //f.seek(17);
 //char CM2=f.read();
 //uint8_t CM_data=conv(CM1,CM2); 
 //Serial.print(CM_data);
 f.close();
 // Serial.println("conversion ok");
}
void loop() {
  
  //If current time is between start time and end time,
  //and proximity sensor sense then the LED output will goes on...
printWall();
      //json_extract();
//          for(int ii=0;ii<sizeof(DataA);ii++) //Read upto complete file size
//      { 
//        DataA[ii]=ii;
//        Serial.print((char)DataA[ii]);
//      }

  if (!deviceConnected && oldDeviceConnected)
  {
    delay(50);
    pServer->startAdvertising();
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
    
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void printWall() {

  uint8_t pastByte[110] = {0}, presentByte[115] = {0}, futureByte[115]; //past, present, future byte for fade up and fade down
  uint8_t changeByte[115] = {0}; //fade up (past and present) and fade down (present and future) byte
  uint8_t colors[110] = {0}; //color value after fading percentage add (0 to fade % and fade % to 0)
  uint16_t fadeDelay=1, rowDelay, progDelay;
  uint8_t fadePercentage;
  //changing data (which pixel should be fade up or fade down) will store in this
  add_inc = 0; add_delay = 0; fade_count = 0;SP=0;
  for (uint8_t progCount = 1; progCount <= prog; progCount++) { 
// total program (pattern) from 1 to last pattern number
    rowDelay = DataA1[SP];//reading row delay
//      Serial.println(DataA1[SP]);
//      Serial.println(rowDelay);
  SP++;
    progDelay = DataA1[SP]; //reading program delay
//      Serial.println(DataA1[SP]);
//      Serial.println(progDelay);
  SP++;
  fadePercentage = DataA1[SP];//flash.readWord((fade_add_sector + fade_count) + 1); //reading fade delay
//      Serial.println(DataA1[SP]);
//      Serial.println(fadePercentage);
  SP++;
    fadeDelay = DataA1[SP]+1;//flash.readByte(fade_add_sector + fade_count);  //reading fading percentage
//      Serial.println(DataA1[SP]);
//      Serial.println(fadeDelay);
  SP++;


    for (uint8_t rowCount = 0; rowCount < row * 2; rowCount++)
    { 

      //function rotate twice (one for fade up and one for fade down)
      for (uint8_t byteCount = 0; byteCount < bytes; byteCount++){ //howmany times we have to read the data from flash is byteCount
        //Here changeByte is used in which, the data will store as 1 if in that pin the fading function will have to occur,
        //otherwise it will store 0 at that perticular array
        //so for that here is the simple calculation
        //fade up byte = (fast ^ present) & present
        //fade down byte = (future ^ present) & present
        if (fadeFlag == 0) { //fade Up
          presentByte[byteCount]=(DataA[add_inc]);
          futureByte[byteCount] = (DataA[add_inc+1+bytes]);
          changeByte[byteCount] = (presentByte[byteCount] ^ pastByte[byteCount]) & presentByte[byteCount];
          add_inc++; //we have to read the data of past, present and future for once for both fade up and fade down process,
          //so will update/increment the count in one function(fade up) only
        }
        else { //fade Down
         // changeByte[byteCount] = ((presentByte[byteCount] ^ futureByte[byteCount]) & presentByte[byteCount]);
         // pastByte[byteCount] = presentByte[byteCount];
         //Serial.println(byteCount);
        }
        for (uint8_t byteToBool = 0; byteToBool <= 7; byteToBool++) { //convert byte into bit
          presentData[byteToBool + (byteCount * 8)] = (changeByte[byteCount] >> byteToBool) & 0x01;
        }
      }


       // Serial.println(DataA[add_inc]);
//    Serial.println(presentByte[0]);
//      Serial.println(DataA[1]);
//  Serial.println(futureByte[0]);
//    //Serial.println(DataA1[2]);
//  Serial.println(changeByte[0]);
      uint8_t fadeP = (fadePercentage * 30) / 100; //convert fade percentage from 0-100 to 0-255
      for (long fade = 0; fade <= fadeDelay; fade++) { //function will work from 0 to fade delay
        
       // Serial.println("not here");
        //mapping the fade percentage with the fade delay
        if (fadeFlag == 0) { //fade up
          valuue = (fade * fadeP) / 100;
              if (fade == fadeDelay) fadeFlag = 1;
        } else { //fade down
          valuue = fadeP - ((fade * fadeP) / fadeDelay);
          vaalue=valuue;
              if (fade == fadeDelay) fadeFlag = 0;
        }
       
        uint8_t rgbCount = 0;
        for (uint8_t outputs = 0; outputs < totalIC; outputs++) {
          for (uint8_t lax = 0; lax <= 2; lax++) { //Ws2811 has 3 outputs in one single IC
            if (presentData[rgbCount + lax] == 1) colors[rgbCount + lax] = vaalue;
          }
          slaveOutput.setPixelColor(outputs, colors[rgbCount], colors[rgbCount + 1], colors[rgbCount + 2]);
          rgbCount = rgbCount + 3;
        }
       slaveOutput.show();
      }
      //Serial.println("6");

      delay(rowDelay); //wait till row delay
    }
      Serial.println("7");

    delay(progDelay); //wait till program delay
    fade_count += 3; //increment the fade count with 3 byte data (fade delay (2 byte) and fade percentage (1 byte))
    add_delay += 4;  //increment the delay count with 4 byte data (row delay (2 byte) and program delay (2 byte))
  }//Serial.println("printwallok");
  // Serial.println("printwallok");
}
//
//bool readRTC() {  //Reading RTC time from DS1307
//  uint8_t _second , _minute, _hour;
//  SPCR = SPCR & 0x00111111;
//  Wire.beginTransmission(0x68); //Start talking
//  Wire.write(0x00);             //Ask for Register zero
//  Wire.endTransmission();       //Complete Transmission
//  Wire.requestFrom(0x68, 3);
//  while (!Wire.available());
//  _second     = bcdToDec(Wire.read());
//  _minute     = bcdToDec(Wire.read());
//  _hour       = bcdToDec(Wire.read());
//  Wire.endTransmission();
//  //enable SPI intrrupt
//  SPCR = SPCR | 0x11000000;
//  uint8_t ST_MIN, ST_HUR, ET_MIN, ET_HUR;
//  ST_HUR = SH_data;
//  ST_MIN = SM_data;
//  ET_HUR = EH_data;
//  ET_MIN = EM_data;
//  //Serial.print(F("TIME  :_  ")); Serial.print(_hour); Serial.print(F(":")); Serial.print(_minute); Serial.print(F(":")); Serial.println(_second);
//  if ((((ST_HUR * 100) + ST_MIN) > ((ET_HUR * 100) + ET_MIN)) && (((_hour * 100) + _minute) > ((ET_HUR * 100) + ET_MIN)) && (((_hour * 100) + _minute)  >=  ((ST_HUR * 100) + ST_MIN))) return 1;
//  else if ( (((_hour * 100) + _minute)  >=  ((ST_HUR * 100) + ST_MIN)) && (((_hour * 100) + _minute)  <  ((ET_HUR * 100) + ET_MIN))) return 1;
//  else return 0;
//}

byte decToBcd(byte val) { //decimal to BCD converter
  return ((val / 10 * 16) + (val % 10));
}

byte bcdToDec(byte data) { //BCD to decimal converter
  byte MSB = 0xF0 & data;
  byte LSB = 0x0F & data;
  MSB = MSB >> 4;
  return ((10 * MSB) + LSB);
}
