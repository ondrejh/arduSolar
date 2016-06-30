#include <OneWire.h>
#include <TimerOne.h>

#define CYCLE_TIME  ((uint32_t)(10*1000))  // second
#define ON_TIME     ((uint32_t)(15*60000)) // minutes
#define OFF_TIME    ((uint32_t)(5*60000))  // minutes

OneWire ds0(10);  // on pin 10,11,12 (a 4.7K resistor is necessary)
OneWire ds1(11);
OneWire ds2(12);

const byte dec_table[16] = {0,1,1,2,3,3,4,4,5,6,6,7,8,8,9,9};

void set_output(byte out)
{
  digitalWrite(7,out?HIGH:LOW);
  digitalWrite(13,out?HIGH:LOW);
}

void decode_temp(int16_t raw, char* str)
{
  byte sig;
  int16_t loc_raw;

  // get signature
  if (raw<0) {
    sig = 1;
    loc_raw = -raw;
  }
  else {
    sig = 0;
    loc_raw = raw;
  }

  // get whole part and decimal
  int16_t whole = raw>>4;
  byte dec = dec_table[raw&0x0F];
  byte h = whole/100;
  whole -= 100*h;
  byte d = whole/10;
  whole -= 10*d;
  byte u = whole;

  if (sig) *str++='-';
  if (h) {
    *str++='0'+h;
    *str++='0'+d;
  }
  else if (d) {
    *str++='0'+d;
  }
  *str++='0'+u;
  *str++='.';
  *str++='0'+dec;
  *str++='\0';
}

void setup(void) {
  // IOs
  pinMode(7,OUTPUT); // output (active high)
  digitalWrite(7,LOW);
  pinMode(8,OUTPUT); // button led (active high)
  digitalWrite(8,LOW);
  pinMode(13,OUTPUT); // onboard led (active high)
  digitalWrite(13,LOW);
  pinMode(9,INPUT); // button (active low)

  // serial port
  Serial.begin(115200);

  // timer with interrupt
  Timer1.initialize(10000);
  Timer1.attachInterrupt(timerInterrupt);
}

void timerInterrupt(void) {

  // filter button, get rising edge
  static byte btn = 0;
  static byte btncnt = 0;
  byte btnedge = 0;
  if (btn) {
    if (digitalRead(9)!=0) {
      btncnt++;
      if (btncnt>3) {
        btncnt=0;
        btn=0;
      }
    }
    else btncnt=0;
  }
  else {
    if (digitalRead(9)==0) {
      btncnt++;
      if (btncnt>3) {
        btncnt=0;
        btn=1;
        btnedge=1;
      }
    }
    else btncnt=0;
  }

  static byte state = 0;
  static uint32_t btnTimer;

  if (btnedge) btnTimer=millis();

  switch(state) {
    case 0: // auto
      if (btnedge)
        state = 1;
      break;
    case 1: // on
      if (btnedge) // if button pressed go off
        state=2;
      if ((millis()-btnTimer)>ON_TIME) // if button not pressed for some time go auto
        state=0;
      break;
    case 2: // off
      if (btnedge) // if button pressed go on
        state=1;
      if ((millis()-btnTimer)>OFF_TIME) // if button not pressed for some time go auto
        state=0;
      break;
  }
  // use button rising edge
  if (btnedge) {
    digitalWrite(8,digitalRead(8)^1);
  }

  // output test
  /*static uint32_t outcnt = millis();
  if ((millis()-outcnt)>=5000) {
    outcnt+=5000;
    set_output(digitalRead(7)^1);
  }*/
}

void loop(void) {
  int i;
  byte present[3];//,present1=0,present2=0;
  byte data[3][12];//,data1[12],data2[12];
  byte addr0[8],addr1[8],addr2[8];
  
  int16_t raw[3];

  static uint32_t msTime = millis();

  //digitalWrite(13,HIGH);
  
  ds0.reset_search();
  ds1.reset_search();
  ds2.reset_search();

  // find sensor adresses and start conversion
  if (ds0.search(addr0)) {
    ds0.reset();
    ds0.select(addr0);
    ds0.write(0x44, 1);
  }

  if (ds1.search(addr1)) {
    ds1.reset();
    ds1.select(addr1);
    ds1.write(0x44, 1);
  }

  if (ds2.search(addr2)) {
    ds2.reset();
    ds2.select(addr2);
    ds2.write(0x44, 1);
  }

  // wait conversion done
  delay(1000);

  // send read data command
  present[0] = ds0.reset();
  ds0.select(addr0);    
  ds0.write(0xBE);

  present[1] = ds1.reset();
  ds1.select(addr1);    
  ds1.write(0xBE);

  present[2] = ds2.reset();
  ds2.select(addr2);    
  ds2.write(0xBE);

  // read data (9 bytes)
  for ( i = 0; i < 9; i++) {
    data[0][i] = ds0.read();
    data[1][i] = ds1.read();
    data[2][i] = ds2.read();
  }
  
  // test crc and conver to readable
  for (i=0; i<3; i++) {
    if (data[i][8]!=OneWire::crc8(data[i],8))
      present[i]=0;
    else {
      raw[i] = (data[i][1] << 8) | data[i][0];
      byte cfg = (data[i][4] & 0x60);
      if (cfg == 0x00) raw[i] = raw[i] & ~7;  // 9 bit resolution, 93.75 ms
      else if (cfg == 0x20) raw[i] = raw[i] & ~3; // 10 bit res, 187.5 ms
      else if (cfg == 0x40) raw[i] = raw[i] & ~1; // 11 bit res, 375 ms
    }
  }

  char str[16];
  
  Serial.print("pool ");
  if (present[0]) {
    decode_temp(raw[0],str);
    Serial.print(str);
    Serial.print("C, ");
  }
  else {
    Serial.print("---, ");
  }

  Serial.print("low ");
  if (present[1]) {
    decode_temp(raw[1],str);
    Serial.print(str);
    Serial.print("C, ");
  }
  else {
    Serial.print("---, ");
  }
  
  Serial.print("high ");
  if (present[2]) {
    decode_temp(raw[2],str);
    Serial.print(str);
    Serial.println("C");
  }
  else {
    Serial.println("---");
  }

  //digitalWrite(13,LOW);
  
  if (present[0]&&present[1]&&present[2]) {
    while ((millis()-msTime)<CYCLE_TIME); // wait 10s
    msTime+=10000;
  }
  else {
    delay(1000);
    msTime=millis();
  }
}
