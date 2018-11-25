/*
  (c)2018 Pawel A. Hernik
  YouTube video: https://youtu.be/GHULqZpVpz4
*/
// *** CONNECTIONS ***
// N5110 LCD from left:
// #1 RST      - Pin 9
// #2 CS/CE    - Pin 10
// #3 DC       - Pin 8
// #4 MOSI/DIN - Pin 11
// #5 SCK/CLK  - Pin 13
// #6 VCC 3.3V or 5V
// #7 LIGHT
// #8 GND

// DHT11 pinout from left:
// VCC DATA NC GND

// Encoder pins to 2 and 4 (uses 1st interrupt)
// Encoder push button to pin 3 (can use 2nd interrupt)
// use debouncing capacitors (100nF seems to be enough)

#define DHT11_PIN 14
#define BACKLIGHT 6

// uncomment to write min/max to EEEPROM
//#define USE_EEPROM

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROM.h>

// define USESPI in LCD driver header for HW SPI version
#include "N5110_SPI.h"
#if USESPI==1
#include <SPI.h>
#endif
N5110_SPI lcd(9,10,8); // RST,CS,DC

#include "c64enh_font.h"
#include "times_dig_16x24_font.h"
#include "term9x14_font.h"
#include "tiny3x7_font.h"
#include "small4x7_font.h"
#include "small5x7_font.h"
#include "small5x7bold_font.h"
//#include "fonts_all.h"

// -------------------------
#define encoderPinA    2
#define encoderPinB    4
#define encoderButton  3
volatile int encoderPos = 0;

void initEncoder()
{
  encoderPos=0;
  pinMode(encoderPinA,   INPUT_PULLUP); 
  pinMode(encoderPinB,   INPUT_PULLUP); 
  pinMode(encoderButton, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoderInt, CHANGE);  // encoder pin on interrupt 0 = pin 2
  attachInterrupt(digitalPinToInterrupt(encoderButton), buttonInt, CHANGE);  // encoder pin on interrupt 1 = pin 3
}

void buttonInt() {}

void readEncoderInt()
{
  //(digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? encoderPos++ : encoderPos--;
  uint8_t pd = PIND & B10100; // pins 2 and 4 direct reading
  ((pd == B10100) || (pd == B00000)) ? encoderPos++ : encoderPos--;
}

int readButton()
{
  static int lastState = HIGH;
  int v=0,state = digitalRead(encoderButton);
  if(state==LOW && lastState==HIGH) v=1;
  lastState = state;
  return v;
}

// -------------------------
int freeMemory ()
{
   extern int __heap_start, *__brkval;
   int v;
   return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
// -------------------------
long readVcc() 
{
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result; // Back-calculate AVcc in mV
  return result;
}
// -------------------------

#define DHT_OK         0
#define DHT_CHECKSUM  -1
#define DHT_TIMEOUT   -2
int humidity,hum;
int temp1,temp10;

int readDHT11(int pin)
{
  uint8_t bits[5];
  uint8_t bit = 7;
  uint8_t idx = 0;

  for (int i = 0; i < 5; i++) bits[i] = 0;

  // REQUEST SAMPLE
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(18);
  digitalWrite(pin, HIGH);
  delayMicroseconds(40);
  pinMode(pin, INPUT_PULLUP);

  // ACKNOWLEDGE or TIMEOUT
  unsigned int loopCnt = 10000;
  while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

  loopCnt = 10000;
  while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

  // READ OUTPUT - 40 BITS => 5 BYTES or TIMEOUT
  for (int i = 0; i < 40; i++) {
    loopCnt = 10000;
    while(digitalRead(pin) == LOW) if(!loopCnt--) return DHT_TIMEOUT;

    unsigned long t = micros();
    loopCnt = 10000;
    while(digitalRead(pin) == HIGH) if(!loopCnt--) return DHT_TIMEOUT;

    if(micros() - t > 40) bits[idx] |= (1 << bit);
    if(bit == 0) {
      bit = 7;    // restart at MSB
      idx++;      // next byte!
    }
    else bit--;
  }
  // bits[1] and bits[3] are always zero???
  hum    = bits[0];
  //humidity    = map(hum,33,55,50,78);
  humidity = hum;
  temp1  = bits[2];
  temp10 = bits[3];
  if(bits[4] != bits[0]+bits[1]+bits[2]+bits[3]) return DHT_CHECKSUM;
  return DHT_OK;
}

// -------------------------
// -100.0 to 199.9 to integer
void wrFloat(float f, int addr)
{
#ifdef USE_EEPROM
  unsigned int ui = (f+100)*10;
  EEPROM.write(addr+0, ui&0xff);
  EEPROM.write(addr+1, (ui>>8)&0xff);
#endif
}

float rdFloat(int addr)
{
  unsigned int ui = EEPROM.read(addr) + (EEPROM.read(addr+1)<<8);
  return (ui/10.0)-100.0;
}

// -------------------------

enum wdt_time {
	SLEEP_15MS,
	SLEEP_30MS,	
	SLEEP_60MS,
	SLEEP_120MS,
	SLEEP_250MS,
	SLEEP_500MS,
	SLEEP_1S,
	SLEEP_2S,
	SLEEP_4S,
	SLEEP_8S,
	SLEEP_FOREVER
};

ISR(WDT_vect) { wdt_disable(); }

void powerDown(uint8_t time)
{
  ADCSRA &= ~(1 << ADEN);  // turn off ADC
  if(time != SLEEP_FOREVER) { // use watchdog timer
    wdt_enable(time);
    WDTCSR |= (1 << WDIE);	
  }
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // most power saving
  cli();
  sleep_enable();
  sleep_bod_disable();
  sei();
  sleep_cpu();
  // ... sleeping here
  sleep_disable();
  ADCSRA |= (1 << ADEN); // turn on ADC
}

// --------------------------------------------------------------------------
byte scr[84*4];  // frame buffer
byte scrWd = 84;
byte scrHt = 4;

void clrBuf()
{
  for(int i=0;i<scrWd*scrHt;i++) scr[i]=0;
}

void drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  if((x < 0) || (x >= scrWd) || (y < 0) || (y >= scrHt*8)) return;
  switch (color) 
  {
    case 1: scr[x+(y/8)*scrWd] |=  (1 << (y&7)); break;
    case 0: scr[x+(y/8)*scrWd] &= ~(1 << (y&7)); break; 
    case 2: scr[x+(y/8)*scrWd] ^=  (1 << (y&7)); break; 
  }
}

void drawLineV(int x, int y0, int y1)
{
  if(y1>y0)
    for(int y=y0; y<=y1; y++) drawPixel(x,y,1);
  else
    for(int y=y1; y<=y0; y++) drawPixel(x,y,1);
}

// --------------------------------------------------------------------------

char buf[25],buf2[15];
float temp,mint=1000,maxt=-1000;
float minh=1000,maxh=-1000;

int numT=0;
int curT=0;
float bufT[300];
int mint2;
int maxt2;
int graphStart=0;

int first=1;
char *menuTxt[] = {"Temperature","Humidity","Both","Battery","Backlight","Contrast","EEPROM dump","Graph","Help","Reboot"};
int numMenus=0;
int menuLine;
int menuStart;
int numScrLines = 6;
int menuMode = -1; // -1 -> menu of options, 0..n -> option
int oldPos = 0;


void setup() 
{
  first = 1;
  Serial.begin(9600);
  lcd.init();
  lcd.clrScr();
  for(int i=0;i<14;i++) pinMode(i, OUTPUT); 
#ifdef USE_EEPROM
  mint=rdFloat(0);
  maxt=rdFloat(2);
  minh=rdFloat(4);
  maxh=rdFloat(6);
#endif
  //Serial.println(mint); Serial.println(maxt);
  if(mint<-40 || mint>100) mint=99;
  if(maxt<-40 || maxt>100) maxt=-99;
  if(minh<0 || minh>100) minh=99;
  if(maxh<0 || maxh>100) maxh=0;
  //CLKPR = 0x80; // lower internal clock frequency to save power
  //CLKPR = 0x02; // 0-16MHz, 1-8MHz, 2-4MHz, 3-2MHz, ..
  initEncoder();
  numMenus = sizeof(menuTxt)/sizeof(char*);
  analogWrite(BACKLIGHT,0); // 0=max
}

void drawBatt(int x, int y, int wd, int perc)
{
  int w = wd*perc/100;
  lcd.fillWin(x,y,1+w,1,B01111111);
  x+=w+1;
  w=wd-w;
  if(w>0) {
    lcd.fillWin(x,y,w,1,B01000001);
    x+=w;
  }
  lcd.fillWin(x++,y,1,1,B01111111);
  lcd.fillWin(x++,y,1,1,B00011100);
  lcd.fillWin(x++,y,1,1,B00011100);
}

void drawBattBig(int x, int y, int wd, int perc)
{
  int w = wd*perc/100;
  lcd.fillWin(x,y+0,1+w,1,B11111110);
  lcd.fillWin(x,y+1,1+w,1,B01111111);
  x+=w+1;
  w=wd-w;
  if(w>0) {
    lcd.fillWin(x,y+0,w,1,B00000010);
    lcd.fillWin(x,y+1,w,1,B01000000);
    x+=w;
  }
  lcd.fillWin(x,y+0,1,1,B11111110);
  lcd.fillWin(x,y+1,1,1,B01111111); x++;
  lcd.fillWin(x,y+0,1,1,B11110000);
  lcd.fillWin(x,y+1,1,1,B00001111); x++;
  lcd.fillWin(x,y+0,1,1,B11110000);
  lcd.fillWin(x,y+1,1,1,B00001111);
}

// ---------------------------------------
int t=0;
void drawSin()
{
  if(encoderPos<0) encoderPos=0;
  if(encoderPos>30) encoderPos=30;
  float mult = encoderPos-15;
  int x,y,yold;
  scrWd = 84;
  scrHt = 4;
  clrBuf();
  for(x=0; x<84; x++) {
    y = 16+mult*(sin((x+t/4.0)/5.0)*cos(x/22.0));
    if(x==0 || y==yold)
      drawPixel(x,y,1);
    else
      drawLineV(x,y,yold);
    yold = y;
  }
  lcd.drawBuf(scr,0,1,scrWd,4);
  lcd.setFont(Small5x7PLBold);
  lcd.setDigitMinWd(4);
  snprintf(buf,99,"  Mult: %2d  ",(int)mult);
  lcd.printStr(ALIGN_CENTER,0,buf);
  t+=4;
}

// ---------------------------------------
int x;
long v;

void showTemp() 
{  
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 0, "Temperature");
  buf[0]=0;
  strcat(buf," <"); dtostrf(mint,1,1,buf2); strcat(buf,buf2);
  strcat(buf,"' >"); dtostrf(maxt,1,1,buf2); strcat(buf,buf2); strcat(buf,"' ");
  lcd.printStr(ALIGN_CENTER, 5, buf);

  snprintf(buf,10,"%d",(int)temp);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  x=3;
  x=lcd.printStr(x, 1, buf);
  snprintf(buf,10,":%d",(int)temp10);
  x=lcd.printStr(x, 1, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(x+1, 1, "`C");
}

void showHum() 
{  
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 0, "Humidity");
  buf[0]=0;
  strcat(buf," <"); dtostrf(minh,1,0,buf2); strcat(buf,buf2);
  strcat(buf,"% >"); dtostrf(maxh,1,0,buf2); strcat(buf,buf2); strcat(buf,"% ");
  lcd.printStr(ALIGN_CENTER, 5, buf);

  snprintf(buf,10,"%d",humidity);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  x=lcd.printStr(22, 1, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(x+2, 2, "%");
}

void showTempHum() 
{  
  snprintf(buf,10,"%d",(int)temp);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  x=lcd.printStr(0, 0, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(x+0, 0, "`C");

  snprintf(buf,10,"%d",humidity);
  lcd.setFont(times_dig_16x24);
  lcd.setDigitMinWd(17);
  lcd.printStr(39, 3, buf);
  lcd.setFont(Term9x14);
  lcd.printStr(ALIGN_RIGHT, 3, "%");

  lcd.setFont(c64enh);
  lcd.setDigitMinWd(6);

  buf[0]=0; strcat(buf,"<"); dtostrf(mint,1,0,buf2); strcat(buf,buf2); strcat(buf,"'");
  lcd.printStr(ALIGN_RIGHT, 0, buf);
  buf[0]=0; strcat(buf,">"); dtostrf(maxt,1,0,buf2); strcat(buf,buf2); strcat(buf,"'");
  lcd.printStr(ALIGN_RIGHT, 1, buf);

  buf[0]=0; strcat(buf,"<"); dtostrf(minh,1,0,buf2); strcat(buf,buf2); strcat(buf,"%");
  lcd.printStr(ALIGN_LEFT, 4, buf);
  buf[0]=0; strcat(buf,">"); dtostrf(maxh,1,0,buf2); strcat(buf,buf2); strcat(buf,"%");
  lcd.printStr(ALIGN_LEFT, 5, buf);
}

void showBattery() 
{  
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 0, "Battery");
  lcd.setFont(c64enh);
  lcd.setDigitMinWd(6);
  dtostrf(v/1000.0,1,3,buf);
  drawBattBig(8,2,62,constrain(map(v,2900,4200,0,100),0,100));
  x=lcd.printStr(20, 5, buf);
  lcd.printStr(x+2, 5, "V");
}

void dumpEEPROM()
{
  lcd.setFont(Tiny3x7PL);
  lcd.setCharMinWd(3);
  lcd.setDigitMinWd(3);
  if(encoderPos>=(128-6)*2) encoderPos=(128-6)*2;
  int st = encoderPos/2;
  for(int j=0;j<numScrLines;j++) {
    int ii = st*8+j*8;
    ii&=0x3ff;
    snprintf(buf,8,"%03X",ii);
    lcd.printStr(0, j, buf);
    for(int i=0;i<8;i++) {
      int v = EEPROM.read(ii+i);
      snprintf(buf,8,"%02X",v);
      lcd.printStr(14+i*9, j, buf);
    }
  }
}

void showLowBatt()
{
  x=8;
  lcd.clrScr();
  lcd.setFont(Term9x14);
  lcd.printStr(ALIGN_CENTER, 0, "Low");
  lcd.printStr(ALIGN_CENTER, 2, "Battery");
  lcd.setFont(c64enh);
  lcd.setDigitMinWd(6);
  dtostrf(v/1000.0,1,3,buf);
  x=lcd.printStr(x, 5, "Vcc: ");
  x=lcd.printStr(x, 5, buf);
  lcd.printStr(x+2, 5, "V");
  powerDown(SLEEP_8S);
  powerDown(SLEEP_8S);
  // disable LCD controller and power down forever to save battery
  lcd.sleep(true);
  powerDown(SLEEP_FOREVER);
}

void showHelp()
{
  lcd.clrScr();
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 0, "Help");
  lcd.setFont(Small4x7PL);
  lcd.setCR(1);
  lcd.printStr(0, 1, "Use encoder to select menu item. Press button to exit.");
  lcd.setCR(0);
  snprintf(buf,25,"Free SRAM: %d B",freeMemory());
  lcd.printStr(ALIGN_CENTER, 5, buf);
}

void setBacklight()
{
  if(encoderPos>84) encoderPos=84;
  snprintf(buf,6," %d ",encoderPos*255/84);
  lcd.setFont(Small5x7PL);
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  lcd.printStr(ALIGN_CENTER, 1, buf);
  lcd.printStr(ALIGN_LEFT, 1, "000");
  lcd.printStr(ALIGN_RIGHT, 1, "255");
  lcd.fillWin(0,2,encoderPos,1,0xfc);
  if(encoderPos<84) lcd.fillWin(encoderPos,2,84-encoderPos,1,0);
  analogWrite(BACKLIGHT,255-encoderPos*3);
}

void setContrast()
{
  if(encoderPos>63) encoderPos=63;
  snprintf(buf,6,"%02X",encoderPos*2);
  lcd.setFont(Small5x7PL);
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  lcd.printStr(28, 1, buf);
  lcd.printStr(ALIGN_LEFT, 1, "00");
  lcd.printStr(58, 1, "7F");
  lcd.fillWin(0,2,encoderPos,1,0xfc);
  if(encoderPos<84) lcd.fillWin(encoderPos,2,84-encoderPos,1,0);
  lcd.setContrast(encoderPos*2);
}

void (*doReset)(void) = 0;
void reboot()
{
  if(encoderPos>=1*2) encoderPos=1*2;
  int st = encoderPos/2;
  lcd.setFont(c64enh);
  lcd.printStr(ALIGN_CENTER, 1, "Reboot?");
  lcd.setInvert(st?0:1);
  lcd.printStr(10, 3, " NO ");
  lcd.setInvert(st?1:0);
  lcd.printStr(43, 3, " YES ");
  lcd.setInvert(0);
  if(readButton()<=0) return;
  menuMode=-1;
  lcd.clrScr();
  if(st>0) { // yes
    lcd.printStr(ALIGN_CENTER, 2, "Rebooting ..."); delay(500); 
    lcd.clrScr();
    doReset();
  }
  encoderPos=oldPos; 
}

void setMenu(int m)
{
  menuMode=m;
  lcd.clrScr();
  oldPos=encoderPos;
  encoderPos=0;
}

void endMenu()
{
  if(readButton()>0) {
    menuMode=-1;
    lcd.clrScr();
    encoderPos=oldPos; 
  }
}

void formatMenu(char *in, char *out, int num)
{
  int j=strlen(in);
  out[0]=' ';
  strncpy(out+1,in,j++);
  for(;j<num;j++) out[j]=' ';
  out[j]=0;
}

/*
// faster but less acurate
void drawMenuSlider()
{
  int n = numScrLines*menuLine/numMenus;
  lcd.fillWin(83,0,1,6,0);
  lcd.fillWin(82,0,1,6,0xff);
  lcd.fillWin(81,0,1,6,0);
  lcd.fillWin(81,n,1,1,B01111100);
  lcd.fillWin(83,n,1,1,B01111100);
}
*/

void drawMenuSlider()
{
  int y, n = (8*numScrLines-2-5-2)*menuLine/(numMenus-1);
  scrWd = 3;
  scrHt = numScrLines;
  clrBuf();
  for(y=0; y<numScrLines*8; y++) drawPixel(1,y,1);
  for(y=0; y<5; y++) { drawPixel(0,y+n+2,1); drawPixel(2,y+n+2,1); }
  lcd.drawBuf(scr,81,0,scrWd,scrHt);
}

void handleMenu()
{
  //lcd.setFont(Small5x7PLBold);
  lcd.setFont(Small5x7PL);
  lcd.setCharMinWd(5);
  lcd.setDigitMinWd(5);
  if(encoderPos<0) encoderPos=0;
  if(menuMode==-1) {
    menuLine = encoderPos/2;
    if(menuLine>=numMenus) { menuLine=numMenus-1; encoderPos=menuLine*2; }
    if(menuLine>=menuStart+numScrLines) menuStart=menuLine-numScrLines+1;
    if(menuLine<menuStart) menuStart=menuLine;
    for(int i=0;i<numScrLines;i++) {
      if(i+menuStart<numMenus) {
        lcd.setInvert(i+menuStart==menuLine ? 1 : 0);
        formatMenu(menuTxt[i+menuStart], buf, 13);
        lcd.printStr(ALIGN_LEFT, i, buf);
      }
    }
    drawMenuSlider();
    if(readButton()) {
      setMenu(menuLine);
      if(menuLine==4) encoderPos=84;
      if(menuLine==5) encoderPos=0x30/2;
    }
  } else
  if(menuMode==0) { showTemp(); endMenu(); } else
  if(menuMode==1) { showHum();  endMenu(); } else
  if(menuMode==2) { showTempHum(); endMenu(); } else
  if(menuMode==3) { showBattery(); endMenu(); } else
  if(menuMode==4) { setBacklight(); endMenu(); } else
  if(menuMode==5) { setContrast(); endMenu(); } else
  if(menuMode==6) { dumpEEPROM(); endMenu(); } else
  if(menuMode==7) { drawSin(); endMenu(); } else
  if(menuMode==8) { showHelp(); endMenu(); } else
  if(menuMode==9) { reboot(); endMenu(); } else
  { menuMode=-1; lcd.clrScr(); } 
}

void loop() 
{
  v=readVcc();
  if(v<2900) showLowBatt();

  int ret = readDHT11(DHT11_PIN);   // only positive values - room temperatures
  temp = abs(temp1+temp10/10.0);

  if(ret==DHT_OK) {
    if(temp<mint) { mint=temp; wrFloat(mint,0); }
    if(temp>maxt) { maxt=temp; wrFloat(maxt,2); }
    if(humidity<minh) { minh=humidity; wrFloat(minh,4); }
    if(humidity>maxh) { maxh=humidity; wrFloat(maxh,6); }
    if(first) {
#ifndef USE_EEPROM
      mint=maxt=temp;
      minh=maxh=humidity;
#endif
      first=0;
      lcd.clrScr();
    }
  }
  if(first && ret!=DHT_OK) {
    lcd.clrScr();
    lcd.setFont(Term9x14);
    lcd.printStr(ALIGN_CENTER, 1, "Sensor");
    lcd.printStr(ALIGN_CENTER, 3, "init");
    powerDown(SLEEP_2S);
    return;
  }

  handleMenu();
  //powerDown(SLEEP_2S);
}

