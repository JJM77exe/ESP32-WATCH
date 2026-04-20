#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include "time.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <esp_sleep.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "driver/ledc.h"
#include <Preferences.h>
#include <cstring>
#include <secret.h>

//data
//chars
// 8x12 bitmap font, ASCII 32-126
// each char is 12 bytes, one byte per row, MSB = leftmost pixel
static const uint8_t font8x12[][12] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 32 ' '
  {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00}, // 33 '!'
  {0x66,0x66,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 34 '"'
  {0x66,0x66,0xFF,0x66,0x66,0xFF,0x66,0x66,0x00,0x00,0x00,0x00}, // 35 '#'
  {0x18,0x7E,0xD8,0xD8,0x78,0x1E,0x1B,0x1B,0x7E,0x18,0x00,0x00}, // 36 '$'
  {0xC6,0xC6,0x0C,0x18,0x18,0x30,0x60,0xC6,0xC6,0x00,0x00,0x00}, // 37 '%'
  {0x38,0x6C,0x6C,0x38,0x76,0xDC,0xCC,0xCC,0xCC,0x76,0x00,0x00}, // 38 '&'
  {0x18,0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 39 '\''
  {0x0C,0x18,0x30,0x30,0x60,0x60,0x60,0x30,0x30,0x18,0x0C,0x00}, // 40 '('
  {0x30,0x18,0x0C,0x0C,0x06,0x06,0x06,0x0C,0x0C,0x18,0x30,0x00}, // 41 ')'
  {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00}, // 42 '*'
  {0x00,0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,0x00,0x00,0x00}, // 43 '+'
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x18,0x30,0x00}, // 44 ','
  {0x00,0x00,0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00,0x00}, // 45 '-'
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00}, // 46 '.'
  {0x03,0x06,0x06,0x0C,0x18,0x18,0x30,0x60,0x60,0xC0,0x00,0x00}, // 47 '/'
  {0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00}, // 48 '0'
  {0x18,0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00}, // 49 '1'
  {0x3C,0x66,0x06,0x06,0x0C,0x18,0x30,0x60,0x60,0x7E,0x00,0x00}, // 50 '2'
  {0x3C,0x66,0x06,0x06,0x1C,0x06,0x06,0x06,0x66,0x3C,0x00,0x00}, // 51 '3'
  {0x0C,0x1C,0x3C,0x6C,0x6C,0x7E,0x0C,0x0C,0x0C,0x0C,0x00,0x00}, // 52 '4'
  {0x7E,0x60,0x60,0x60,0x7C,0x06,0x06,0x06,0x66,0x3C,0x00,0x00}, // 53 '5'
  {0x3C,0x60,0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x3C,0x00,0x00}, // 54 '6'
  {0x7E,0x06,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30,0x30,0x00,0x00}, // 55 '7'
  {0x3C,0x66,0x66,0x66,0x3C,0x66,0x66,0x66,0x66,0x3C,0x00,0x00}, // 56 '8'
  {0x3C,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x66,0x3C,0x00,0x00}, // 57 '9'
  {0x00,0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00}, // 58 ':'
  {0x00,0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x18,0x30,0x00,0x00}, // 59 ';'
  {0x00,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00,0x00,0x00,0x00}, // 60 '<'
  {0x00,0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,0x00}, // 61 '='
  {0x00,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x00,0x00,0x00,0x00}, // 62 '>'
  {0x3C,0x66,0x06,0x06,0x0C,0x18,0x18,0x00,0x18,0x18,0x00,0x00}, // 63 '?'
  {0x3C,0x66,0xC3,0xDB,0xDB,0xDB,0xDE,0xC0,0x60,0x3E,0x00,0x00}, // 64 '@'
  {0x18,0x3C,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x66,0x00,0x00}, // 65 'A'
  {0x7C,0x66,0x66,0x66,0x7C,0x66,0x66,0x66,0x66,0x7C,0x00,0x00}, // 66 'B'
  {0x3C,0x66,0x60,0x60,0x60,0x60,0x60,0x60,0x66,0x3C,0x00,0x00}, // 67 'C'
  {0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x7C,0x00,0x00}, // 68 'D'
  {0x7E,0x60,0x60,0x60,0x7C,0x60,0x60,0x60,0x60,0x7E,0x00,0x00}, // 69 'E'
  {0x7E,0x60,0x60,0x60,0x7C,0x60,0x60,0x60,0x60,0x60,0x00,0x00}, // 70 'F'
  {0x3C,0x66,0x60,0x60,0x6E,0x66,0x66,0x66,0x66,0x3C,0x00,0x00}, // 71 'G'
  {0x66,0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x66,0x66,0x00,0x00}, // 72 'H'
  {0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00}, // 73 'I'
  {0x0E,0x06,0x06,0x06,0x06,0x06,0x06,0x66,0x66,0x3C,0x00,0x00}, // 74 'J'
  {0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x66,0x66,0x66,0x00,0x00}, // 75 'K'
  {0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00,0x00}, // 76 'L'
  {0xC3,0xE7,0xFF,0xDB,0xC3,0xC3,0xC3,0xC3,0xC3,0xC3,0x00,0x00}, // 77 'M'
  {0x66,0x76,0x7E,0x6E,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00}, // 78 'N'
  {0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00}, // 79 'O'
  {0x7C,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x60,0x60,0x00,0x00}, // 80 'P'
  {0x3C,0x66,0x66,0x66,0x66,0x66,0x66,0x6E,0x3C,0x06,0x00,0x00}, // 81 'Q'
  {0x7C,0x66,0x66,0x66,0x7C,0x6C,0x66,0x66,0x66,0x66,0x00,0x00}, // 82 'R'
  {0x3C,0x66,0x60,0x60,0x3C,0x06,0x06,0x06,0x66,0x3C,0x00,0x00}, // 83 'S'
  {0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00}, // 84 'T'
  {0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00}, // 85 'U'
  {0x66,0x66,0x66,0x66,0x66,0x3C,0x3C,0x18,0x18,0x18,0x00,0x00}, // 86 'V'
  {0xC3,0xC3,0xC3,0xC3,0xDB,0xDB,0xFF,0xE7,0xC3,0xC3,0x00,0x00}, // 87 'W'
  {0x66,0x66,0x3C,0x3C,0x18,0x3C,0x3C,0x66,0x66,0x66,0x00,0x00}, // 88 'X'
  {0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00}, // 89 'Y'
  {0x7E,0x06,0x0C,0x0C,0x18,0x18,0x30,0x60,0x60,0x7E,0x00,0x00}, // 90 'Z'
  {0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,0x00}, // 91 '['
  {0xC0,0x60,0x60,0x30,0x18,0x18,0x0C,0x06,0x06,0x03,0x00,0x00}, // 92 '\'
  {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,0x00}, // 93 ']'
  {0x18,0x3C,0x66,0xC3,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 94 '^'
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00}, // 95 '_'
  {0x18,0x18,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 96 '`'
  {0x00,0x00,0x3C,0x06,0x3E,0x66,0x66,0x66,0x3E,0x00,0x00,0x00}, // 97 'a'
  {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x7C,0x00,0x00,0x00}, // 98 'b'
  {0x00,0x00,0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,0x00,0x00}, // 99 'c'
  {0x06,0x06,0x3E,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00}, // 100 'd'
  {0x00,0x00,0x3C,0x66,0x66,0x7E,0x60,0x66,0x3C,0x00,0x00,0x00}, // 101 'e'
  {0x1E,0x30,0x30,0x7E,0x30,0x30,0x30,0x30,0x30,0x00,0x00,0x00}, // 102 'f'
  {0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00}, // 103 'g'
  {0x60,0x60,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00}, // 104 'h'
  {0x18,0x18,0x00,0x38,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00}, // 105 'i'
  {0x06,0x06,0x00,0x06,0x06,0x06,0x06,0x06,0x06,0x66,0x3C,0x00}, // 106 'j'
  {0x60,0x60,0x66,0x6C,0x78,0x6C,0x66,0x66,0x66,0x00,0x00,0x00}, // 107 'k'
  {0x38,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00}, // 108 'l'
  {0x00,0x00,0x76,0xFF,0xDB,0xDB,0xDB,0xDB,0xDB,0x00,0x00,0x00}, // 109 'm'
  {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00}, // 110 'n'
  {0x00,0x00,0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,0x00,0x00}, // 111 'o'
  {0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x7C,0x60,0x60,0x60,0x00}, // 112 'p'
  {0x00,0x00,0x3E,0x66,0x66,0x66,0x66,0x3E,0x06,0x06,0x06,0x00}, // 113 'q'
  {0x00,0x00,0x6C,0x76,0x60,0x60,0x60,0x60,0x60,0x00,0x00,0x00}, // 114 'r'
  {0x00,0x00,0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00,0x00,0x00}, // 115 's'
  {0x18,0x18,0x7E,0x18,0x18,0x18,0x18,0x18,0x0E,0x00,0x00,0x00}, // 116 't'
  {0x00,0x00,0x66,0x66,0x66,0x66,0x66,0x66,0x3E,0x00,0x00,0x00}, // 117 'u'
  {0x00,0x00,0x66,0x66,0x66,0x66,0x3C,0x3C,0x18,0x00,0x00,0x00}, // 118 'v'
  {0x00,0x00,0xC3,0xC3,0xC3,0xDB,0xFF,0xE7,0xC3,0x00,0x00,0x00}, // 119 'w'
  {0x00,0x00,0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00,0x00,0x00}, // 120 'x'
  {0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00,0x00}, // 121 'y'
  {0x00,0x00,0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,0x00,0x00}, // 122 'z'
  {0x0E,0x18,0x18,0x18,0x18,0x70,0x18,0x18,0x18,0x18,0x0E,0x00}, // 123 '{'
  {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00}, // 124 '|'
  {0x70,0x18,0x18,0x18,0x18,0x0E,0x18,0x18,0x18,0x18,0x70,0x00}, // 125 '}'
  {0x00,0x00,0x00,0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 126 '~'
};

//pin defintions
#define TFT_MISO 5
#define TFT_MOSI 6
#define TFT_SCK 4
#define DC 21
#define BL 10
#define SDCS 20
#define TOUCHCS 3
#define DISPLAYCS 0
#define BATVPIN 1
#define WAKEUPBTN GPIO_NUM_2

//screen commands
#define CMD_SWRESET  0x01
#define CMD_SLPOUT   0x11
#define CMD_COLMOD   0x3A
#define CMD_MADCTL   0x36
#define CMD_CASET    0x2A
#define CMD_PASET    0x2B
#define CMD_RAMWR    0x2C
#define CMD_DISPON   0x29
#define CMD_FRMCTR1  0xB1
#define CMD_DFUNCTR  0xB6
#define CMD_PWCTR1   0xC0
#define CMD_PWCTR2   0xC1
#define CMD_VMCTR1   0xC5
#define CMD_VMCTR2   0xC7
#define CMD_GAMMASET 0x26
#define CMD_GMCTRP1  0xE0
#define CMD_GMCTRN1  0xE1

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} Vector3;
typedef struct{
  float x;
  float y;
} Vector2;
float brightness = 0.5f;
float timeToSleep = 20.0f;
int touchClkSpeed = 2000000;
int screenClkSpeed = 40000000;
Preferences prefs;


//watch data
wl_status_t lastWifiStatus = WiFi.status();
bool gotTime = false;
int32_t unixTimeStart = 0;
int32_t unixTime = 0;
float batVoltage= 0.0f;
//used for polling
unsigned long lastTouch = 0;  
uint8_t appIndex = 0;
bool configedTime = false;
unsigned long timeStart = 0;
unsigned long lastConnect = 0;
unsigned long lastTouchCheck = 0;
const int touchCheckInterval = 10;

//useful funcs
#include <cstdint>

uint16_t reverseBits(uint16_t n) {
    uint16_t result = 0;

    for (int i = 0; i < 16; i++) {
        result <<= 1;        // make space for next bit
        result |= (n & 1);   // copy lowest bit of n
        n >>= 1;             // shift n to process next bit
    }

    return result;
}
uint8_t fortnightDay = 0;
String formattedTime(uint32_t t)
{
  t=t+3600;
  uint32_t days = (uint32_t)(t/86400);
  fortnightDay = ((days+10) % 14);
  uint32_t daySecs = t-(days*86400);
  uint8_t hours = (uint8_t)(daySecs/3600);
  uint32_t hourSecs = daySecs-(hours*3600);
  uint8_t minutes = (uint8_t)(hourSecs/60);
  uint32_t minSecs = hourSecs-(minutes*60);
  uint8_t seconds = (uint8_t)(minSecs);
  char hrs[3];
  sprintf(hrs, "%02d", hours);
  char mins[3];
  sprintf(mins, "%02d", minutes);
  char secs[3];
  sprintf(secs, "%02d", seconds);
  return(String(hrs)+":"+String(mins)+":"+String(secs));
}
void configTime()
{
  configTime(3600, 0, "pool.ntp.org");
  configedTime = true;
}

//audio
uint32_t curToneLength=0;
unsigned long toneStartTime=0;
void playTone(uint32_t freq, uint32_t length)
{
  toneStartTime = millis();
  curToneLength = length;
  ledcSetup(0, freq, 8);
  ledcAttachPin(7, 0);
  ledcWrite(0, 128);
}
//hardware spi interfacing
void writeCommand(uint8_t cmd)
{
  //pull data command line low for commands
  digitalWrite(DC, LOW);
  //cs low whenever talking
  digitalWrite(DISPLAYCS, LOW);
  SPI.transfer(cmd);
  digitalWrite(DISPLAYCS, HIGH);
}
//write 8bit data
void writeData(uint8_t data){
  digitalWrite(DC, HIGH);
  digitalWrite(DISPLAYCS, LOW);
  SPI.transfer(data);
  digitalWrite(DISPLAYCS, HIGH);
}
void writeData16(uint16_t data){
  digitalWrite(DC, HIGH);
  digitalWrite(DISPLAYCS, LOW);
  SPI.transfer16(data);
  digitalWrite(DISPLAYCS, HIGH);
}
//direct screen interfacing
//drawing
void setWindow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1)
{
  writeCommand(CMD_CASET);
  writeData16(x0); writeData16(x1);
  writeCommand(CMD_PASET);
  writeData16(y0); writeData16(y1);
  writeCommand(CMD_RAMWR);
}
void initDisplay()
{
  //config spi control pins
  pinMode(DISPLAYCS, OUTPUT);
  pinMode(DC, OUTPUT);
  //deselect screen before we are ready
  digitalWrite(DISPLAYCS, HIGH);
  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, -1);
  SPI.beginTransaction(SPISettings(screenClkSpeed, MSBFIRST, SPI_MODE0));
  //reset and wake up
  writeCommand(CMD_SWRESET);
  delay(100);
  writeCommand(CMD_SLPOUT);
  delay(100);
  writeCommand(CMD_PWCTR1); writeData(0x23); //gvdd (display ref v)
  writeCommand(CMD_PWCTR2); writeData(0x10);
  writeCommand(CMD_VMCTR1); writeData(0x3E); writeData(0x28); // contrast (high/low)
  writeCommand(CMD_VMCTR2); writeData(0x86); // VCOM offset
  //set how display reads from ram and orientates
  writeCommand(CMD_MADCTL); writeData(0x48);
  //set 16 bit color
  writeCommand(CMD_COLMOD); writeData(0x55);
  //set framerate 
  writeCommand(CMD_FRMCTR1); writeData(0x00); writeData(0x18);
  //config gate driver
  writeCommand(CMD_DFUNCTR); writeData(0x08); writeData(0x82); writeData(0x27);
  //set gamma
  writeCommand(CMD_GAMMASET); writeData(0x01);
  writeCommand(CMD_GMCTRP1); // 15 bytes of positive gamma curve
  writeCommand(CMD_GMCTRN1); // 15 bytes of negative gamma curve
  //switch on
  writeCommand(CMD_DISPON); delay(100);
}
void drawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  setWindow(x, y, x, y);
  digitalWrite(DC, HIGH);
  digitalWrite(DISPLAYCS, LOW);
  SPI.transfer16(reverseBits(color));
  digitalWrite(DISPLAYCS, HIGH);
}
//app level functions
void drawRect(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1, uint16_t color, uint8_t bevelRadius=0)
{
  setWindow(x0, y0, x1, y1);
  digitalWrite(DC, HIGH);
  digitalWrite(DISPLAYCS, LOW);
  uint32_t rectSize = ((x1-x0)+1) * ((y1-y0)+1);
  //draw pixels
  for(uint32_t i=0; i<rectSize; i++)
  {
    SPI.transfer16(reverseBits(color));
  }
  digitalWrite(DISPLAYCS, HIGH);
}
void drawCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t color)
{
  for(int i=0; i<2*radius; i++)
  {
    uint16_t globalY = i+y0-radius;
    uint16_t halfWidth = sqrt((radius*radius)-((0.5+i-radius)*(0.5+i-radius)))+0.5;
    setWindow(x0-halfWidth, globalY, x0+halfWidth, globalY);
    digitalWrite(DC, HIGH);
    digitalWrite(DISPLAYCS, LOW);
    for(uint32_t i=0; i<2*halfWidth; i++)
    {
      SPI.transfer16(reverseBits(color));
    }
    digitalWrite(DISPLAYCS, HIGH);
  }
}
void clearScreen(uint16_t color)
{
  drawRect(0,0,319, 239, color);
}
void drawChar8x12(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg, uint8_t size=1) {
  fg = reverseBits(fg);
  bg = reverseBits(bg);
  if (c < 32 || c > 126) c = '?';
  const uint8_t* glyph = font8x12[c - 32];
  setWindow(x, y, (x-1)+8*size, (y-1) + 12*size);
  digitalWrite(DC, HIGH);
  digitalWrite(DISPLAYCS, LOW);
  for (uint8_t row = 0; row < 12; row++) {
    for(int i=0; i<size; i++){
      uint8_t bits = glyph[row];
      for (uint8_t col = 0; col < 8; col++) {
        for(int i=0; i<size; i++){
          SPI.transfer16((bits & 0x80) ? fg : bg);
        }
        bits <<= 1;
      }
    }
  }
  digitalWrite(DISPLAYCS, HIGH);
}

void drawString8x12(uint16_t x, uint16_t y, String stringParam, uint16_t fg, uint16_t bg, uint8_t size=1) {
  const char* str = stringParam.c_str();
  while (*str) {
    drawChar8x12(x, y, *str++, fg, bg, size);
    x += 9*size; // 8px + 1px gap per size increment
  }
}
//touch control
Vector3 pollTouch()
{
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(touchClkSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(TOUCHCS, LOW);
  SPI.transfer(0x90);
  delayMicroseconds(5);
  uint16_t y = SPI.transfer16(0x0000);
  y=y>>3;
  //Serial.println(String(y));
  SPI.transfer(0xD0);
  delayMicroseconds(5);
  uint16_t x = SPI.transfer16(0x0000);
  x=x>>3;
  digitalWrite(TOUCHCS, HIGH);
  SPI.endTransaction();
  SPI.beginTransaction(SPISettings(screenClkSpeed, MSBFIRST, SPI_MODE0));
  //check if valid touch
  bool validTouch = false;
  if(x>0 && x<4095 && y>0 && y<4095){
    validTouch = true;
    lastTouch = millis();
  }
  //convert touch to screen coordinates
  x = 319-int(x*0.0778998778998779*1.2) +32;
  y = int(y*0.0583638583638584*1.2) -24;
  //Serial.println("x: " + String(x));
  //Serial.println("y: " +String(y));
  //drawPixel(x, y, 0xFFFF);
  Vector3 touch;
  touch.x=x;
  touch.y=y;
  touch.z=3000;
  return touch;
}
//physics engine
typedef struct{
  Vector2 pos;
  Vector2 vel;
  float radius;
  float mass;
  float frictionCoefficient;
}physicsObj;

physicsObj physicsObjs [] = {};
//os built ins
void goSleep()
{
  esp_light_sleep_start();
}
void detectSleep()
{
  if(digitalRead(WAKEUPBTN) == 0)
  {
    lastTouch = millis();
  }
  if(millis()-lastTouch > timeToSleep*1000)
  {
    Serial.println("seepy time");
    goSleep();
  }
}
void readBatteryVoltage()
{
  batVoltage = (analogReadMilliVolts(1)/1000)*2;
}
//button controlling
int buttonCount=0;
typedef struct{
  uint16_t x0; 
  uint16_t y0;
  uint16_t x1;
  uint16_t y1;
  void(*callback)(uint8_t);
  uint8_t code;
} Button;
Button buttons[100];
void assignButton(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, void(*callback)(uint8_t), uint8_t code)
{
  buttons[buttonCount].x0 = x0;
  buttons[buttonCount].y0 = y0;
  buttons[buttonCount].x1 = x1;
  buttons[buttonCount].y1 = y1;
  buttons[buttonCount].callback = callback;
  buttons[buttonCount].code = code;
  buttonCount++;
}
void checkButtons()
{
  Vector3 touch = pollTouch();
  //check if touch is in any button and then call their callback function and send their code.
  for(int i=0; i<buttonCount; i++)
  {
    if(buttons[i].x0<touch.x && touch.x<buttons[i].x1 && buttons[i].y0<touch.y && touch.y<buttons[i].y1 && touch.z>1000)
    {
      //button pressed
      buttons[i].callback(buttons[i].code);
    }
  }
}
void clearButtons()
{
  buttonCount=0;
}
//app handling
typedef struct {
  const char* name;
  uint16_t themeColor;
  void(*Init)();
  void(*Loop)();
} App;
void homeAppInit();
void homeAppLoop();
void drawingAppInit();
void drawingAppLoop();
void WeatherAppInit();
void WeatherAppLoop();
void snookerAppInit();
void snookerAppLoop();
uint16_t backgroundColor = 0x0000;
App apps[] = {
  {"homeApp", backgroundColor, homeAppInit, homeAppLoop},
  {"DRAW", 0xFFFF, drawingAppInit, drawingAppLoop},
  {"WEATHER", 0x187F, WeatherAppInit, WeatherAppLoop},
  {"SNOOKER", 0x0700, snookerAppInit, snookerAppLoop}
};

//home app
String lessons[] = {
    // Week 1
    "PSHE","En","Ph","RE","Bio",
    "PSHE","DT","RE","En","Bu",
    "PSHE","CS","Ph","Ma","PSHE",
    "PSHE","Ga","CS","Bu","Ma",
    "PSHE","Ma","DT","En","Ch",

    // Week 2
    "PSHE","Ma","DT","RE","CS",
    "PSHE","Bu","Bio","En","Ph",
    "PSHE","Ch","Bu","En","Ph",
    "PSHE","Bio","Ga","Ma","DT",
    "PSHE","Ma","En","CS","Ch"
};
void homeAppOpenApp(uint8_t appI)
{
  appIndex = appI;
}
void homeAppInit()
{
  clearScreen(backgroundColor);
  //drawString8x12(118, 72, "20:39", 0xFFFF, backgroundColor,2);
  for(int i=1; i<sizeof(apps)/sizeof(apps[0]); i++){
    drawRect((i-1)*106 + 14, 140, (i-1)*106 + 94, 220, apps[i].themeColor);
    assignButton((i-1)*106 + 14, 140, (i-1)*106 + 94, 220,homeAppOpenApp,i);   
    //drawString8x12((i-1)*106 + 54 - (uint8_t)((sizeof(apps[i].name)*9 - 1) / 2), 170, apps[i].name, 0xFFFF, apps[i].themeColor);
  }
}
void homeAppLoop()
{
  char buffer[30];
  sprintf(buffer, "%ld", (uint32_t)(time(nullptr)));
  String timeStr = "";
  if(gotTime){
    timeStr = formattedTime((uint32_t)(time(nullptr)));
  }
  else{
    prefs.begin("mainPrefs", false);
    timeStr = formattedTime(prefs.getInt("time", 0));
    prefs.end();
  }
  const char* formattedTimeStr = timeStr.c_str();
  drawString8x12(89, 60, formattedTimeStr, 0x01F8, backgroundColor,2);
  uint8_t weekday = 0;
  if(fortnightDay > 4 && fortnightDay < 7 or fortnightDay > 11)
  {
    drawString8x12(0,0, "no lessons!", 0x01F8, apps[appIndex].themeColor);
  }
  else{
    weekday = fortnightDay;
    if(weekday > 4){
      weekday = weekday-2;
    }
    String todaysLessons = "";
    for(int i=0; i<5; i++)
    {
      todaysLessons += lessons[(weekday*5)+i];
      if(i<4){
        todaysLessons += ",";
      }
    }
    drawString8x12(0,0, todaysLessons, 0x01F8, apps[appIndex].themeColor);
  }
  const char* wordedWifiStatus = (WiFi.status()==WL_CONNECTED)?"CONNECTED":"DISCONNECTED";
  drawString8x12(320-(strlen(wordedWifiStatus)*9), 2, wordedWifiStatus, 0x01F8, apps[appIndex].themeColor);
}

//drawing app
void drawingAppInit()
{
  clearScreen(apps[appIndex].themeColor);
}
void drawingAppLoop()
{
  Vector3 touch = pollTouch();
  if(touch.z>1000){
    drawPixel(touch.x, touch.y, 0xF800);
  }
}
//snooker app
void snookerAppInit()
{
  clearScreen(apps[appIndex].themeColor);
  drawCircle(100,100, 7, 0xFFFF);
}
void snookerAppLoop()
{
  delay(100);
}
//detailed weather data
uint32_t weatherDataRetrievalInterval = 100000;
unsigned long lastWeatherDataRetrieval = 0;
float temp = 0.0;
long sunset = 0;
void getWeatherData()
{
  if(WiFi.status() == WL_CONNECTED){
    HTTPClient http;
    http.begin("http://api.openweathermap.org/data/2.5/weather?q=Brackley&appid=deb19891f2e13b16efdadb1e42493c60&units=metric");
    int code = http.GET();
    Serial.println(String(code));
    if (code == 200) {
      String payload = http.getString();
      JsonDocument doc;
      deserializeJson(doc, payload);
      temp = doc["main"]["temp"];
      sunset = doc["sys"]["sunset"];
      Serial.println(temp);
      Serial.println(sunset);
    }
    http.end();
    lastWeatherDataRetrieval=millis();
  }
}
void displayWeatherData()
{
  char buf[6];
  sprintf(buf,"%.1f", temp);
  drawString8x12(0,0,"Temp: " + String(buf) + "C",0xFFFF, apps[appIndex].themeColor, 2);
  String sunsetTime = formattedTime((uint32_t)(sunset));
  drawString8x12(0,28,"Sunset: " + sunsetTime,0xFFFF, apps[appIndex].themeColor, 2);
}
void WeatherAppInit()
{
  clearScreen(apps[appIndex].themeColor);
  getWeatherData();
  displayWeatherData();
}
void WeatherAppLoop()
{
  if(millis()-lastWeatherDataRetrieval > weatherDataRetrievalInterval)
  {
    getWeatherData();
    displayWeatherData();
  }
}
void setup()
{
  //disable brownouts
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  Serial.println("Awake");
  //setup screen
  //setup backlight
  pinMode(BL, OUTPUT);
  //analogWriteFrequency(5000);
  //analogWrite(BL, int(brightness * 255));
  digitalWrite(BL, HIGH);
  initDisplay();

  //setup batVoltatage detector
  pinMode(BATVPIN, INPUT);
  analogSetAttenuation(ADC_11db);

  //setup sleeper button
  pinMode(WAKEUPBTN, INPUT_PULLUP);
  gpio_wakeup_enable(WAKEUPBTN, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  lastTouch = millis();
  //goSleep();
  //connect to wifi
  //Serial.print("Connected");
  WiFi.mode(WIFI_STA);
  pinMode(TOUCHCS, OUTPUT);
  //launch home app at start
}
float lastbatVoltage = 0.0f;
uint8_t lastAppIndex = 0xFF;
uint8_t WiFiConfigSize = sizeof(WiFiConfigs)/sizeof(WiFiConfigs[0]);
uint8_t WiFiConfigIndex=0;
void loop()
{
  //app logic
  if(appIndex != lastAppIndex)
  {
    //open app
    //clean last app
    clearButtons();
    //initialize
    apps[appIndex].Init();
    //confirm init to avoid multi-init
    lastAppIndex = appIndex;
  }
  else
  {
    //run app loop
    apps[appIndex].Loop();
  }
  //background processes
  //sleep detection
  detectSleep();
  //get time
  //getTime();
  //check if pressing home button
  if(digitalRead(WAKEUPBTN) == LOW)
  {
    //go home
    appIndex = 0;
    //Serial.println("Returning to home");
  }
  //look if to poll for touch
  if(millis()-lastTouchCheck > touchCheckInterval)
  {
    lastTouchCheck = millis();
    //Serial.println("polling touch");
    checkButtons();
  }
  //Serial.println(WiFi.status());
  //deal with audio
  if(millis()-toneStartTime > curToneLength)
  {
    ledcWrite(0, 0);
  }
  //config time
  if(!configedTime){
    if(WiFi.status() == WL_CONNECTED){
      configTime();
    }
  }
  if(WiFi.status() == WL_CONNECTED && time(nullptr)-unixTimeStart > 1000)
  {
    configTime();
  }
  if(WiFi.status() != WL_CONNECTED && millis() - lastConnect > 1000)
  {
    Serial.println("Connecting...: " + String(WiFi.status()));
    lastConnect = millis();
    //try connect
    if(WiFiConfigSize>0){
      WiFi.begin(WiFiConfigs[WiFiConfigIndex].ssid, WiFiConfigs[WiFiConfigIndex].pass);
    }
    WiFiConfigIndex++;
    if(WiFiConfigIndex >= WiFiConfigSize)
    {
      WiFiConfigIndex=0;
    }
  }
  if(!gotTime){
    uint32_t knownTime = (uint32_t)(time(nullptr));
    if(knownTime > 1700000000)
    {
      unixTimeStart = knownTime;
      gotTime = true;
      timeStart = millis();
      prefs.begin("mainPrefs", false);
      prefs.putInt("time", unixTimeStart);
      prefs.end();
    }
  }
  if(gotTime)
  {
    float timeSinceGotUnixTime = (millis()-timeStart)/1000;
    unixTime = unixTimeStart + int(timeSinceGotUnixTime);
  }
}