#ifndef FITLED_H
#define FITLED_H

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_PIN_L    9
#define LED_PIN_R    10

#define  NUMBER_OF_LED_L 100
#define  NUMBER_OF_LED_R 100

#define LED_L_PIN    6
#define LED_R_PIN    5

#define  LED_L_COUNT 21
#define  LED_R_COUNT 21

#define LED_BLANK 0

#define LED_STATES_OFF           0x00
#define LED_STATES_DRIVING       0x01
#define LED_STATES_REVERSE       0x02
#define LED_STATES_TURN_L        0x03
#define LED_STATES_TURN_R        0x04
#define LED_STATES_CAUTION_ZONE  0x05
#define LED_STATES_STANDBY       0x06
#define LED_STATES_FAULT         0x07
#define LED_STATES_CHARGING      0x08
#define LED_STATES_BATTERY_LOW   0x09
#define LED_STATES_E_STOP        0x0A

#define LED_L  0x01
#define LED_R  0x02
#define LED_LR 0x03

#define BATTERY_LOW_INTERVAL 5000


class FITLED{

public:

FITLED ();

~FITLED ();

bool isbLedOnL();

bool isbLedOnR();

void turn_r_led_onoff(bool bEnable,uint32_t color,int brightness);

void turn_l_led_onoff(bool bEnable,uint32_t color,int brightness);

void setLedStatus(uint8_t status);

void blink_LED(uint8_t side,uint32_t color);

void breatheLed(uint8_t side,uint32_t color);

void change_led_color(uint8_t status,uint32_t color,uint8_t side);

void setbrightness(uint8_t brightness,uint8_t side);

void setblinkInterval(uint32_t Interval);

void setbreatheInterval(uint32_t Interval);

void led_setup(uint16_t numberofLED_L,uint16_t numberofLED_R , int16_t pinL, int16_t pinR, neoPixelType t);

void set_breatheLevel(int maxlevel,int minlevel);

void process_led();

protected:
bool bLedOnL=false;
bool bLedOnR=false;
bool bBlink=false;
bool bBreathe=false;

int breatheMaxLevel;
int breatheMinLevel;
uint8_t led_status;
uint8_t prv_led_status;
uint8_t process_led_status;
unsigned long blinkInterval;//ms
unsigned long blinkTickCount;//ms
unsigned long breatheInterval;//ms
unsigned long breatheTickCount;//ms
unsigned long batteryLowInterval;//ms
unsigned long batteryLowTickCount;//ms
uint8_t brightnessL;
uint8_t brightnessR;
int brightnessCount;
int brightnessCoeff;
uint32_t ledColor_L[11];
uint32_t ledColor_R[11];

};
#endif