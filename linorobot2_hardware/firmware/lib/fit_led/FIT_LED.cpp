#include <Arduino.h>
#include "FIT_LED.h"

Adafruit_NeoPixel strip_L(LED_L_COUNT, LED_L_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip_R(LED_R_COUNT, LED_R_PIN, NEO_GRB + NEO_KHZ800);


FITLED::FITLED(){

  ledColor_L[LED_STATES_OFF]=strip_L.Color(0,   0,   0);
  ledColor_R[LED_STATES_OFF]=strip_R.Color(0,   0,   0);

  ledColor_L[LED_STATES_DRIVING]=strip_L.Color(0,   0,   255);
  ledColor_R[LED_STATES_DRIVING]=strip_R.Color(0,   0,   255);

  ledColor_L[LED_STATES_REVERSE]=strip_L.Color(0,   0,   255);
  ledColor_R[LED_STATES_REVERSE]=strip_R.Color(0,   0,   255);

  ledColor_L[LED_STATES_TURN_L]=strip_L.Color(0,   0,   255);
  ledColor_R[LED_STATES_TURN_L]=strip_R.Color(0,   0,   255);

  ledColor_L[LED_STATES_TURN_R]=strip_L.Color(0,   0,   255);
  ledColor_R[LED_STATES_TURN_R]=strip_R.Color(0,   0,   255);

  ledColor_L[LED_STATES_CAUTION_ZONE]=strip_L.Color(255,   255,   0);
  ledColor_R[LED_STATES_CAUTION_ZONE]=strip_R.Color(255,   255,   0);

  ledColor_L[LED_STATES_STANDBY]=strip_L.Color(255,   255,   0);
  ledColor_R[LED_STATES_STANDBY]=strip_R.Color(255,   255,   0);

  ledColor_L[LED_STATES_FAULT]=strip_L.Color(255,   0,   0);
  ledColor_R[LED_STATES_FAULT]=strip_R.Color(255,   0,   0);

  ledColor_L[LED_STATES_CHARGING]=strip_L.Color(255,   0,   0);
  ledColor_R[LED_STATES_CHARGING]=strip_R.Color(255,   0,   0);

  ledColor_L[LED_STATES_E_STOP]=strip_L.Color(255,   0,   0);
  ledColor_R[LED_STATES_E_STOP]=strip_R.Color(255,   0,   0);

  blinkInterval =666;//ms
  blinkTickCount =0;

  breatheInterval =15;//ms
  breatheTickCount =0;//ms

  brightnessL=50;
  brightnessR=50;

  brightnessCount=0;
  brightnessCoeff=1;

  led_status=0x00;
  prv_led_status=0x00;
  process_led_status=0x00;

  breatheMaxLevel=100;
  breatheMinLevel=-10;

  batteryLowInterval=BATTERY_LOW_INTERVAL;
  batteryLowTickCount=0;

  bLedOnL=false;
  bLedOnR=false;

  bBlink=false;
  bBreathe=false;
}

FITLED::~FITLED(){/*nothing to do*/}

void FITLED::led_setup(uint16_t numberofLED_L,uint16_t numberofLED_R , int16_t pinL, int16_t pinR, neoPixelType t)
{
  strip_L.updateType(t);
  strip_R.updateType(t);

  strip_L.updateLength(numberofLED_L);
  strip_R.updateLength(numberofLED_R);

  strip_L.setPin(pinL);
  strip_R.setPin(pinR);	

  strip_L.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip_L.show();            // Turn OFF all pixels ASAP

  strip_R.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip_R.show();            // Turn OFF all pixels ASAP
}

bool FITLED::isbLedOnL(){
  return bLedOnL;
}

bool FITLED::isbLedOnR(){
  return bLedOnR;
}

void  FITLED::turn_r_led_onoff(bool bEnable,uint32_t color,int brightness)
{
  if(brightness<0)
    return;

  strip_R.clear();
  if(bEnable)
  {
    strip_R.setBrightness(brightness);
    int blankcount=0;
    for(int i=0; i<strip_R.numPixels(); i++) { // For each pixel in strip...
      if(blankcount<LED_BLANK&&i!=0)
      {
        blankcount++;
        strip_R.setPixelColor(i,ledColor_R[LED_STATES_OFF]);         //  Set pixel's color (in RAM
      }
      else
      {
        strip_R.setPixelColor(i, color);         //  Set pixel's color (in RAM
        blankcount=0;
      }
    }
    bLedOnR=true;
  }
  else
  {
    bLedOnR=false;
    strip_R.setBrightness(0);
  }
  strip_R.show();
}

void  FITLED::turn_l_led_onoff(bool bEnable,uint32_t color,int brightness)
{
  if(brightness<0)
    return;

  strip_L.clear();
  if(bEnable)
  {
    strip_L.setBrightness(brightness);
    int blankcount=0;
    for(int i=0; i<strip_L.numPixels(); i++) { // For each pixel in strip...
      if(blankcount<LED_BLANK&&i!=0)
      {
        blankcount++;
        strip_L.setPixelColor(i,ledColor_L[LED_STATES_OFF]);         //  Set pixel's color (in RAM
      }
      else
      {
        strip_L.setPixelColor(i, color);         //  Set pixel's color (in RAM
        blankcount=0;
      }
    }
    bLedOnL=true;
  }
  else
  {
    strip_L.setBrightness(0);
    bLedOnL=false;
  }
  strip_L.show();
}

void  FITLED::setLedStatus(uint8_t status)
{

  if(led_status!=status||status==LED_STATES_OFF)
  {
    if(led_status==LED_STATES_E_STOP&&status!=LED_STATES_OFF&&status!=LED_STATES_BATTERY_LOW)
    {
      return;
    }
    if(led_status==LED_STATES_BATTERY_LOW)
    {
      if(status!=LED_STATES_BATTERY_LOW&&prv_led_status!=LED_STATES_E_STOP)
        prv_led_status=status;
      return;
    }
    prv_led_status=led_status;
    led_status=status;
    blinkTickCount =0;//ms
    breatheTickCount=0;
    brightnessCount=0;
    batteryLowTickCount=0;
    bBlink=false;
    bBreathe=false;
    turn_r_led_onoff(false,ledColor_R[LED_STATES_OFF],0);
    turn_l_led_onoff(false,ledColor_L[LED_STATES_OFF],0);   
  }
}

void  FITLED::blink_LED(uint8_t side,uint32_t color)
{
  bBlink=true;
  if(blinkTickCount==0)
  {
    blinkTickCount = millis();
  }
  else
  {
    unsigned long instantBlinkTickCount =millis();
    if(instantBlinkTickCount>blinkTickCount)
    {
      if(instantBlinkTickCount-blinkTickCount>blinkInterval)
      {
        if(side==LED_L)
        {
          if(isbLedOnL())
          {
            turn_l_led_onoff(false,color,0);
          }
          else
          {
            turn_l_led_onoff(true,color,brightnessL);
          }
        }
        else if(side==LED_R)
        {
            if(isbLedOnR())
            {
              turn_r_led_onoff(false,color,0);
            }
            else
            {
              turn_r_led_onoff(true,color,brightnessR);  
            }
        }
        else if(side==LED_LR)
        {
          if(isbLedOnL())
          {
            turn_l_led_onoff(false,color,0);
          }
          else
          {
            turn_l_led_onoff(true,color,brightnessL);
          }

          if(isbLedOnR())
          {
            turn_r_led_onoff(false,color,0);
          }
          else
          {
            turn_r_led_onoff(true,color,brightnessR);
          }
        }

        blinkTickCount=0;
      }
    }
    else
    {
      blinkTickCount=0;
    }
  }
}

void FITLED::setblinkInterval(uint32_t Interval)
{
  blinkInterval=Interval;
}

void  FITLED::breatheLed(uint8_t side,uint32_t color)
{
  bBreathe=true;
  if(breatheTickCount==0)
  {
    breatheTickCount = millis();
  }
  else
  {
    unsigned long instantBreatheTickCount =millis();
    if(instantBreatheTickCount>breatheTickCount)
    {
      if(instantBreatheTickCount-breatheTickCount>breatheInterval)
      {
        if(side==LED_L)
        {
          turn_l_led_onoff(true,color,brightnessCount);
        }
        else if(side==LED_R)
        {
          turn_r_led_onoff(true,color,brightnessCount); 
        }
        else if(side==LED_LR)
        {
          turn_l_led_onoff(true,color,brightnessCount);
          turn_r_led_onoff(true,color,brightnessCount);
        }

        breatheTickCount=0;

        if(brightnessCount>=breatheMaxLevel)
        {
          brightnessCoeff=-1;
        }
        else if(brightnessCount<=breatheMinLevel)
        {
          brightnessCoeff=1;
        }
        brightnessCount+=brightnessCoeff;
      }
    }
    else
    {
      breatheTickCount=0;
    }
  }
}

void FITLED::set_breatheLevel(int maxlevel,int minlevel)
{
  breatheMaxLevel=maxlevel;
  breatheMinLevel=minlevel;
}

void FITLED::setbreatheInterval(uint32_t Interval)
{
  breatheInterval=Interval;
}

void FITLED::change_led_color(uint8_t status,uint32_t color,uint8_t side)
{
  if(side==LED_L)
  {
    ledColor_L[status]=color;
  }
  else if(side==LED_R)
  {
    ledColor_R[status]=color;
  }
  else if(side==LED_LR)
  {
    ledColor_R[status]=color;
    ledColor_L[status]=color;
  }
}

void FITLED::setbrightness(uint8_t brightness,uint8_t side)
{
  if(side==LED_L)
  {
    brightnessL=brightness;
  }
  else if(side==LED_R)
  {
    brightnessR=brightness;
  }
  else if(side==LED_LR)
  {
    brightnessL=brightness;
    brightnessR=brightness;
  }
}

void  FITLED::process_led()
{
  if(process_led_status!=led_status||bBlink||bBreathe)
  {
    if(led_status==LED_STATES_OFF)
    {
      turn_r_led_onoff(false,ledColor_R[LED_STATES_OFF],0);
      turn_l_led_onoff(false,ledColor_L[LED_STATES_OFF],0);
    }
    else if(led_status==LED_STATES_DRIVING)
    {
      turn_r_led_onoff(true,ledColor_R[LED_STATES_DRIVING],brightnessR);
      turn_l_led_onoff(true,ledColor_L[LED_STATES_DRIVING],brightnessL);
    }
    else if(led_status==LED_STATES_REVERSE)
    {
      blink_LED(LED_LR,ledColor_L[LED_STATES_REVERSE]);
    }
    else if(led_status==LED_STATES_TURN_L)
    {
      blink_LED(LED_L,ledColor_L[LED_STATES_TURN_L]);
    }
    else if(led_status==LED_STATES_TURN_R)
    {
      blink_LED(LED_R,ledColor_R[LED_STATES_TURN_R]);
    }
    else if(led_status==LED_STATES_CAUTION_ZONE)
    {
      turn_r_led_onoff(true,ledColor_R[LED_STATES_CAUTION_ZONE],brightnessR);
      turn_l_led_onoff(true,ledColor_L[LED_STATES_CAUTION_ZONE],brightnessL);
    }
    else if(led_status==LED_STATES_STANDBY)
    {
      blink_LED(LED_LR,ledColor_L[LED_STATES_STANDBY]);
    }
    else if(led_status==LED_STATES_FAULT)
    {
      turn_r_led_onoff(true,ledColor_R[LED_STATES_FAULT],brightnessR);
      turn_l_led_onoff(true,ledColor_L[LED_STATES_FAULT],brightnessL);
    }
    else if(led_status==LED_STATES_CHARGING)
    {
      breatheLed(LED_LR,ledColor_L[LED_STATES_CHARGING]);
    }
    else if(led_status==LED_STATES_BATTERY_LOW)
    {
      if(blinkInterval!=80)
        setblinkInterval(80);

      blink_LED(LED_LR,ledColor_L[LED_STATES_CHARGING]);

      if(batteryLowTickCount==0)
      {
        batteryLowTickCount = millis();
      }
      else
      {
        if(millis()-batteryLowTickCount>batteryLowInterval)
        {
          setblinkInterval(666);
          led_status=LED_STATES_OFF;
          setLedStatus(prv_led_status);
          batteryLowTickCount=0;
        }
      }
    }
    else if(led_status==LED_STATES_E_STOP)
    {
      blink_LED(LED_LR,ledColor_L[LED_STATES_E_STOP]);
    }
    process_led_status=led_status;
  }
}

