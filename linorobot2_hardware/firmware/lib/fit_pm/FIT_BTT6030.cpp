#include <Arduino.h>
#include "FIT_BTT6030.h"
#include "debugmessage.h"

FITBTT6030::FITBTT6030(int16_t lAdcPin,int16_t rAdcPin,int16_t lSwitchPin,int16_t rSwitchPin)
{
  isLeftPin = lAdcPin;
  isRightPin = rAdcPin;
  LeftSwitchPin = lSwitchPin;
  RightSwitchPin = rSwitchPin;
  updateISValueInterval=ADC_UPDATE_INTERVAL;
  overCurrent = OVER_CURRENT;
  OverCurrentRecoverInterval=OVER_CURRENT_RECOVER_INTERVAL;
  lowloadCurrent=LOWLOAD_CURRENT;
  LowloadCurrentInterval=LOWLOAD_CURRENT_INTERVAL;
  OverCurrentCheckTimes=OVER_CURRENT_CHECK_TIMES;
}

FITBTT6030::~FITBTT6030(){/*nothing to do*/}

void FITBTT6030::initFITBTT6030()
{
  pinMode(LeftSwitchPin , OUTPUT);
  pinMode(RightSwitchPin , OUTPUT);
  digitalWrite(LeftSwitchPin, LOW);
  digitalWrite(RightSwitchPin, LOW);
  analogReadRes(12);
  analogReadAveraging(10);
  updateISValueTickCount=0;
  lOverCurrentCount=0;
  rOverCurrentCount=0;
  lOverCurrentRecoverCount=0;
  rOverCurrentRecoverCount=0;
  bLeftOverCurrent=false;
  bRightOverCurrent=false;
  lLowLoadCurrentRecoverCount=0;
  rLowLoadCurrentRecoverCount=0;
  processISAVGvalue();
}

void FITBTT6030::processISAVGvalue()
{
  leftISAVGvalue=analogRead(isLeftPin);
  rightISAVGvalue=analogRead(isRightPin);

#if 0
  if(isLeftSwitchOn()&&leftISAVGvalue<=lowloadCurrent)
  {
    if(lLowLoadCurrentRecoverCount==0)
    {
      lLowLoadCurrentRecoverCount = millis();
    }
    else
    {
      if(millis()-lLowLoadCurrentRecoverCount>=LowloadCurrentInterval)
      {
      	enableLeftSwitch(false);
      	lLowLoadCurrentRecoverCount=0;
      }
    }
  }
  else
  {
    if(lLowLoadCurrentRecoverCount)
    	lLowLoadCurrentRecoverCount=0;

    if(rLowLoadCurrentRecoverCount)
      rLowLoadCurrentRecoverCount=0;
  }

  if(isRightSwitchOn()&&rightISAVGvalue<=lowloadCurrent)
  {
    if(rLowLoadCurrentRecoverCount==0)
    {
      rLowLoadCurrentRecoverCount = millis();
    }
    else
    {
      if(millis()-rLowLoadCurrentRecoverCount>=LowloadCurrentInterval)
      {
        enableRightSwitch(false);
      	rLowLoadCurrentRecoverCount=0;
      }
    }
  }
  else
  {
    if(lLowLoadCurrentRecoverCount)
      lLowLoadCurrentRecoverCount=0;

    if(rLowLoadCurrentRecoverCount)
      rLowLoadCurrentRecoverCount=0;
  }
#endif

  if(leftISAVGvalue>=overCurrent)
  {
    if(!bLeftOverCurrent)
    {
      lOverCurrentCount++;
      if(lOverCurrentCount>=OverCurrentCheckTimes)
      {
        bLeftOverCurrent=true;
        lOverCurrentCount=0;
        lOverCurrentRecoverCount=0;
      }
    }
  }
  else
  {
    if(lOverCurrentCount)
      lOverCurrentCount=0;

    if(bLeftOverCurrent)
    {
      if(lOverCurrentRecoverCount==0)
      {
      	lOverCurrentRecoverCount = millis();
      }
      else
      {
      	if(millis()-lOverCurrentRecoverCount>=OverCurrentRecoverInterval)
      	{
      	  bLeftOverCurrent=false;
      	  lOverCurrentRecoverCount=0;
      	}
      }
    }
  }

  if(rightISAVGvalue>=overCurrent)
  {
    if(!bRightOverCurrent)
    {
      rOverCurrentCount++;
      if(rOverCurrentCount>=OverCurrentCheckTimes)
      {
        bRightOverCurrent=true;
        rOverCurrentCount=0;
        rOverCurrentRecoverCount=0;
      }
    }
  }
  else
  {
    if(rOverCurrentCount)
      rOverCurrentCount=0;

    if(bRightOverCurrent)
    {
      if(rOverCurrentRecoverCount==0)
      {
      	rOverCurrentRecoverCount = millis();
      }
      else
      {
      	if(millis()-rOverCurrentRecoverCount>=OverCurrentRecoverInterval)
      	{
      	  bRightOverCurrent=false;
      	  rOverCurrentRecoverCount=0;
      	}
      }
    }
  }

  if(isLeftSwitchOn())
  {
    dbg_printf("leftISAVGvalue:%d\r\n",leftISAVGvalue);
    dbg_printf("rightISAVGvalue:%d\r\n",rightISAVGvalue);
  }
}

int16_t  FITBTT6030::getLeftISAVGvalue()
{
  return leftISAVGvalue;
}

int16_t  FITBTT6030::getRightISAVGvalue()
{
  return rightISAVGvalue;
}

void FITBTT6030::enableLeftSwitch(bool enable)
{
  lLowLoadCurrentRecoverCount=0;
  bLeftSwitchOn=enable;
}

void FITBTT6030::enableRightSwitch(bool enable)
{
  rLowLoadCurrentRecoverCount=0;
  bRightSwitchOn=enable;
}

bool FITBTT6030::isLeftSwitchOn()
{
  return bLeftSwitchOn;
}

bool FITBTT6030::isRightSwitchOn()
{
  return bRightSwitchOn;
}

bool FITBTT6030::isLeftOverCurrent()
{
  return bLeftOverCurrent;
}

bool FITBTT6030::isRightOverCurrent()
{
  return bRightOverCurrent;
}

void FITBTT6030::processBTT6030()
{
  if(updateISValueTickCount==0)
  {
    updateISValueTickCount = millis();
  }
  else
  {
    if(millis()-updateISValueTickCount>=updateISValueInterval)
    {
      updateISValueTickCount=0;
      processISAVGvalue();
    }
  }

  if(isLeftSwitchOn()&&!isLeftOverCurrent()&&!isRightOverCurrent()&&digitalRead(LeftSwitchPin)==LOW)
  {
    dbg_printf("turn on left\r\n");
    digitalWrite(LeftSwitchPin, HIGH);
  }
  else if((!isLeftSwitchOn()||isLeftOverCurrent()||isRightOverCurrent())&&digitalRead(LeftSwitchPin)==HIGH)
  {
    dbg_printf("turn off left\r\n");
    digitalWrite(LeftSwitchPin, LOW);
  }

  if(isRightSwitchOn()&&!isRightOverCurrent()&&!isLeftOverCurrent()&&digitalRead(RightSwitchPin)==LOW)
  {
    dbg_printf("turn on right\r\n");
    digitalWrite(RightSwitchPin, HIGH);
  }
  else if((!isRightSwitchOn()||isRightOverCurrent()||isLeftOverCurrent())&&digitalRead(RightSwitchPin)==HIGH)
  {
    dbg_printf("turn off right\r\n");
    digitalWrite(RightSwitchPin, LOW);
  }
}