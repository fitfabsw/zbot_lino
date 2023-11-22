#include <Arduino.h>
#include "FIT_PowerManagement.h"
#include "debugmessage.h"

FITPM::~FITPM(){/*nothing to do*/}

void FITPM::begin()
{
  FITDBGbegin();
  initFITLM5066I();
  initFITBTT6030();
  batteryLowNotifyInterval=0;
  batteryLowNotifyCount=0;
  battery_callback_func_ptr=NULL;
}

void FITPM::enableHSwitch(double lpwm,double rpwm,bool bSwitchImmediately)
{
  if(lpwm!=0)
  {
    enableLeftSwitch(true);
  }
  else
  {
    enableLeftSwitch(false);
  }

  if(rpwm!=0)
  {
    enableRightSwitch(true);
  }
  else
  {
    enableRightSwitch(false);
  }

  if(bSwitchImmediately)
  {
    processBTT6030();
  }
}

void FITPM::updateBatteryPercentage()
{
  if(percentage!=Prvpercentage)
  {
    if(battery_callback_func_ptr!=NULL)
    {
      battery_callback_func_ptr(percentage);
    }

    Prvpercentage=percentage;
  }

  if(percentage<=BATTERY_LOW_PERCENTAGE)
  {
    if(batteryLowNotifyCount==0)
    {
    	batteryLowNotifyCount = millis();
    }
    else
    {
      if(millis()-batteryLowNotifyCount>batteryLowNotifyInterval)
      {
        setLedStatus(LED_STATES_BATTERY_LOW);
        batteryLowNotifyInterval=60000;
        batteryLowNotifyCount=0;
      }
    }
  }
  else
  {
    if(batteryLowNotifyInterval)
      batteryLowNotifyInterval=0;
    if(batteryLowNotifyCount)
      batteryLowNotifyCount=0;
  }
}

void FITPM::registBatteryPercentageCallBack(void (*callback_func_ptr)(int))
{
  battery_callback_func_ptr=callback_func_ptr;
}

void FITPM::processPowerManagement()
{
  processLM5066I();
  processBTT6030();
  updateBatteryPercentage();
  process_led();
}
