#include <Arduino.h>
#include "FIT_PowerManagement.h"
#include "debugmessage.h"

#if defined(SOFT_E_STOP)
unsigned long E_STOP_BTN_INT_Press_TickCount;
unsigned long E_STOP_BTN_INT_Release_TickCount;
#endif

FITPM::~FITPM(){/*nothing to do*/}

#if defined(SOFT_E_STOP)
void E_STOP_CB()
{
  if (digitalRead(E_STOP_BUTTON) == LOW)
  {
    dbg_printf("E_STOP_CB press\r\n");
    E_STOP_BTN_INT_Release_TickCount=0;
    E_STOP_BTN_INT_Press_TickCount=millis();

  }
  else
  {
    dbg_printf("E_STOP_CB release\r\n");
    E_STOP_BTN_INT_Press_TickCount=0;
    E_STOP_BTN_INT_Release_TickCount=millis();
  }
}
#endif

void FITPM::begin()
{
  FITDBGbegin();
  initFITLM5066I();
  initFITBTT6030();
  batteryLowNotifyInterval=0;
  batteryLowNotifyCount=0;
  battery_callback_func_ptr=NULL;

  #if defined(SOFT_E_STOP)
  E_STOP_BTN_INT_Press_TickCount=0;
  E_STOP_BTN_INT_Release_TickCount=0;
  bE_STOP_BTN_Press=false;
  bE_STOP_ACTION=false;
  Estop_callback_func_ptr=NULL;
  pinMode(E_STOP_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(E_STOP_BUTTON),E_STOP_CB, CHANGE);
  E_STOP_CB();
  #endif
}

#if defined(SOFT_E_STOP)
void FITPM::process_E_STOP_BTN()
{
  if(!bE_STOP_BTN_Press)
  {
    if(E_STOP_BTN_INT_Press_TickCount!=0)
    {
      if(millis()-E_STOP_BTN_INT_Press_TickCount>DEBOUNCE_TIME)
      {
        E_STOP_BTN_INT_Press_TickCount=0;
        bE_STOP_BTN_Press=true;
        //dbg_printf("process_E_STOP_BTN press\r\n");
      }
    }
  }
  else
  {
    if(E_STOP_BTN_INT_Release_TickCount!=0)
    {
      if(millis()-E_STOP_BTN_INT_Release_TickCount>DEBOUNCE_TIME)
      {
        E_STOP_BTN_INT_Release_TickCount=0;
        bE_STOP_BTN_Press=false;
        //dbg_printf("process_E_STOP_BTN release\r\n");
      }
    }
  }

  if(bE_STOP_BTN_Press)
  {
    if(!bE_STOP_ACTION)
    {
      bE_STOP_ACTION=true;
      if(Estop_callback_func_ptr!=NULL)
      {
        Estop_callback_func_ptr(E_STOP_ENABLE);
      }
      dbg_printf("E_STOP_BTN E_STOP_ENABLE\r\n");
    }
  }
  else
  {
    if(bE_STOP_ACTION)
    {
      bE_STOP_ACTION=false;
      if(Estop_callback_func_ptr!=NULL)
      {
        Estop_callback_func_ptr(E_STOP_DISABLE);
      }
      dbg_printf("E_STOP_BTN E_STOP_DISABLE\r\n");
    }
  }
}

void FITPM::registEStopCallBack(void (*callback_func_ptr)(uint8_t))
{
  Estop_callback_func_ptr=callback_func_ptr;
}
#endif

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
  #if defined(SOFT_E_STOP)
  process_E_STOP_BTN();
  #endif
  processLM5066I();
  processBTT6030();
  updateBatteryPercentage();
  process_led();
}
