#include <Arduino.h>
#include "FIT_PowerManagement.h"
#include "debugmessage.h"

#if defined(SOFT_E_STOP)
unsigned long E_STOP_BTN_INT_Press_TickCount;
unsigned long E_STOP_BTN_INT_Release_TickCount;
#endif

unsigned long LIDAR_INT1_HIGH_TickCount;
unsigned long LIDAR_INT1_LOW_TickCount;

unsigned long LIDAR_INT2_HIGH_TickCount;
unsigned long LIDAR_INT2_LOW_TickCount;

unsigned long ARRIVE_BTN_INT_Press_TickCount;
unsigned long ARRIVE_BTN_INT_Release_TickCount;

unsigned long GENERAL_BTN_INT_Press_TickCount;
unsigned long GENERAL_BTN_INT_Release_TickCount;

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

void LIDAR_INT1_CB()
{
  if (digitalRead(LIDAR_INT1) == HIGH)
  {
    dbg_printf("LIDAR_INT1 HIGH\r\n");
    LIDAR_INT1_LOW_TickCount=0;
    LIDAR_INT1_HIGH_TickCount=millis();

  }
  else
  {
    dbg_printf("LIDAR_INT1 LOW\r\n");
    LIDAR_INT1_HIGH_TickCount=0;
    LIDAR_INT1_LOW_TickCount=millis();
  }
}

void LIDAR_INT2_CB()
{
  if (digitalRead(LIDAR_INT2) == HIGH)
  {
    dbg_printf("LIDAR_INT2 HIGH\r\n");
    LIDAR_INT2_LOW_TickCount=0;
    LIDAR_INT2_HIGH_TickCount=millis();

  }
  else
  {
    dbg_printf("LIDAR_INT2 LOW\r\n");
    LIDAR_INT2_HIGH_TickCount=0;
    LIDAR_INT2_LOW_TickCount=millis();
  }
}

void ARRIVE_BTN_CB()
{
  if (digitalRead(ARRIVE_BTN) == LOW)
  {
    dbg_printf("ARRIVE_BTN_CB press\r\n");
    //Serial.println("ARRIVE_BTN_CB press");
    ARRIVE_BTN_INT_Release_TickCount=0;
    ARRIVE_BTN_INT_Press_TickCount=millis();

  }
  else
  {
    dbg_printf("ARRIVE_BTN_CB release\r\n");
    //Serial.println("ARRIVE_BTN_CB release");
    ARRIVE_BTN_INT_Press_TickCount=0;
    ARRIVE_BTN_INT_Release_TickCount=millis();
  }
}

void GENERAL_BTN_CB()
{
  if (digitalRead(GENERAL_BTN) == LOW)
  {
    dbg_printf("GENERAL_BTN_CB press\r\n");
    //Serial.println("GENERAL_BTN_CB press");
    GENERAL_BTN_INT_Release_TickCount=0;
    GENERAL_BTN_INT_Press_TickCount=millis();

  }
  else
  {
    dbg_printf("GENERAL_BTN_CB release\r\n");
    //Serial.println("GENERAL_BTN_CB release");
    GENERAL_BTN_INT_Press_TickCount=0;
    GENERAL_BTN_INT_Release_TickCount=millis();
  }
}
void FITPM::begin()

{
  FITDBGbegin();
  initFITLM5066I();
  initFITBTT6030();
  batteryLowNotifyInterval=0;
  batteryLowNotifyCount=0;
  battery_callback_func_ptr=NULL;

  pinMode(SLEEP_ENABLE_PIN, OUTPUT);
  digitalWrite(SLEEP_ENABLE_PIN, HIGH);

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

  LiadrWarning_callback_func_ptr=NULL;
  Button_callback_func_ptr=NULL;

  LIDAR_INT1_HIGH_TickCount=0;
  LIDAR_INT1_LOW_TickCount=0;
  bLiadr_INT1_HIGH=false;
  bLiadr_INT1_ACTION=false;
  pinMode(LIDAR_INT1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIDAR_INT1),LIDAR_INT1_CB, CHANGE);
  LIDAR_INT1_CB();

  LIDAR_INT2_HIGH_TickCount=0;
  LIDAR_INT2_LOW_TickCount=0;
  bLiadr_INT2_HIGH=false;
  bLiadr_INT2_ACTION=false;
  pinMode(LIDAR_INT2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIDAR_INT2),LIDAR_INT2_CB, CHANGE);
  LIDAR_INT2_CB();


  ARRIVE_BTN_INT_Press_TickCount=0;
  ARRIVE_BTN_INT_Release_TickCount=0;
  bArrive_BTN_Press=false;
  bArrive_ACTION=false;
  pinMode(ARRIVE_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ARRIVE_BTN),ARRIVE_BTN_CB, CHANGE);
  ARRIVE_BTN_CB();

  GENERAL_BTN_INT_Press_TickCount=0;
  GENERAL_BTN_INT_Release_TickCount=0;
  bGeneral_BTN_Press=false;
  bGeneral_ACTION=false;
  pinMode(GENERAL_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GENERAL_BTN),GENERAL_BTN_CB, CHANGE);
  GENERAL_BTN_CB();
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
        Estop_callback_func_ptr(E_STOP_ENABLE,true);
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
        Estop_callback_func_ptr(E_STOP_DISABLE,true);
      }
      dbg_printf("E_STOP_BTN E_STOP_DISABLE\r\n");
    }
  }
}

void FITPM::registEStopCallBack(void (*callback_func_ptr)(uint8_t,bool))
{
  Estop_callback_func_ptr=callback_func_ptr;
}
#endif

void FITPM::process_Liadr_Warning_Interrupt()
{
  if(!bLiadr_INT1_HIGH)
  {
    if(LIDAR_INT1_HIGH_TickCount!=0)
    {
      if(millis()-LIDAR_INT1_HIGH_TickCount>DEBOUNCE_TIME)
      {
        LIDAR_INT1_HIGH_TickCount=0;
        bLiadr_INT1_HIGH=true;
        //dbg_printf("process_E_STOP_BTN press\r\n");
      }
    }
  }
  else
  {
    if(LIDAR_INT1_LOW_TickCount!=0)
    {
      if(millis()-LIDAR_INT1_LOW_TickCount>DEBOUNCE_TIME)
      {
        LIDAR_INT1_LOW_TickCount=0;
        bLiadr_INT1_HIGH=false;
        //dbg_printf("process_E_STOP_BTN release\r\n");
      }
    }
  }

  if(!bLiadr_INT2_HIGH)
  {
    if(LIDAR_INT2_HIGH_TickCount!=0)
    {
      if(millis()-LIDAR_INT2_HIGH_TickCount>DEBOUNCE_TIME)
      {
        LIDAR_INT2_HIGH_TickCount=0;
        bLiadr_INT2_HIGH=true;
        //dbg_printf("process_E_STOP_BTN press\r\n");
      }
    }
  }
  else
  {
    if(LIDAR_INT2_LOW_TickCount!=0)
    {
      if(millis()-LIDAR_INT2_LOW_TickCount>DEBOUNCE_TIME)
      {
        LIDAR_INT2_LOW_TickCount=0;
        bLiadr_INT2_HIGH=false;
        //dbg_printf("process_E_STOP_BTN release\r\n");
      }
    }
  }

  if(bLiadr_INT1_HIGH)
  {
    if(!bLiadr_INT1_ACTION)
    {
      bLiadr_INT1_ACTION=true;
      if(LiadrWarning_callback_func_ptr!=NULL)
      {
        LiadrWarning_callback_func_ptr(LIADR_ID_1,LIADR_STATUS_WARRING);
      }
      dbg_printf("Liadr_INT1 Warning\r\n");
    }
  }
  else
  {
    if(bLiadr_INT1_ACTION)
    {
      bLiadr_INT1_ACTION=false;
      if(LiadrWarning_callback_func_ptr!=NULL)
      {
        LiadrWarning_callback_func_ptr(LIADR_ID_1,LIADR_STATUS_NONE);
      }
      dbg_printf("Liadr_INT1 None\r\n");
    }
  }

  if(bLiadr_INT2_HIGH)
  {
    if(!bLiadr_INT2_ACTION)
    {
      bLiadr_INT2_ACTION=true;
      if(LiadrWarning_callback_func_ptr!=NULL)
      {
        LiadrWarning_callback_func_ptr(LIADR_ID_2,LIADR_STATUS_WARRING);
      }
      dbg_printf("Liadr_INT2 Warning\r\n");
    }
  }
  else
  {
    if(bLiadr_INT2_ACTION)
    {
      bLiadr_INT2_ACTION=false;
      if(LiadrWarning_callback_func_ptr!=NULL)
      {
        LiadrWarning_callback_func_ptr(LIADR_ID_2,LIADR_STATUS_NONE);
      }
      dbg_printf("Liadr_INT2 None\r\n");
    }
  }
}

void FITPM::registLiadrWarningCallBack(void (*callback_func_ptr)(uint8_t,uint8_t))
{
  LiadrWarning_callback_func_ptr=callback_func_ptr;
}

void FITPM::process_Button_Interrupt()
{
  if(!bArrive_BTN_Press)
  {
    if(ARRIVE_BTN_INT_Press_TickCount!=0)
    {
      if(millis()-ARRIVE_BTN_INT_Press_TickCount>DEBOUNCE_TIME)
      {
        ARRIVE_BTN_INT_Press_TickCount=0;
        bArrive_BTN_Press=true;
        //dbg_printf("process_E_STOP_BTN press\r\n");
      }
    }
  }
  else
  {
    if(ARRIVE_BTN_INT_Release_TickCount!=0)
    {
      if(millis()-ARRIVE_BTN_INT_Release_TickCount>DEBOUNCE_TIME)
      {
        ARRIVE_BTN_INT_Release_TickCount=0;
        if(bArrive_BTN_Press)
        {
          bArrive_ACTION=true;
        }
        bArrive_BTN_Press=false;
        //dbg_printf("process_E_STOP_BTN release\r\n");
      }
    }
  }

  if(!bGeneral_BTN_Press)
  {
    if(GENERAL_BTN_INT_Press_TickCount!=0)
    {
      if(millis()-GENERAL_BTN_INT_Press_TickCount>DEBOUNCE_TIME)
      {
        GENERAL_BTN_INT_Press_TickCount=0;
        bGeneral_BTN_Press=true;
        //dbg_printf("process_E_STOP_BTN press\r\n");
      }
    }
  }
  else
  {
    if(GENERAL_BTN_INT_Release_TickCount!=0)
    {
      if(millis()-GENERAL_BTN_INT_Release_TickCount>DEBOUNCE_TIME)
      {
        GENERAL_BTN_INT_Release_TickCount=0;
        if(bGeneral_BTN_Press)
        {
          bGeneral_ACTION=true;
        }
        bGeneral_BTN_Press=false;
        //dbg_printf("process_E_STOP_BTN release\r\n");
      }
    }
  }

  if(bArrive_ACTION)
  {
    bArrive_ACTION=false;
    if(Button_callback_func_ptr!=NULL)
    {
      Button_callback_func_ptr(BTN_TYPE_ARRIVE,BTN_STATUS_ACTION);
    }
    dbg_printf("bArrive_ACTION\r\n");
  }


  if(bGeneral_ACTION)
  {
    bGeneral_ACTION=false;
    if(Button_callback_func_ptr!=NULL)
    {
      Button_callback_func_ptr(BTN_TYPE_GENERAL,BTN_STATUS_ACTION);
    }
    dbg_printf("bGeneral_ACTION\r\n");
  }
}

void FITPM::registButtonCallBack(void (*callback_func_ptr)(uint8_t,uint8_t))
{
  Button_callback_func_ptr=callback_func_ptr;
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

void FITPM::setLidarEnterSleepMode(uint8_t status)
{
  if(status==LIDAR_SLEEP_DISABLE)
  {
    digitalWrite(SLEEP_ENABLE_PIN, HIGH);
  }
  else
  {
    digitalWrite(SLEEP_ENABLE_PIN, LOW);
  }
}

void FITPM::processPowerManagement()
{
  #if defined(SOFT_E_STOP)
  process_E_STOP_BTN();
  #endif
  process_Button_Interrupt();
  process_Liadr_Warning_Interrupt();
  processLM5066I();
  processBTT6030();
  updateBatteryPercentage();
  process_led();
}
