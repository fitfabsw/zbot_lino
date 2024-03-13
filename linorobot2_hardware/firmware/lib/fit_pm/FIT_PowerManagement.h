#ifndef FITPM_H
#define FITPM_H

#include "FIT_BTT6030.h"
#include "FIT_LM5066I.h"
#include "FIT_LED.h"

#define LM5066I_SLAVE_ADDR 0x15
#define LM5066I_I2C_SPEED 100000
#define BTT6030_IS_L_PIN 38
#define BTT6030_IS_R_PIN 39
#define BTT6030_IN_L_PIN 11
#define BTT6030_IN_R_PIN 12
#define BATTERY_LOW_PERCENTAGE 20

#define SOFT_E_STOP 1

#if defined(SOFT_E_STOP)
#define DEBOUNCE_TIME 30
#define E_STOP_BUTTON 37
#define E_STOP_DISABLE 0x00
#define E_STOP_ENABLE  0x01
#endif

#define SLEEP_ENABLE_PIN 30
#define LIDAR_INT1       31
#define LIDAR_INT2       32
#define ARRIVE_BTN       34
#define GENERAL_BTN      35

#define BTN_TYPE_ARRIVE  0x01
#define BTN_TYPE_GENERAL 0x02

#define BTN_STATUS_PRESS      0x01
#define BTN_STATUS_RELEASE    0x02
#define BTN_STATUS_ACTION     0x03
#define BTN_STATUS_LONG_PRESS 0x04

#define LIADR_ID_1  0x01
#define LIADR_ID_2  0x02

#define LIADR_STATUS_NONE     0x00
#define LIADR_STATUS_WARRING  0x01

#define LIDAR_SLEEP_DISABLE 0x00
#define LIDAR_SLEEP_ENABLE  0x01

#define LIDAR_READY_DELAY_TIME 8000

class FITPM : public FITLM5066I, public FITBTT6030, public FITLED{

public:
FITPM(int slaveAddr = 0x15,uint32_t i2cspeed=100000,int16_t isLeftPin = 26,int16_t isRightPin = 27,
      int16_t LeftSwitchPin = 11, int16_t RightSwitchPin = 12):FITLM5066I(slaveAddr,i2cspeed),
      FITBTT6030(isLeftPin,isRightPin,LeftSwitchPin,RightSwitchPin){}

~FITPM ();
void processPowerManagement();
void enableHSwitch(double lpwm,double rpwm,bool bSwitchImmediately);
void updateBatteryPercentage();
void registBatteryPercentageCallBack(void (*callback_func_ptr)(int));
void (*battery_callback_func_ptr)(int);
#if defined(SOFT_E_STOP)
void process_E_STOP_BTN();
void registEStopCallBack(void (*callback_func_ptr)(uint8_t,bool));
void (*Estop_callback_func_ptr)(uint8_t,bool);
#endif
void process_Liadr_Warning_Interrupt();
void registLiadrWarningCallBack(void (*callback_func_ptr)(uint8_t,uint8_t));
void (*LiadrWarning_callback_func_ptr)(uint8_t,uint8_t);

void process_Button_Interrupt();
void registButtonCallBack(void (*callback_func_ptr)(uint8_t,uint8_t));
void (*Button_callback_func_ptr)(uint8_t,uint8_t);

void setLidarEnterSleepMode(uint8_t status);
void begin();

protected:
unsigned long batteryLowNotifyInterval;//ms
unsigned long batteryLowNotifyCount;//ms
#if defined(SOFT_E_STOP)
bool bE_STOP_BTN_Press;
bool bE_STOP_ACTION;
#endif

bool bLiadr_INT1_HIGH;
bool bLiadr_INT1_ACTION;

bool bLiadr_INT2_HIGH;
bool bLiadr_INT2_ACTION;

bool bLiadrReady;
unsigned long waittingLiadrReadyCount=0;//ms

bool bArrive_BTN_Press;
bool bArrive_ACTION;

bool bGeneral_BTN_Press;
bool bGeneral_ACTION;

};

#endif//FITPM_H
