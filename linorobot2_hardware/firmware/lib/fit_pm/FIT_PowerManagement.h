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
void registEStopCallBack(void (*callback_func_ptr)(uint8_t));
void (*Estop_callback_func_ptr)(uint8_t);
#endif
void begin();

protected:
unsigned long batteryLowNotifyInterval;//ms
unsigned long batteryLowNotifyCount;//ms
#if defined(SOFT_E_STOP)
bool bE_STOP_BTN_Press;
bool bE_STOP_ACTION;
#endif

};

#endif//FITPM_H
