#ifndef FITPM_H
#define FITPM_H

#include "FIT_BTT6030.h"
#include "FIT_LM5066I.h"
#include "FIT_LED.h"

#define LM5066I_SLAVE_ADDR 0x15
#define LM5066I_I2C_SPEED 100000
#define BTT6030_IS_L_PIN 26
#define BTT6030_IS_R_PIN 27
#define BTT6030_IN_L_PIN 11
#define BTT6030_IN_R_PIN 12
#define BATTERY_LOW_PERCENTAGE 20


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
void begin();
protected:
unsigned long batteryLowNotifyInterval;//ms
unsigned long batteryLowNotifyCount;//ms
};

#endif//FITPM_H