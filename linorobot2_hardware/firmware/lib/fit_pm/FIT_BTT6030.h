#ifndef FITBTT6030_H
#define FITBTT6030_H

#define ADC_UPDATE_INTERVAL 30
#define OVER_CURRENT 1050
#define OVER_CURRENT_RECOVER_INTERVAL 5000
#define LOWLOAD_CURRENT 15
#define LOWLOAD_CURRENT_INTERVAL 5000
#define OVER_CURRENT_CHECK_TIMES 10

class FITBTT6030{

public:
FITBTT6030(int16_t isLeftPin = 26,int16_t isRightPin = 27, int16_t LeftSwitchPin = 11, int16_t RightSwitchPin = 12);
FITBTT6030();
~FITBTT6030();
void processISAVGvalue();
float getRightISAVGvalue();
float getLeftISvalue();
void enableLeftSwitch(bool enable);
void enableRightSwitch(bool enable);
bool isLeftSwitchOn();
bool isRightSwitchOn();
bool isLeftOverCurrent();
bool isRightOverCurrent();
void processBTT6030();
void initFITBTT6030();

protected:
unsigned long updateISValueInterval;//ms
unsigned long updateISValueTickCount;//ms
int16_t isLeftPin;
int16_t isRightPin;
int16_t LeftSwitchPin;
int16_t RightSwitchPin;
int16_t leftISAVGvalue;
int16_t rightISAVGvalue;
bool bLeftSwitchOn;
bool bRightSwitchOn;
int overCurrent;
int lOverCurrentCount;
int rOverCurrentCount;
int OverCurrentCheckTimes;
unsigned long OverCurrentRecoverInterval;//ms
unsigned long lOverCurrentRecoverCount;//ms
unsigned long rOverCurrentRecoverCount;//ms
int lowloadCurrent;
unsigned long LowloadCurrentInterval;//ms
unsigned long lLowLoadCurrentRecoverCount;//ms
unsigned long rLowLoadCurrentRecoverCount;//ms
bool bLeftOverCurrent;
bool bRightOverCurrent;
};

#endif//FITBTT6030_H