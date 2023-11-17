#ifndef FITLM5066I_H
#define FITLM5066I_H

#include <Wire.h>

#define UPDATE_AVG_BLOCK_INTERVAL 30000

#define LM5066I_REG_OPERATION             0x01
#define LM5066I_REG_CLEAR_FAULTS          0x03
#define LM5066I_REG_CAPABILITY            0x19
#define LM5066I_REG_VOUT_UV_WARN_LIMIT    0x43
#define LM5066I_REG_OT_FAULT_LIMIT        0x4F
#define LM5066I_REG_OT_WARN_LIMIT         0x51
#define LM5066I_REG_VIN_OV_WARN_LIMIT     0x57
#define LM5066I_REG_VIN_UV_WARN_LIMIT     0x58
#define LM5066I_REG_IIN_OC_WARN_LIMIT     0x5D
#define LM5066I_REG_STATUS_BYTE           0x78
#define LM5066I_REG_STATUS_WORD           0x79
#define LM5066I_REG_STATUS_VOUT           0x7A
#define LM5066I_REG_STATUS_INPUT          0x7C
#define LM5066I_REG_STATUS_TEMPERATURE    0x7D
#define LM5066I_REG_STATUS_CML            0x7E
#define LM5066I_REG_STATUS_OTHER          0x7F
#define LM5066I_REG_STATUS_MFR_SPECIFIC   0x80
#define LM5066I_REG_READ_EIN              0x86
#define LM5066I_REG_READ_VIN              0x88
#define LM5066I_REG_READ_IIN              0x89
#define LM5066I_REG_READ_VOUT             0x8B
#define LM5066I_REG_READ_TEMPERATURE_1    0x8D
#define LM5066I_REG_READ_PIN              0x97
#define LM5066I_REG_MFR_ID                0x99
#define LM5066I_REG_MFR_MODEL             0x9A
#define LM5066I_REG_MFR_REVISION          0x9B
#define LM5066I_REG_READ_VAUX             0xD0
#define LM5066I_REG_MFR_READ_IIN          0xD1
#define LM5066I_REG_MFR_READ_PIN          0xD2
#define LM5066I_REG_MFR_IIN_OC_WARN_LIMIT 0xD3
#define LM5066I_REG_MFR_PIN_OP_WARN_LIMIT 0xD4
#define LM5066I_REG_READ_PIN_PEAK         0xD5
#define LM5066I_REG_CLEAR_PIN_PEAK        0xD6
#define LM5066I_REG_GATE_MASK             0xD7
#define LM5066I_REG_ALERT_MASK            0xD8
#define LM5066I_REG_DEVICE_SETUP          0xD9
#define LM5066I_REG_BLOCK_READ            0xDA
#define LM5066I_REG_SAMPLES_FOR_AVG       0xDB
#define LM5066I_REG_READ_AVG_VIN          0xDC
#define LM5066I_REG_READ_AVG_VOUT         0xDD
#define LM5066I_REG_READ_AVG_IIN          0xDE
#define LM5066I_REG_READ_AVG_PIN          0xDF
#define LM5066I_REG_BLACK_BOX_READ        0xE0
#define LM5066I_REG_DIAGNOSTIC_WORD_READ  0xE1
#define LM5066I_REG_AVG_BLOCK_READ        0xE2

#define BATTERY_MAX_VOLTAGE 28.6
#define BATTERY_MIN_VOLTAGE 20.3

class FITLM5066I{

public:
FITLM5066I(int slaveAddr = 0x15,uint32_t i2cspeed=100000);

~FITLM5066I ();
bool getLM5600IChipInfo();
void getAVGBlockInfo();
void CalculatePercentage(double voltage);
void initFITLM5066I();
void processLM5066I();

protected:
uint8_t lm5600iChipInfo[13];
int slaveAddr;
uint32_t i2cspeed;
unsigned long GetAVGBlockInterval;//ms
unsigned long GetAVGBlockCount;//ms
uint16_t DIAGNOSTIC_WORD;
double AVG_IIN;
double AVG_VOUT;
double AVG_VIN;
double AVG_PIN;
double TEMPERATURE;
int percentage=0;
int Prvpercentage=0;
bool binitSuccess;
};

#endif//FITLM5066I_H