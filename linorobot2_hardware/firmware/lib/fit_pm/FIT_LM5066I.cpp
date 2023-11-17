#include <Arduino.h>
#include "FIT_LM5066I.h"
#include "debugmessage.h"

FITLM5066I::FITLM5066I(int addr,uint32_t speed)
{
  slaveAddr = addr;
  i2cspeed = speed;
  GetAVGBlockInterval=UPDATE_AVG_BLOCK_INTERVAL;
}

FITLM5066I::~FITLM5066I(){/*nothing to do*/}

void FITLM5066I::initFITLM5066I()
{
  Wire1.begin();
  Wire1.setClock(i2cspeed);
  GetAVGBlockCount=0;
  percentage=0;
  Prvpercentage=0;
  DIAGNOSTIC_WORD=0;
  AVG_IIN=0;
  AVG_VOUT=0;
  AVG_VIN=0;
  AVG_PIN=0;
  TEMPERATURE=0;
  binitSuccess=false;
  getLM5600IChipInfo();
  getAVGBlockInfo();
}

bool FITLM5066I::getLM5600IChipInfo()
{
  uint8_t infoIndex=0;
  uint8_t len=0;

  Wire1.beginTransmission(slaveAddr);
  Wire1.write(LM5066I_REG_MFR_ID);
  Wire1.endTransmission(false);

  Wire1.requestFrom(slaveAddr,4);
  while(Wire1.available()) 
  {
    if(len==0)
    {
      len = Wire1.read();
    }
    else
    {
      lm5600iChipInfo[infoIndex] = Wire1.read();
      infoIndex++;
    }
  }

  len=0;
  lm5600iChipInfo[infoIndex-1]=0x2D;

  Wire1.beginTransmission(slaveAddr);
  Wire1.write(LM5066I_REG_MFR_MODEL);
  Wire1.endTransmission(false);

  Wire1.requestFrom(slaveAddr,9);
  while(Wire1.available()) 
  {
    if(len==0)
    {
      len = Wire1.read();
    }
    else
    {
      lm5600iChipInfo[infoIndex] = Wire1.read();
      infoIndex++;
    }
  }

  len=0;
  lm5600iChipInfo[infoIndex-1]=0x2D;

  Wire1.beginTransmission(slaveAddr);
  Wire1.write(LM5066I_REG_MFR_REVISION);
  Wire1.endTransmission(false);

  Wire1.requestFrom(slaveAddr,2);
  while(Wire1.available()) 
  {
    if(len==0)
    {
      len = Wire1.read();
    }
    else
    {
      lm5600iChipInfo[infoIndex] = Wire1.read();
      infoIndex++;
    }
  }

  dbg_printf("(%d)%s\r\n",infoIndex,lm5600iChipInfo);

  if(infoIndex>=12)
    binitSuccess=true;
  else
    binitSuccess=false;

  return binitSuccess;
}

void FITLM5066I::getAVGBlockInfo()
{
  uint8_t infoIndex=0;
  uint8_t readBuf[13];

  if(!binitSuccess)
  {
    dbg_printf("FITLM5066I i2c fail\r\n");
    return;
  }

  Wire1.beginTransmission(slaveAddr);
  Wire1.write(LM5066I_REG_AVG_BLOCK_READ);
  Wire1.endTransmission(false);

  Wire1.requestFrom(slaveAddr,13);
  while(Wire1.available())
  {
    readBuf[infoIndex] = Wire1.read();
    infoIndex++;
  }

  DIAGNOSTIC_WORD = readBuf[1]|readBuf[2]<<8;
  AVG_IIN = ((readBuf[3]|readBuf[4]<<8)*pow(10,2)+503.9)/15067;
  AVG_VOUT = ((readBuf[5]|readBuf[6]<<8)*pow(10,2)-500)/4602;
  AVG_VIN = ((readBuf[7]|readBuf[8]<<8)*pow(10,2)+140)/4617;
  AVG_PIN = ((readBuf[9]|readBuf[10]<<8)*pow(10,3)+4000)/1701;
  TEMPERATURE =((readBuf[11]|readBuf[12]<<8)*pow(10,3)+0)/16000;

  CalculatePercentage(AVG_VIN);

  dbg_printf("DIAGNOSTIC_WORD:%x\r\n",DIAGNOSTIC_WORD);
  dbg_printf("AVG_IIN:%04f\r\n",AVG_IIN);
  dbg_printf("AVG_VOUT:%04f\r\n",AVG_VOUT);
  dbg_printf("AVG_VIN:%04f\r\n",AVG_VIN);
  dbg_printf("AVG_PIN:%04f\r\n",AVG_PIN);
  dbg_printf("Temperature:%04f\r\n",TEMPERATURE);
  dbg_printf("percentage:%d\r\n",percentage);

}

void FITLM5066I::CalculatePercentage(double voltage)
{
  if(voltage>BATTERY_MAX_VOLTAGE)
    voltage=BATTERY_MAX_VOLTAGE;

  if(voltage<BATTERY_MIN_VOLTAGE)
    voltage=BATTERY_MIN_VOLTAGE;

  percentage = (voltage - BATTERY_MIN_VOLTAGE)*100/(BATTERY_MAX_VOLTAGE-BATTERY_MIN_VOLTAGE);

  if(percentage>100)
    percentage=100;
}

void FITLM5066I::processLM5066I()
{
  if(GetAVGBlockCount==0)
  {
    GetAVGBlockCount = millis();
  }
  else
  {
    if(millis()-GetAVGBlockCount>=GetAVGBlockInterval)
    {
      getAVGBlockInfo();
      GetAVGBlockCount=0;
    }
  }
}
