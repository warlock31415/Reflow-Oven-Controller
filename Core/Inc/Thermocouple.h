#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include<stdint.h>


#define ThermocoupleADDR 0x67<<1


typedef enum ThermocoupleRegisters{
    Reg_Th,
    Reg_Tdelta,
    Reg_Tc,
    Reg_ADCRaw,
    Reg_Status,
    Reg_TCConfig,
    Reg_DeviceConfig

}ThermocoupleRegisters;

typedef enum SensorType{
    Type_K,
    Type_J,
    Type_T,
    Type_N,
    Type_S,
    Type_E,
    Type_B,
    Type_R
}SensorType;





#endif
