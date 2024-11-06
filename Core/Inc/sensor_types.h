#ifndef __SENSOR_TYPES_H
#define __SENSOR_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
  uint8_t IR_Digital_value;
  uint8_t LDR_Digital_value;
  uint8_t RHI, RHD, TCI, TCD, SUM;
} Sensor_Digital_Data_t;

typedef struct {
  uint16_t LDR_Analog_value;
} hdac1_Analog_Data_t;

typedef struct {
  uint16_t IR_Analog_value;
} hdac2_Analog_Data_t;

typedef struct {  
  uint16_t encoder_count;
} Enc_SensorData_t;

typedef struct {
    Enc_SensorData_t encData;
    Sensor_Digital_Data_t digitalData;
    hdac1_Analog_Data_t adc1Data;
    hdac2_Analog_Data_t adc2Data;
} Combined_Sensor_Data_t;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_TYPES_H */