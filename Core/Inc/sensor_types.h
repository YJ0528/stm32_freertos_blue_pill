#ifndef __SENSOR_TYPES_H
#define __SENSOR_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    uint8_t digital_value; 
    uint16_t analog_value;
} IR_Sensor_Data_t;

typedef struct {
    uint8_t digital_value;
    uint16_t analog_value;
} LDR_Sensor_Data_t;

typedef struct {
    uint8_t RHI, RHD, TCI, TCD, SUM;
} DHT11_Sensor_Data_t;

typedef struct {  
    int16_t encoder_state;
    int16_t encoder_count;
    int16_t enc_difference;
    int16_t buzzer_params_select;
} Enc_SensorData_t;

typedef struct {  
    int32_t frequency;
    int16_t volume;
} Buzzer_Data_t;


typedef struct {
    Enc_SensorData_t encData;
    DHT11_Sensor_Data_t dht11Data;
    IR_Sensor_Data_t irData;
    LDR_Sensor_Data_t ldrData;
    Buzzer_Data_t buzzerData;
} Combined_Sensor_Data_t;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_TYPES_H */