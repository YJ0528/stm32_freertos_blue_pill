#ifndef __SENSOR_TYPES_H
#define __SENSOR_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "mpu6050.h"

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
    int16_t angle;
} Servo_Data_t;

// Define message types
typedef enum {
    MSG_TYPE_ENC,
    MSG_TYPE_IR,
    MSG_TYPE_LDR,
    MSG_TYPE_DHT11,
    MSG_TYPE_BUZZER,
    MSG_TYPE_SERVO,
    MSG_TYPE_MPU6050,
} Message_Type_t;

// Create a wrapper struct for messages
typedef struct {
    Message_Type_t type;    // Type of data
    void* data;            // Pointer to actual data
} Queue_Message_t;

typedef struct {
    Enc_SensorData_t encData;
    DHT11_Sensor_Data_t dht11Data;
    IR_Sensor_Data_t irData;
    LDR_Sensor_Data_t ldrData;
    Buzzer_Data_t buzzerData;
    Servo_Data_t servoData;
    MPU6050_t mpuData;
} Combined_Sensor_Data_t;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_TYPES_H */