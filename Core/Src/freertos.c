/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "stdio.h" 
#include "tim.h"
#include "sensor_types.h"
#include "mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM_FREQ 72000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

// Structure to hold sensor data


// Original variables
int previous;
float tCelsius;
float tFahrenheit;
float RH;
uint8_t TFI;
uint8_t TFD;
uint32_t pMillis, cMillis;
char snum[32];
char strCopy[15];

/* USER CODE END Variables */
osThreadId sensorTaskHandle;
osThreadId displayTaskHandle;
osThreadId hardwareTaskHandle;
osMessageQId sensorQueueHandle;
osMessageQId DHT11SensorQueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

uint8_t DHT11_Start (void);
uint8_t DHT11_Read (void);
void microDelay (uint16_t delay);

/* USER CODE END FunctionPrototypes */

void StartSensorTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartHardwareTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 5 */
__weak void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of sensorQueue */
  osMessageQDef(sensorQueue, 32, uint32_t);
  sensorQueueHandle = osMessageCreate(osMessageQ(sensorQueue), NULL);

  /* definition and creation of DHT11SensorQueue */
  osMessageQDef(DHT11SensorQueue, 16, uint32_t);
  DHT11SensorQueueHandle = osMessageCreate(osMessageQ(DHT11SensorQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of sensorTask */
  osThreadDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128);
  sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, StartDisplayTask, osPriorityNormal, 0, 256);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* definition and creation of hardwareTask */
  osThreadDef(hardwareTask, StartHardwareTask, osPriorityNormal, 0, 128);
  hardwareTaskHandle = osThreadCreate(osThread(hardwareTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartSensorTask */
/**
  * @brief  Function implementing the sensorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
    static uint16_t hadc1_values[1];
    static uint16_t hadc2_values[1];
    static int16_t enc_previous_count = 0;
    static int16_t enc_current_count = 0;
    static int16_t button_previous = 0;
    static int16_t button_current = 0;
    
   // Static allocation to ensure data remains valid
    static Enc_SensorData_t EncSensorData = {0};
    static LDR_Sensor_Data_t LDRSensorData = {0};
    static IR_Sensor_Data_t IRSsensorData = {0};
    static DHT11_Sensor_Data_t DHT11SensorData = {0};
    static Buzzer_Data_t buzzerSignalData = {0};
    static Servo_Data_t servoPWMSignalData = {0};
    static MPU6050_t MPU6050 = {0};

    static Queue_Message_t encMsg = {0};
    static Queue_Message_t ldrMsg = {0};
    static Queue_Message_t irMsg = {0};
    static Queue_Message_t dht11Msg = {0};
    static Queue_Message_t buzzerMsg = {0};
    static Queue_Message_t servoMsg = {0};
    static Queue_Message_t mpu6050Msg = {0};

    static Combined_Sensor_Data_t combinedData = {0};


    // Initialize message structures once
    encMsg.type = MSG_TYPE_ENC;
    encMsg.data = &EncSensorData;
    
    ldrMsg.type = MSG_TYPE_IR;
    ldrMsg.data = &LDRSensorData;
    
    irMsg.type = MSG_TYPE_LDR;
    irMsg.data = &IRSsensorData;
    
    dht11Msg.type = MSG_TYPE_DHT11;
    dht11Msg.data = &DHT11SensorData;

    buzzerMsg.type = MSG_TYPE_BUZZER;
    buzzerMsg.data = &buzzerSignalData;

    servoMsg.type = MSG_TYPE_SERVO;
    servoMsg.data = &servoPWMSignalData;

    mpu6050Msg.type = MSG_TYPE_MPU6050;
    mpu6050Msg.data = &MPU6050;

    // Initialize buzzer states based on the timer settings
    buzzerSignalData.frequency = 7500;
    buzzerSignalData.volume =  100 * htim3.Instance->CCR1 / htim3.Instance->ARR;

    // Initialize servo states to 90 degree
    servoPWMSignalData.angle =  90;

    // Declare pointers to ADC readings
    static uint16_t* ADC1_IN2;
    static uint16_t* ADC1_IN4;

    // Assign pointers address
    ADC1_IN2 = &LDRSensorData.analog_value;
    ADC1_IN4 = &IRSsensorData.analog_value;

    // Wait MPU6050 to finish initialization
    while (MPU6050_Init(&hi2c2) == 1);

  /* Infinite loop */
  for(;;)
  {
    // Read sensor analog values (Alternative)
    // LDRSensorData.analog_value = Sensor_ReadAnalog (&hadc1);
    // IRSsensorData.analog_value = Sensor_ReadAnalog (&hadc2);

    // Read sensor analog values (Whole HADC)
    if(Read_ADC_AllChannels(&hadc1, hadc1_values, 1))
      {
        // LDRSensorData.analog_value = hadc1_values[0];
        *ADC1_IN2 = hadc1_values[0]; 
      }

    if(Read_ADC_AllChannels(&hadc2, hadc2_values, 1))
      {
        // IRSsensorData.analog_value = hadc2_values[0];
        *ADC1_IN4 = hadc2_values[0]; 
      }

    // Read sensor digital values
    LDRSensorData.digital_value = HAL_GPIO_ReadPin(LDR_SENSOR_DIGITAL_GPIO_Port, LDR_SENSOR_DIGITAL_Pin);
    IRSsensorData.digital_value = HAL_GPIO_ReadPin(IR_SENSOR_DIGITAL_GPIO_Port, IR_SENSOR_DIGITAL_Pin);

    // Read MPU6050 values
    MPU6050_Read_All(&hi2c2, &MPU6050);
    HAL_Delay(10);
  
    // Read encoder
    // Check encoder selected function (changing slides or configure hardware parameter)
    button_current = HAL_GPIO_ReadPin(BUTTON_BLUE_GPIO_Port, BUTTON_BLUE_Pin);
    if (button_previous != button_current && button_current != 1) EncSensorData.directory[1] += 1;
    button_previous = button_current;

    EncSensorData.enc_difference = TIM1->CNT - enc_previous_count;
    enc_previous_count = TIM1->CNT;
    if (EncSensorData.enc_difference > 32767)EncSensorData.enc_difference -= 65533;
    else if (EncSensorData.enc_difference < -32767) EncSensorData.enc_difference += 65533;
    
    if (EncSensorData.directory[1]%3 && 
        ((enc_current_count >= 0 && enc_current_count/4 % 6 >= 4) ||
        (enc_current_count < 0 && enc_current_count/4 % 6 >= -2)))    
    {
      
      switch (enc_current_count / 4 % 6)
      {
        // Buzzer parameters
        case 4: case -2:
        switch (EncSensorData.directory[1])
        {
          case 1:
          buzzerSignalData.volume += EncSensorData.enc_difference;
          if (buzzerSignalData.volume > 100) buzzerSignalData.volume = 100;
          if (buzzerSignalData.volume < 0) buzzerSignalData.volume = 0;
          break;

          case 2:
          buzzerSignalData.frequency += EncSensorData.enc_difference*100;
          if (buzzerSignalData.frequency > 30000) buzzerSignalData.frequency  = 30000;
          if (buzzerSignalData.frequency < 0) buzzerSignalData.frequency = 0;
          break;
        }
        break; 

        // Servo motor parameters
        case 5: case -1:
        switch (EncSensorData.directory[1])
        {
          case 1:
          servoPWMSignalData.angle += EncSensorData.enc_difference;
          if (servoPWMSignalData.angle > 180) servoPWMSignalData.angle  = 180;
          if (servoPWMSignalData.angle < 0) servoPWMSignalData.angle = 0;
          break;
          case 2:
          EncSensorData.directory[1] += 1;
          break;
        }
        break; 
      }
    }
    else
    {
      enc_current_count += EncSensorData.enc_difference;
      EncSensorData.directory[0] = enc_current_count/4;
      EncSensorData.directory[1] = 0;
    }
    // Read DHT11 sensor values
    // if(DHT11_Start())
    // {
    //   DHT11SensorData.RHI = DHT11_Read(); // Relative humidity integral
    //   DHT11SensorData.RHD = DHT11_Read(); // Relative humidity decimal
    //   DHT11SensorData.TCI = DHT11_Read(); // Celsius integral
    //   DHT11SensorData.TCD = DHT11_Read(); // Celsius decimal
    //   DHT11SensorData.SUM = DHT11_Read(); // Check sum
    // }
    // else 
    // {
    //   SSD1306_Clear();
    //   SSD1306_GotoXY(0,0);
    //   SSD1306_Puts("FAILED", &Font_11x18, 1);
    //   SSD1306_UpdateScreen();
    // }

    // Setup and send signals message
    combinedData.encData = EncSensorData;
    combinedData.ldrData = LDRSensorData;
    combinedData.irData = IRSsensorData;
    combinedData.dht11Data = DHT11SensorData;
    combinedData.buzzerData = buzzerSignalData;
    combinedData.servoData = servoPWMSignalData;
    combinedData.mpuData = MPU6050;
    osMessagePut(sensorQueueHandle, (uint32_t)&combinedData, 0);

    osDelay(1);

    // Alternate: Control the os time
    // osDelayUntil(&PreviousWakeTime, period);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */

  // Initialize sensor msg type object
  static Queue_Message_t* receivedMsg;

  // Initialize sensor data object
  static Enc_SensorData_t receivedEncData = {0};
  static IR_Sensor_Data_t receivedIRData = {0};
  static LDR_Sensor_Data_t receivedLDR1Data = {0};
  static DHT11_Sensor_Data_t receivedDHT11Data = {0};
  static Buzzer_Data_t receivedBuzzerData = {0};
  static Servo_Data_t receivedServoData = {0};
  static MPU6050_t receivedMPU6050Data = {0};
  
  
  static Combined_Sensor_Data_t* receivedData;

  previous = 0;
  SSD1306_Init();
  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("Hello,", &Font_11x18, 1);
  SSD1306_GotoXY (0, 30);
  SSD1306_Puts ("World!", &Font_11x18, 1);
  SSD1306_UpdateScreen();
  HAL_Delay (500);

  SSD1306_ScrollRight(0,7);
  HAL_Delay(500);
  SSD1306_ScrollLeft(0,7);
  HAL_Delay(500);
  SSD1306_Stopscroll();
  SSD1306_Clear();
  
  /* Infinite loop */
  for(;;)
  {

    osEvent event = osMessageGet(sensorQueueHandle, 100);
    if (event.status == osEventMessage)
    {
      // Unpack the messages to local
      Combined_Sensor_Data_t* receivedData = (Combined_Sensor_Data_t*)event.value.p;

      // Unpack the encoder data
      receivedEncData = receivedData->encData;

      // Compare the encoder value with the previous
      if ((receivedEncData.directory[0] % 6) != previous)
      {
        SSD1306_Clear();
        previous = receivedEncData.directory[0] % 6;
      }

      // Display the current page info
      switch (receivedEncData.directory[0] % 6) {

        case 0:
        // Unpack MPU6050 data (for the hardware temperature only)
        receivedMPU6050Data = receivedData->mpuData;

        // Get current tick count and convert to time units
        uint32_t currentTicks = osKernelSysTick();
        uint32_t seconds = (currentTicks / 1000) % 60;  // Seconds from milliseconds
        uint32_t minutes = (currentTicks / 60000) % 60; // Minutes from milliseconds
        uint32_t hours = (currentTicks / 3600000) % 24; // Hours from milliseconds

        // Format and display the time in HH:MM:SS
        SSD1306_DrawFilledRectangle (4, 6, 116 , 26, 1);
        sprintf(snum, " %02lu:%02lu:%02lu ", hours, minutes, seconds);
        SSD1306_GotoXY(4, 12);
        SSD1306_Puts(snum, &Font_11x18, 0);

        // Hardware Temperature
        sprintf(snum, "Hw Temp: %2d.%02dC", (int16_t)receivedMPU6050Data.Temperature, (int16_t)(receivedMPU6050Data.Temperature*100)%100);
        SSD1306_GotoXY(0, 41);
        SSD1306_Puts(snum, &Font_7x10, 1);

        // Display uptime in total seconds
        sprintf(snum, "Uptime:  %lus", currentTicks / 1000);
        SSD1306_GotoXY(0, 52);
        SSD1306_Puts(snum, &Font_7x10, 1);      
        break;

        case 1: case -5:  
        // Unpack IR sensor data
        receivedIRData = receivedData->irData;
      
        // Display IR sensor data 
        SSD1306_GotoXY(0,0);
        SSD1306_Puts("IR Sensor", &Font_11x18, 1);
        sprintf(snum, "Analog   :%4d", receivedIRData.analog_value);
        SSD1306_GotoXY(0, 30);
        SSD1306_Puts(snum, &Font_7x10, 1);
        sprintf(snum, "Digital : %d", receivedIRData.digital_value);
        SSD1306_GotoXY(0, 45);
        SSD1306_Puts(snum, &Font_7x10, 1);
        break;

        case 2: case -4: 
        // Unpack LDR sensor data
        receivedLDR1Data = receivedData->ldrData;
  
        // Display LDR sensor data
        SSD1306_GotoXY(0,0);
        SSD1306_Puts("LDR Sensor", &Font_11x18, 1);
        sprintf(snum, "Analog   :%4d", receivedLDR1Data.analog_value);
        SSD1306_GotoXY(0, 30);
        SSD1306_Puts(snum, &Font_7x10, 1);
        sprintf(snum, "Digital : %d", receivedLDR1Data.digital_value);
        SSD1306_GotoXY(0, 45);
        SSD1306_Puts(snum, &Font_7x10, 1);
        break;

        case 3: case -3: 

        // Unpack MPU6050 data
        receivedMPU6050Data = receivedData->mpuData;

        // Display MPU6050 data
        SSD1306_GotoXY(0,0);
        SSD1306_Puts("MPU-6050", &Font_11x18, 1);

        // Display Header with partition
        SSD1306_GotoXY(0, 20);
        SSD1306_Puts("Acce:", &Font_7x10, 1);
        SSD1306_GotoXY(70, 20);  // Adjusted x position for "Gyro:"
        SSD1306_Puts("RPY:", &Font_7x10, 1);

        // Draw vertical line partition 
        SSD1306_DrawLine(64, 20, 64, 63, SSD1306_COLOR_WHITE); 

        // Accelerometer X 
        sprintf(snum, "X:%1d.%03d", (int16_t)(receivedMPU6050Data.Ax),
                abs((int16_t)(receivedMPU6050Data.Ax * 100) % 1000));
        SSD1306_GotoXY(0, 31);
        SSD1306_Puts(snum, &Font_7x10, 1);

        // Accelerometer Y 
        sprintf(snum, "Y:%1d.%03d", (int16_t)(receivedMPU6050Data.Ay),
                abs((int16_t)(receivedMPU6050Data.Ay * 100) % 1000));
        SSD1306_GotoXY(0, 42);
        SSD1306_Puts(snum, &Font_7x10, 1);

        // Accelerometer Z 
        sprintf(snum, "Z:%1d.%03d", (int16_t)(receivedMPU6050Data.Az),
                abs((int16_t)(receivedMPU6050Data.Az * 100) % 1000));
        SSD1306_GotoXY(0, 53);
        SSD1306_Puts(snum, &Font_7x10, 1);

        // Gyro roll
        sprintf(snum, "R:%2d.%02d", (int16_t)receivedMPU6050Data.KalmanAngleX,
                abs((int16_t)(receivedMPU6050Data.KalmanAngleX * 100)%100));
        SSD1306_GotoXY(70, 31);
        SSD1306_Puts(snum, &Font_7x10, 1);

        // Gyro pitch
        sprintf(snum, "P:%2d.%02d", (int16_t)receivedMPU6050Data.KalmanAngleY,
                abs((int16_t)(receivedMPU6050Data.KalmanAngleY * 100)%100));
        SSD1306_GotoXY(70, 42);
        SSD1306_Puts(snum, &Font_7x10, 1);

        // Gyro yaw
        sprintf(snum, "Y:%2d.%02d", (int16_t)receivedMPU6050Data.yaw,
                abs((int16_t)(receivedMPU6050Data.yaw * 100)%100));
        SSD1306_GotoXY(70, 53);
        SSD1306_Puts(snum, &Font_7x10, 1);
        break;

        case 4: case -2: 
        // Unpack buzzer data
        receivedBuzzerData = receivedData->buzzerData;

        // Display buzzer data
        SSD1306_GotoXY(0,0);
        SSD1306_Puts("Buzzer", &Font_11x18, 1);
        sprintf(snum, "vol  :    %3d %%", receivedBuzzerData.volume);
        SSD1306_GotoXY(0, 30);
        SSD1306_Puts(snum, &Font_7x10, 1);
        sprintf(snum, "freq :  %4d hz", receivedBuzzerData.frequency);
        SSD1306_GotoXY(0, 45);
        SSD1306_Puts(snum, &Font_7x10, 1);
        break;

        case 5: case -1: 
        // Unpack servo motor data
        receivedServoData = receivedData->servoData;

        // Display servo motor data
        SSD1306_GotoXY(0,0);
        SSD1306_Puts("Servo Motor", &Font_11x18, 1);
        sprintf(snum, "PWM  :    %4d %%", receivedServoData.angle);
        SSD1306_GotoXY(0, 30);
        SSD1306_Puts(snum, &Font_7x10, 1);
        break;
        
        case 6:
        // Unpack the DHT11 sensor data
        receivedDHT11Data = receivedData->dht11Data;

        // Display servo motor data
        if (receivedDHT11Data.RHI + receivedDHT11Data.RHD + receivedDHT11Data.TCI + receivedDHT11Data.TCD == receivedDHT11Data.SUM)
        {
            // Can use RHI and TCI for any purposes if whole number only needed
            tCelsius = (float)receivedDHT11Data.TCI + (float)(receivedDHT11Data.TCD/10.0);
            tFahrenheit = tCelsius * 9/5 + 32;
            RH = (float)receivedDHT11Data.RHI + (float)(receivedDHT11Data.RHD/10.0);
            // Can use tCelsius, tFahrenheit and RH for any purposes
            TFI = tFahrenheit; // Fahrenheit integral
            TFD = tFahrenheit*10-TFI*10; // Fahrenheit decimal
            // Display Temperature data
            SSD1306_GotoXY(0,0);
            SSD1306_Puts("Weather", &Font_11x18, 1);
            sprintf(snum, "Pin state :%d", HAL_GPIO_ReadPin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin));
            SSD1306_GotoXY(0, 30);
            SSD1306_Puts(snum, &Font_7x10, 1);
            sprintf(snum, "Pin type : %d", (DHT11_SENSOR_GPIO_Port->CRL >> (4 * (__builtin_ctz(DHT11_SENSOR_Pin) % 8))) & 0x3);
            SSD1306_GotoXY(0, 45);
            SSD1306_Puts(snum, &Font_7x10, 1);
            // sprintf(strCopy,"%d.%d C ", receivedDHT11Data.TCI, receivedDHT11Data.TCD);
            // SSD1306_GotoXY (0, 30);
            // SSD1306_Puts (strCopy, &Font_7x10, 1);
            // sprintf(strCopy,"%d.%d F ", TFI, TFD);
            // SSD1306_GotoXY (0, 40);
            // SSD1306_Puts (strCopy, &Font_7x10, 1);
            // sprintf(strCopy,"%d.%d %% ", receivedDHT11Data.RHI, receivedDHT11Data.RHD);
            // SSD1306_GotoXY (0, 50);
            // SSD1306_Puts (strCopy, &Font_7x10, 1);
        }
        else
        {
                    
        }
        break;

        default:
        SSD1306_Clear();
        SSD1306_GotoXY(0,0);
        SSD1306_Puts("ERROR", &Font_11x18, 1);
        SSD1306_UpdateScreen();
        break;
      }
      SSD1306_UpdateScreen();
    }
    
    osDelay(1);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartHardwareTask */
/**
* @brief Function implementing the hardwareTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHardwareTask */
void StartHardwareTask(void const * argument)
{
  /* USER CODE BEGIN StartHardwareTask */

  // Initialize sensor msg type object
  static Queue_Message_t* receivedMsg;

  // Initialize sensor data object
  static Enc_SensorData_t receivedEncData = {0};
  static IR_Sensor_Data_t receivedIRData = {0};
  static LDR_Sensor_Data_t receivedLDR1Data = {0};
  // static DHT11_Sensor_Data_t receivedDHT11Data = {0};
  static Buzzer_Data_t receivedBuzzerData = {0};
  static Servo_Data_t receivedServoData = {0};

  static Combined_Sensor_Data_t* receivedData;
  
  static uint32_t buzzerToggleTime = 0;
  static uint8_t buzzerState = 0;
  
  /* Infinite loop */
  for(;;)
  {
    osEvent event = osMessageGet(sensorQueueHandle, 100);
    if (event.status == osEventMessage)
    {
      // Unpack the messages to local
      Combined_Sensor_Data_t* receivedData = (Combined_Sensor_Data_t*)event.value.p;

      // Pointers for signal data
      receivedIRData = receivedData->irData;
      receivedLDR1Data = receivedData->ldrData;
      receivedBuzzerData = receivedData->buzzerData;
      receivedServoData = receivedData->servoData;

      HAL_GPIO_WritePin (LED_BUILDIN_GPIO_Port, LED_BUILDIN_Pin, !receivedLDR1Data.digital_value);
      
      if (!receivedIRData.digital_value)
      {
        if (receivedIRData.analog_value <= 200) setBuzzer(&htim3,TIM_CHANNEL_1, &receivedBuzzerData);
        else
        {
          setBuzzer(&htim3,TIM_CHANNEL_1,&receivedBuzzerData);
          osDelay(pow(sqrt(receivedIRData.analog_value), 1.4));
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
          osDelay(pow(sqrt(receivedIRData.analog_value), 1.4));
        }
      } 
      else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
      
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 500 + ((float)receivedServoData.angle/180 * 2000) );

      



    }
    osDelay(1);
  }
  /* USER CODE END StartHardwareTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */



void setBuzzer(TIM_HandleTypeDef *htim, uint32_t channel, Buzzer_Data_t *Buzzer) 
{  

  // If frequency is 0 or volume is 0, turn off the buzzer
  if (Buzzer->frequency == 0 || Buzzer->volume == 0) {
      __HAL_TIM_SET_COMPARE(htim, channel, 0); // Set duty 100% (OFF for active LOW)
      return;
  }

  // Set the ARR value for frequency control
  __HAL_TIM_SET_PRESCALER(htim, TIM_FREQ/(htim->Instance->ARR * Buzzer->frequency));

  // Calculate CCR value for volume control (inverted for active LOW)
  // Volume is 0-100%, so we calculate the duty cycle as a percentage of ARR
  uint32_t ccr = Buzzer->volume * htim->Instance->ARR / 100;

  // Set the CCR value for the specified channel
  __HAL_TIM_SET_COMPARE(htim, channel, ccr);

}

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  while (__HAL_TIM_GET_COUNTER(&htim4) < delay);
  
}

uint8_t DHT11_Start(void)
{
    
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_SENSOR_Pin;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_SENSOR_GPIO_Port, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin, 0);   // pull the pin low
  HAL_Delay(20);   // wait for 20ms
  HAL_GPIO_WritePin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_SENSOR_GPIO_Port, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  
  return Response;
}

uint8_t DHT11_Read (void)
{
  uint8_t a,b= 0;

  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT11_SENSOR_GPIO_Port, DHT11_SENSOR_Pin)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

/* USER CODE END Application */

