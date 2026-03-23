#include "cpp_main.h"
#include "main.h"
#include <cstring>
#include <stdlib.h>

#include "altimeter_ms5607_spi.h"
#include "imu_bmi088_spi.h"
#include "radio_sx127x_spi.h"
#include "gnss_ubloxM8_uart.h"
#include "magnetometer_bmm350_i2c.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"

#pragma pack(push, 1)
    struct afsTelemetryData {
        uint8_t type = 0x00;
        uint32_t timestamp;
        uint8_t state;
        int16_t angularVelocityX = 0xFFFF;
        int16_t angularVelocityY = 0xFFFF;
        int16_t angularVelocityZ = 0xFFFF;
        int16_t accelerationX = 0xFFFF;
        int16_t accelerationY = 0xFFFF;
        int16_t accelerationZ = 0xFFFF;
        int16_t magneticFieldX = 0xFFFF;
        int16_t magneticFieldY = 0xFFFF;
        int16_t magneticFieldZ = 0xFFFF;
        int16_t temperature = 0xFFFF;
        int32_t altitude = 0xFFFFFFFF;
        int32_t ecefPositionX = 0xFFFFFFFF;
        int32_t ecefPositionY = 0xFFFFFFFF;
        int32_t ecefPositionZ = 0xFFFFFFFF;
        uint32_t ecefPositionAccuracy = 0xFFFFFFFF;
        int32_t ecefVelocityX = 0xFFFFFFFF;
        int32_t ecefVelocityY = 0xFFFFFFFF;
        int32_t ecefVelocityZ = 0xFFFFFFFF;
        uint32_t ecefVelocityAccuracy = 0xFFFFFFFF;
        uint16_t crc = 0x0000;
    };
#pragma pack(pop)


// UART inputs from Pi
#pragma pack(push, 1)
typedef struct {
    uint8_t id = 0xBA;  // Every data packet starts with this ID byte
    int8_t  throttle;   // 0 to 100
    int8_t  pitch;      // -100 to 100
    int8_t  roll;       // -100 to 100
    int8_t  yaw;        // -100 to 100
    int8_t  flaps;      // 0 to 100
} RC_Values_t;
#pragma pack(pop)

#define BUF_SIZE 64  // Larger than the struct to handle multiple packets/jitter
__attribute__((aligned(4))) uint8_t rx_dma_buffer[BUF_SIZE];
RC_Values_t latestRCValues;

// Call this once in your main initialization
void Start_RC_Listening(UART_HandleTypeDef *huart) {
    // Start DMA in circular mode
    HAL_UART_Receive_DMA(huart, rx_dma_buffer, BUF_SIZE);
}

// Call this in your main loop to process the buffer
void Process_RC_Data() {
    int latest_found_index = -1;

    for (int i = 0; i <= (BUF_SIZE - sizeof(RC_Values_t)); i++) {
        if (rx_dma_buffer[i] == 0xBA) {
            latest_found_index = i;
            // Don't break! Keep looking for a newer packet further in the buffer
        }
    }

    if (latest_found_index != -1) {
        RC_Values_t *packet = (RC_Values_t *)&rx_dma_buffer[latest_found_index];
        memcpy(&latestRCValues, packet, sizeof(RC_Values_t));
        
        // Clear the ID so we don't re-process this exact memory address 
        // if no new data has arrived by the next loop.
        rx_dma_buffer[latest_found_index] = 0x00;
    }
}



// Add extern defined handles to peripheral interfaces defined in main.c
// extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;


void cpp_main(void) {
    /* Sensor Objects */
    AltimeterMs5607Spi altimeter(&hspi2, ALT_CS_GPIO_Port, ALT_CS_Pin, ALT_MISO_GPIO_Port, ALT_MISO_Pin, 1013.25, 100);
    // ImuBmi088Spi imu()
    GnssUbloxM8Uart gps(&huart1, 100);
    MagBmm350i2c magnetometer(&hi2c1, MAG_INT_GPIO_Port, MAG_INT_Pin, MAG_INT_GPIO_Port, MAG_INT_Pin);

    /* Data packets */
    afsTelemetryData data;
    AltimeterMs5607Spi::Data alt_data;
    // ImuBmi088Spi::Data imu_data;
    GnssUbloxM8Uart::Data gps_data;
    MagBmm350i2c::Data mag_data;

    /* Sensor Inits */
    volatile int lol1 = altimeter.Reset();
    magnetometer.Reset();
    HAL_Delay(100);

    volatile AltimeterMs5607Spi::State lol2 = altimeter.Init();
    magnetometer.Init();

    // turn on Servo and Pi Buck converter
    HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_SERVO_GPIO_Port,EN_SERVO_Pin, GPIO_PIN_SET); // Servo enable
    HAL_GPIO_WritePin(EN_PI_GPIO_Port, EN_PI_Pin, GPIO_PIN_SET); // Pi enable


    //start PWM for all control surfaces
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Throttle
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Elevator
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Rudder
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Aileron Left
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // Aileron Right
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Flap left
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Flap right
    
    // start DMA listening from Pi
    Start_RC_Listening(&huart3);

    //random variables
    uint8_t memoryBuffer[64];

    // Buffer variables for PWM
    int16_t throBuff = 0;
    int16_t elevBuff = 0;
    int16_t ruddBuff = 0;
    int16_t aileBuff = 0;
    int16_t flapBuff = 0;

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1){
        /**** Translate servos to PWM values ****/
        /* Get RC data */
        Process_RC_Data();

        /* Modulate percentages to timers */
        // Throttle       0 to 100 mapped to 1000 to 2000
        throBuff = latestRCValues.throttle * 10 + 1000;
        // Elevator       PITCH UP 2050,    PITCH DOWN 950
        elevBuff = latestRCValues.pitch * 5.5 + 1500;
        // Rudder         LEFT 1800,        RIGHT 950
        ruddBuff = latestRCValues.yaw * 5.5 + 1500;
        // Aileron Right  UP 2050,          DOWN  950
        aileBuff = latestRCValues.roll * 5.5 + 1500;
        // Flaps          UP 0,             DOWN 2000
        flapBuff = latestRCValues.flaps * 10 + 1000;

        /* Actuate servos */
        // Throttle
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, throBuff);
        // Elevator
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, elevBuff);
        // Rudder
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ruddBuff);
        // Ailerons
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, aileBuff);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, aileBuff);
        // Flaps
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, flapBuff);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, flapBuff);    

        /* Sensor Readings */
        data.timestamp = HAL_GetTick();

        if(AltimeterMs5607Spi::State::COMPLETE == altimeter.Read(AltimeterMs5607Spi::Rate::OSR4096)){
            alt_data = altimeter.GetData();
            data.altitude = alt_data.altitude;
            data.temperature = alt_data.temperature;
        }

        mag_data = magnetometer.Read();
        data.magneticFieldX = mag_data.magneticFieldX;
        data.magneticFieldY = mag_data.magneticFieldY;
        data.magneticFieldZ = mag_data.magneticFieldZ;

        /* Transmit data packet*/
        memcpy(memoryBuffer, &data, sizeof(memoryBuffer));
        HAL_UART_Transmit(&huart1, memoryBuffer, sizeof(memoryBuffer), 100);
    }
}