#include "cpp_main.h"
#include "main.h"
#include <cstring>

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
    
    // turn on Servo and Pi Buck converter
    HAL_GPIO_WritePin(EN_SERVO_GPIO_Port,EN_SERVO_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(EN_PI_GPIO_Port, EN_PI_Pin, GPIO_PIN_SET);


    //start PWM for all control surfaces
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Throttle
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Elevator
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Rudder
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Aileron Left
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // Aileron Right
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // Flap left
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Flap right

    // // Elevator       DOWN 2050,  UP    950
    // // Rudder         LEFT 1800,  RIGHT 950
    // // Aileron Right  UP 2050,    DOWN  950
    // // Aileron Left   UP 950,     DOWN  2050
    // // Flaps          UP 950,     DOWN  2050

    // start DMA listening from Pi
    Start_RC_Listening(&huart3);

    //random variables
    uint8_t memoryBuffer[100];

    // Buffer variables for PWM
    int16_t throBuff = 0;
    int16_t elevBuff = 0;
    int16_t ruddBuff = 0;
    int16_t aileBuff = 0;
    int16_t flapBuff = 0;

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1){
        // Get PI data
        Process_RC_Data();

        /* Translate servos to PWM values */
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
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, elevBuff);
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
    }
}