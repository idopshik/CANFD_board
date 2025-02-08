/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

#define CAN_SPECIAL_ID 0x003
#define APB1_CLK 150000000
#define MinutTeethFactor 1.6
#define numFL 0
#define numFR 1
#define numRL 2
#define numRR 3

FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;

uint8_t canRX[8];  // CAN Bus Receive Buffer
uint8_t freshCanMsg = 0;

char ms100Flag = 0;
char ms100Flag_2 = 0;

typedef struct {
    uint8_t wheel_num;
    TIM_HandleTypeDef *htim;
    int prev_speed;
    int prev_psc;
    int cur_psc;
    int initial_tmp_flag;
    uint8_t psc_change_flag;
} whl_chnl;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void CANFD1_Set_Filtes(void);
void my_printf(const char *fmt, ...);
int calculete_prsc_and_perio(int val, int *arr, whl_chnl *whl_arr[], int wheelnum);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// this is for printf
int _write(int file, char *ptr, int len)
{
    int i = 0;
    for (i = 0; i < len; i++) ITM_SendChar(*ptr++);
    return len;
}

void PrintArrayLen(uint8_t *data_arr, uint8_t data_length)
{
    for (int i = 0; i < data_length; i++) {
        // printf("%lf\n",foo[i]);
        printf("%#x ", data_arr[i]);
    }
    printf("\n");
}

void PrintArray(uint8_t *data_arr)
{
    int debval;

    int denominator = sizeof(data_arr[0]);

    if (denominator > 0) {
        debval = (sizeof(data_arr) / denominator);
        for (int i = 0; i < debval; i++) {
            // printf("%lf\n",foo[i]);
            printf("%#x ", data_arr[i]);
        }
    }
    printf("\n");
}

uint32_t calculete_period_only(int val)
{
    // For 32bit timer

    float factor = 0.02;
    return (APB1_CLK / (val * factor * MinutTeethFactor * (12 + 1))) - 1;  // значение регистра ARR
}

int calculete_prsc_and_perio(int val, int *arr, whl_chnl *whl_arr[], int wheelnum)
{
    // Нужно будет отладить.

    float factor = 0.02;

    int newpresc;

    if (val < 4274) {
        newpresc = 1200;
    }
    else {
        newpresc = 24;
    }

    // Только временно
    if (whl_arr[wheelnum]->initial_tmp_flag == 0) {
        /* arr[0] =  newpresc; */
        /* my_printf("prescaler_only_once for %d wheel\n\r", wheelnum); */
        whl_arr[wheelnum]->initial_tmp_flag = 1;
        arr[0] = 24;  // tmp
    }
    else {

        arr[0] = whl_arr[wheelnum]->prev_psc;
    }
    whl_arr[wheelnum]->prev_psc = newpresc;

    arr[1] = newpresc;

    uint32_t tmp;
    /* int tmp; */
    tmp = (APB1_CLK / (val * factor * MinutTeethFactor * (arr[0] + 1))) - 1;  // значение регистра ARR
    if (tmp > 65536) {
        /* my_printf(" -------------> GLITCH!\n\r"); */
        /* my_printf("res: %d \n\r", tmp); */
        /* my_printf("PSC:%d \n\r", arr[0]); */
        /* my_printf("signal: %d \n\n\r", val); */
        tmp = 65535;  // обрезаем. Хоть это и не правильно1
    }
    arr[2] = (int)tmp;
    if (arr[0] != arr[1]) {
        /* my_printf("             --              \n\r"); */
        /* my_printf("old_arr: %d\n\r", tmp); */

        tmp = ((APB1_CLK / (val * factor * MinutTeethFactor * (newpresc + 1))) - 1);  // значение регистра ARR
                                                                                      //
        /* my_printf("new_arr: %d\n\r", tmp); */

        if (tmp > 65536) {
            /* my_printf(" -------------> GLITCH_2!\n\r"); */
            /* HAL_GPIO_TogglePin(PreLast_GPIO_Port, PreLast_Pin); */
            tmp = 65535;  // обрезаем. Хоть это и не правильно.
        }

        arr[3] = (int)tmp;
        /* arr[3] = 100; */

        /* my_printf("old_arr_casted: %d\n\r", arr[2]); */
        /* my_printf("arr_casted: %d\n\r", arr[3]); */
        /* my_printf("PSC old: %d\n\r", arr[0]); */
        /* my_printf("PSC current: %d\n\n\r", arr[1]); */
        /* my_printf("val: %d\n\n\r", val); */
        /* my_printf("\n"); */

    }
    else {
        arr[3] = arr[2];
    }

    return 0;
}

void set_new_speeds(int vFLrpm, int vFRrpm, int vRLrpm, int vRRrpm, whl_chnl *whl_arr[])
{
    //__HAL_TIM_SET_PRESCALER(&htim3, val );
    //__HAL_TIM_SET_AUTORELOAD(&htim3, val );
    /* permutations! */
    /* 1) changing prescaler up or down  */
    /* 2) the same prescaler */
    /* ----- */
    /* 1) going up (speed up) */
    /* 2) going down. */

    // 32-bit timers first

    ////////////    TIMER2 -  RL  ///////////////
    if (vRLrpm == 0) {
        TIM2->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM2->CR1 |= TIM_CR1_CEN;  // enable
                                   //
        uint32_t current_32bit_period = calculete_period_only(vRLrpm);

        if (vRLrpm > whl_arr[numRL]->prev_speed) {
            TIM2->CR1 |= TIM_CR1_ARPE;
            TIM2->ARR = current_32bit_period;

            if (TIM2->CNT > current_32bit_period) {
                TIM2->EGR |= TIM_EGR_UG;
            }

            HAL_GPIO_TogglePin(LED_RX2_GPIO_Port, LED_RX2_Pin);
        }
        else {
            /* my_printf(" ARR: %d\n\r", arr_with_calculations[0]); */

            HAL_GPIO_TogglePin(LED_TX2_GPIO_Port, LED_TX2_Pin);

            if (TIM2->CNT < current_32bit_period) {
                TIM2->CR1 |= TIM_CR1_ARPE;
                TIM2->ARR = current_32bit_period;
            }
            else {
                // Не должна никогда выполняться.
                TIM2->CR1 &= ~TIM_CR1_ARPE;
                TIM2->ARR = current_32bit_period;
            }
        }

        //Костыльный предохранитель от убегания.
        if (TIM2->CNT > current_32bit_period) {
            TIM2->EGR |= TIM_EGR_UG;
        }
    }
    whl_arr[numRL]->prev_speed = vRLrpm;







    ////////////    TIMER5 -  RR  ///////////////
    if (vRRrpm == 0) {
        TIM5->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM5->CR1 |= TIM_CR1_CEN;  // enable
                                   //
        uint32_t current_32bit_period = calculete_period_only(vRRrpm);

        if (vRRrpm > whl_arr[numRR]->prev_speed) {
            TIM5->CR1 |= TIM_CR1_ARPE;
            TIM5->ARR = current_32bit_period;

            if (TIM5->CNT > current_32bit_period) {
                TIM5->EGR |= TIM_EGR_UG;
            }

            HAL_GPIO_TogglePin(LED_RX2_GPIO_Port, LED_RX2_Pin);
        }
        else {
            /* my_printf(" ARR: %d\n\r", arr_with_calculations[0]); */

            HAL_GPIO_TogglePin(LED_TX2_GPIO_Port, LED_TX2_Pin);

            if (TIM5->CNT < current_32bit_period) {
                TIM5->CR1 |= TIM_CR1_ARPE;
                TIM5->ARR = current_32bit_period;
            }
            else {
                TIM5->CR1 &= ~TIM_CR1_ARPE;
                TIM5->ARR = current_32bit_period;
            }
        }

        //Костыльный предохранитель от убегания.
        if (TIM5->CNT > current_32bit_period) {
            TIM5->EGR |= TIM_EGR_UG;
            HAL_GPIO_TogglePin(LED_RX3_GPIO_Port, LED_RX3_Pin);
        }
    }
    whl_arr[numRR]->prev_speed = vRRrpm;







    int arr_with_calculations[4] = {300, 300, 300, 300};

    ////////////    TIMER3 -  FR  ///////////////

    if (vFRrpm == 0) {
        TIM3->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM3->CR1 |= TIM_CR1_CEN;  // enable

        calculete_prsc_and_perio(vFRrpm, arr_with_calculations, whl_arr, numFR);

        if (vFRrpm != whl_arr[numFR]->prev_speed) {
            if (vFRrpm > whl_arr[numFR]->prev_speed) {


                if (whl_arr[numFR]->psc_change_flag == 1) {
                    //this isn't functional now.
                    whl_arr[numFR]->psc_change_flag = 0;
                    TIM3->CCMR1 |= TIM_CCMR1_OC1M_0;
                    TIM3->CCMR1 |= TIM_CCMR1_OC1M_1;
                }

                if (TIM3->CNT > (arr_with_calculations[2])) {
                    TIM3->PSC = arr_with_calculations[1];
                    TIM3->ARR = arr_with_calculations[3];
                    TIM3->EGR = TIM_EGR_UG;

                }
                /*
                В любом случае рассчитываем точку по старому прескалеру.
                Если точка пройдена (по старому прескалеру) то.

                     - Если есть новое значение пресклера, кладём его в
                     PSC . Мы всё равно ведь будем вызывать  UG ?? оно как
                     раз применится.
                     - Кладём ARR (рассчитанный по новому)
                     - Вызываем UG.
                 */
                else {
                    /*
                     Точка ещё не пройдена. В этом случае нельзя вызвать UG.
                     будет Glitch.
                     - пишем новый ARR  по старому прескалеру (без буфера)
                     - если PSC поменялся, то кладём уже по буферу и PSC и ARR.
                     */
                    TIM3->CR1 |= TIM_CR1_ARPE;
                    TIM3->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {


                        TIM3->CR1 &= ~TIM_CR1_ARPE;
                        TIM3->PSC = arr_with_calculations[1];
                        TIM3->ARR = arr_with_calculations[3];

                        HAL_GPIO_TogglePin(LastPin_GPIO_Port, LastPin_Pin);

                        // только для теста. TODO 
                        TIM3->EGR = TIM_EGR_UG;
                    }

                    /* TIM3->CR1 |= TIM_CR1_ARPE;    // будем писать сразу в ARR (мимо shadow) */
                    /* TIM3->ARR = arr_with_calculations[2]; */
                    /* [> Костыльный предохранитель от убегания. <] */
                    /* if (TIM3->CNT > arr_with_calculations[2]) { */
                        /* TIM3->EGR |= TIM_EGR_UG; */
                    /* } */


                }
            }
            else {
                HAL_GPIO_TogglePin(LED_TX2_GPIO_Port, LED_TX2_Pin);
                /*
                меньше. Скорость уменьшается. В этом случае обычно.период
                увеличивается. Но - прескалер может
                увеличиться (переключиться), и тем самым рассчётное значение ARR
                может быть значительно меньше, чем текущее.
                1.  Если не меняется  PSC - пишем  ARR без буфер. Это не
                вызывает никакх проблем вообще
                2.  �?зменение  PSC в этом случае пишем по старому прескелеру
                без буфера, рассчитываем новую пару и пишем по буферу!
                 */

                TIM3->CR1 |= TIM_CR1_ARPE;
                TIM3->ARR = arr_with_calculations[2];

                HAL_GPIO_TogglePin(PreLast_GPIO_Port, Pre_pre_Pin);

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    
                    /* TIM3->CR1 &= ~TIM_CR1_ARPE; */

                    TIM3->PSC = arr_with_calculations[1];
                    TIM3->ARR = arr_with_calculations[3];

                    HAL_GPIO_TogglePin(PreLast_GPIO_Port, PreLast_Pin);
                }


            }
            whl_arr[numFR]->initial_tmp_flag = arr_with_calculations[1];
            whl_arr[numFR]->prev_speed = vFRrpm;
        }
    }








    ////////////    TIMER4 -  FL  ///////////////


    if (vFLrpm == 0) {
        TIM4->CR1 &= ~((uint16_t)TIM_CR1_CEN);
    }
    else {
        TIM4->CR1 |= TIM_CR1_CEN;  // enable

        calculete_prsc_and_perio(vFLrpm, arr_with_calculations, whl_arr, numFL);

        if (vFLrpm != whl_arr[numFL]->prev_speed) {
            if (vFLrpm > whl_arr[numFL]->prev_speed) {


                if (whl_arr[numFL]->psc_change_flag == 1) {
                    //this isn't functional now.
                    whl_arr[numFL]->psc_change_flag = 0;
                    TIM4->CCMR1 |= TIM_CCMR1_OC1M_0;
                    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1;
                }

                if (TIM4->CNT > (arr_with_calculations[2])) {
                    TIM4->PSC = arr_with_calculations[1];
                    TIM4->ARR = arr_with_calculations[3];
                    TIM4->EGR = TIM_EGR_UG;

                }
                /*
                В любом случае рассчитываем точку по старому прескалеру.
                Если точка пройдена (по старому прескалеру) то.

                     - Если есть новое значение пресклера, кладём его в
                     PSC . Мы всё равно ведь будем вызывать  UG ?? оно как
                     раз применится.
                     - Кладём ARR (рассчитанный по новому)
                     - Вызываем UG.
                 */
                else {
                    /*
                     Точка ещё не пройдена. В этом случае нельзя вызвать UG.
                     будет Glitch.
                     - пишем новый ARR  по старому прескалеру (без буфера)
                     - если PSC поменялся, то кладём уже по буферу и PSC и ARR.
                     */
                    TIM4->CR1 |= TIM_CR1_ARPE;
                    TIM4->ARR = arr_with_calculations[2];
                    if (arr_with_calculations[0] != arr_with_calculations[1]) {


                        TIM4->CR1 &= ~TIM_CR1_ARPE;
                        TIM4->PSC = arr_with_calculations[1];
                        TIM4->ARR = arr_with_calculations[3];

                        HAL_GPIO_TogglePin(LastPin_GPIO_Port, LastPin_Pin);

                        // только для теста. TODO 
                        TIM4->EGR = TIM_EGR_UG;
                    }

                    /* TIM4->CR1 |= TIM_CR1_ARPE;    // будем писать сразу в ARR (мимо shadow) */
                    /* TIM4->ARR = arr_with_calculations[2]; */
                    /* [> Костыльный предохранитель от убегания. <] */
                    /* if (TIM4->CNT > arr_with_calculations[2]) { */
                        /* TIM4->EGR |= TIM_EGR_UG; */
                    /* } */


                }
            }
            else {
                HAL_GPIO_TogglePin(LED_TX2_GPIO_Port, LED_TX2_Pin);
                /*
                меньше. Скорость уменьшается. В этом случае обычно.период
                увеличивается. Но - прескалер может
                увеличиться (переключиться), и тем самым рассчётное значение ARR
                может быть значительно меньше, чем текущее.
                1.  Если не меняется  PSC - пишем  ARR без буфер. Это не
                вызывает никакх проблем вообще
                2.  �?зменение  PSC в этом случае пишем по старому прескелеру
                без буфера, рассчитываем новую пару и пишем по буферу!
                 */

                TIM4->CR1 |= TIM_CR1_ARPE;
                TIM4->ARR = arr_with_calculations[2];

                HAL_GPIO_TogglePin(PreLast_GPIO_Port, Pre_pre_Pin);

                if (arr_with_calculations[0] != arr_with_calculations[1]) {
                    
                    /* TIM4->CR1 &= ~TIM_CR1_ARPE; */

                    TIM4->PSC = arr_with_calculations[1];
                    TIM4->ARR = arr_with_calculations[3];

                    HAL_GPIO_TogglePin(PreLast_GPIO_Port, PreLast_Pin);
                }


            }
            whl_arr[numFL]->initial_tmp_flag = arr_with_calculations[1];
            whl_arr[numFL]->prev_speed = vFLrpm;
        }
    }






}

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if (0 < vsprintf(string, fmt, argp))  // build string
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)string, strlen(string),
                          0xffffff);  // send message via UART
    }
}

void my_printf(const char *fmt, ...)  // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

void print_test(void)
{
    printf("\r\nRTOS\r\nCharacters: %c %c\r\n", 'a', 65);
    printf("Decimals: %d %ld\r\n", 1977, 650000L);
    printf("Preceding with blanks: %10d\r\n", 1977);
    printf("Preceding with zeros: %010d\r\n", 1977);
    printf("Some different radices: %d %x %o %#x %#o\r\n", 100, 100, 100, 100, 100);
    printf("floats: %4.2f %+.0e %E\r\n", 3.1416, 3.1416, 3.1416);
    printf("Width trick: %*d\r\n", 5, 10);
    printf("%s\r\n\r\n", "A string");

    my_printf("\r\nRTOS\r\nCharacters: %c %c\r\n", 'a', 65);
    my_printf("Decimals: %d %ld\r\n", 1977, 650000L);
    my_printf("Preceding with blanks: %10d\r\n", 1977);
    my_printf("Preceding with zeros: %010d\r\n", 1977);
    my_printf("Some different radices: %d %x %o %#x %#o\r\n", 100, 100, 100, 100, 100);
    my_printf("floats: %4.2f %+.0e %E\r\n", 3.1416, 3.1416, 3.1416);
    my_printf("Width trick: %*d\r\n", 5, 10);
    my_printf("%s\r\n\r\n", "A string");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM6) {
        /*my_printf("100 ms \n");*/
        ms100Flag = 1;
        ms100Flag_2 = 1;
    }

    if (htim->Instance == TIM3) {

        HAL_GPIO_TogglePin(PreLast_GPIO_Port, FourB_Pin);

        // почему разыменование?? - требование к аргументу - __HANDLE__: TIM
        //
        // handle
        /* int t_val = __HAL_TIM_GET_COUNTER(&htim2); */
        /* int v_arr = __HAL_TIM_GET_AUTORELOAD(&htim2); */

        /* my_printf("t2:ctr: %d /n", t_val); */
        /* my_printf("t2:arr: %d /n", v_arr); */

        /* if (g_psc_change_flag == 1) { */
            /* g_psc_change_flag = 0; */


            /* [> TIM2 -> CCMR1 |= TIM_CCMR1_OC1M; //set to 1 - toggle again <] */
        /* } */
    }
    else if (htim->Instance == TIM4) {

        HAL_GPIO_TogglePin(PreLast_GPIO_Port, fiveB_Pin);
    }
    else if (htim->Instance == TIM8) {

        /* reset CEN bit of the TIMx_CR1 (disable counter) */
        /* write new prescaler value to TIMx_PSC */
        /* set the UG (Update generation) bit of the TIMx_EGR */
        /* reset UIF bit of TIMx_SR        (clear IRQ flag) */
        /* set CEN bit of the TIMx_CR1 (enable counter) */
    }
    /* else if (htim->Instance == TIM5) { */
    /* } */
}
void calculate_arr(whl_chnl *whl_arr[], int whl)
{
    //
    // примеры обращения к элемента структуры
    //   whl_arr[whl]->pref_speed
    //   whl_arr[whl]->cur_psc
    //   whl_arr[whl]->flag
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

    CANFD1_Set_Filtes();

    // Start four timers
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);

    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_4);
    HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);

    // timer6 (10Hz);
    HAL_TIM_Base_Start_IT(&htim6);
#ifdef DEBUG
    printf("[ INFO ] Program start now\n");
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    int counter = 0;
    int realcou = 0;
    int seconds_from_start = 0;
    printf("async debug is on");

    whl_chnl fl_whl_s = {numFL, &htim2, 0, 12, 0, 0};
    whl_chnl *p_fl_whl;
    p_fl_whl = &fl_whl_s;

    whl_chnl fr_whl_s = {numFR, &htim3, 0, 24, 0, 0};
    whl_chnl *p_fr_whl;
    p_fr_whl = &fr_whl_s;

    whl_chnl rl_whl_s = {numRL, &htim4, 0, 24, 0, 0};
    whl_chnl *p_rl_whl;
    p_rl_whl = &rl_whl_s;

    whl_chnl rr_whl_s = {numRR, &htim5, 0, 12, 0, 0};
    whl_chnl *p_rr_whl;
    p_rr_whl = &rr_whl_s;

    whl_chnl *whl_arr[4];

    whl_arr[numFL] = p_fl_whl;
    whl_arr[numFR] = p_fr_whl;
    whl_arr[numRL] = p_rl_whl;
    whl_arr[numRR] = p_rr_whl;

    /* calculate_arr(whl_arr, 0);    //  делаю функцию, которая  принимае int */

    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


        /* Не забудь потом убрать эту задержку  */

        /* HAL_Delay(0.01); // Insert delay 100 ms */
        /* printf("100ms passed\n"); */

        if (ms100Flag_2 > 0) {
            ms100Flag_2 = 0;
            realcou++;
            if (realcou > 9) {
                realcou = 0;
                /* my_printf(" %d seconds\n\r", seconds_from_start); */
                HAL_GPIO_TogglePin(LED_TX3_GPIO_Port, LED_TX3_Pin);
                seconds_from_start++;
            }
        }

        if (freshCanMsg == 1) {
            freshCanMsg = 0;
            HAL_GPIO_TogglePin(LED_TX1_GPIO_Port, LED_TX1_Pin);

            /*PrintArrayLen(canRX, sizeof(canRX));  // works
             * perfectly.*/
            // PrintArray(canRX);  // But why??

            int vFLrpm, vFRrpm, vRLrpm, vRRrpm;

            vFLrpm = (uint8_t)canRX[0] << 8 | (uint8_t)canRX[1];
            vFRrpm = (uint8_t)canRX[2] << 8 | (uint8_t)canRX[3];
            vRLrpm = (uint8_t)canRX[4] << 8 | (uint8_t)canRX[5];
            vRRrpm = (uint8_t)canRX[6] << 8 | (uint8_t)canRX[7];

            /*my_printf("vFLrpm: %d vFRrpm: %d  vRLrpm: %d  vRRrpm: %d \n",
             * vFLrpm, vFRrpm, vRLrpm, vRRrpm);*/
            set_new_speeds(vFLrpm, vFRrpm, vRLrpm, vRRrpm, whl_arr);

            counter += 1;

            if (ms100Flag > 0) {
                ms100Flag = 0;
                if (counter > 0) {
                    /* my_printf("receiving at %d per second\n\r", counter *
                     * 10); */
                    /* my_printf("vFLrpm: %d vFRrpm: %d  vRLrpm: %d  vRRrpm: %d
                       \n\r", vFLrpm, vFRrpm, vRLrpm, vRRrpm);*/

                    counter = 0;
                }
            }
        }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 7;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 8;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 1;
  hfdcan2.Init.NominalSyncJumpWidth = 2;
  hfdcan2.Init.NominalTimeSeg1 = 13;
  hfdcan2.Init.NominalTimeSeg2 = 2;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 7;
  hfdcan2.Init.DataTimeSeg1 = 7;
  hfdcan2.Init.DataTimeSeg2 = 8;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 12;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 24;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 24;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 12;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 10000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 15000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_TX1_Pin|LED_RX1_Pin|LED_TX2_Pin|LED_RX2_Pin
                          |LED_TX3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RX3_GPIO_Port, LED_RX3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ZeroB_Pin|FourB_Pin|fiveB_Pin|Pre_pre_Pin
                          |PreLast_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LastPin_GPIO_Port, LastPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_3_Pin Button_2_Pin Button_1_Pin */
  GPIO_InitStruct.Pin = Button_3_Pin|Button_2_Pin|Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_TX1_Pin LED_RX1_Pin LED_TX2_Pin LED_RX2_Pin
                           LED_TX3_Pin */
  GPIO_InitStruct.Pin = LED_TX1_Pin|LED_RX1_Pin|LED_TX2_Pin|LED_RX2_Pin
                          |LED_TX3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RX3_Pin */
  GPIO_InitStruct.Pin = LED_RX3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RX3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ZeroB_Pin FourB_Pin fiveB_Pin Pre_pre_Pin
                           PreLast_Pin */
  GPIO_InitStruct.Pin = ZeroB_Pin|FourB_Pin|fiveB_Pin|Pre_pre_Pin
                          |PreLast_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_RX3_Pin */
  GPIO_InitStruct.Pin = CAN_RX3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_FDCAN3;
  HAL_GPIO_Init(CAN_RX3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LastPin_Pin */
  GPIO_InitStruct.Pin = LastPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LastPin_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void CANFD1_Set_Filtes(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = CAN_SPECIAL_ID;
    sFilterConfig.FilterID2 = 0x07FF;
    // sFilterConfig.RxBufferIndex = 0;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        /* Filter configuration Error */
        Error_Handler();
    }
    else {
        my_printf("filterOK\n\r");
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Error_Handler();
    }

    // Activate the notification for new data in FIFO0 for FDCAN1
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }
    // STart FDCAN1
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

// FDCAN1 Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    HAL_GPIO_TogglePin(LED_RX1_GPIO_Port, LED_RX1_Pin);
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        /* Retreive Rx messages from RX FIFO0 */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, canRX) != HAL_OK) {
            /* Reception Error */
            Error_Handler();
        }
        else {
            if ((RxHeader1.Identifier != CAN_SPECIAL_ID)) {
                HAL_GPIO_TogglePin(LED_TX3_GPIO_Port, LED_TX3_Pin);
                my_printf("wrongID: %#x \n\r", RxHeader1.Identifier);
            }
            else {
                freshCanMsg = 1;
            }
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return
     * state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
