/* USER CODE BEGIN Header */
/**
 * @file freertos.c
 * @author BusyBox (busybox177634@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-12-26
 *
 * @copyright Copyright (c) 2024
 *
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include "math.h"
#include "string.h"
#include "stdio.h"
#include "led/led.h"
#include "atk_md0350/atk_md0350.h"
#include "ads1292/ads1292.h"
#include "iir_filter/iir_filter.h"

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for drawTask */
osThreadId_t drawTaskHandle;
const osThreadAttr_t drawTask_attributes = {
  .name = "drawTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for adsAvaiable */
osSemaphoreId_t adsAvaiableHandle;
const osSemaphoreAttr_t adsAvaiable_attributes = {
  .name = "adsAvaiable"
};
/* Definitions for drawValueAvailable */
osSemaphoreId_t drawValueAvailableHandle;
const osSemaphoreAttr_t drawValueAvailable_attributes = {
  .name = "drawValueAvailable"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

volatile double draw_value = 0;

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == GPIO_PIN_4) {
        osSemaphoreRelease(adsAvaiableHandle);

        static int a = 0;
        a++;
        if (a % 40 == 0)
            led_ds1_toggle();
    }
}


void lcd_draw_heartbeat(double value)
{
    value = - value; /* 由于LCD绘制方向，需要翻转一下图像 */

    static const int x1 = 5, y1 = 10, x2 = 432, y2 = 150; /* 绘制范围 */
    static int x = x1;
    static const double scale = 0.4;
    static int last_draw_x = x1, last_draw_y = (y1 + y2) / 2;
    static double min_value = 1.0, max_value = -1.0;
    static double c_min_value = 1.0, c_max_value = -1.0;

    if (x > (x2 / scale)) {
        x = x1;
        atk_md0350_fill(last_draw_x, y1, last_draw_x + 1, y2, ATK_MD0350_WHITE);
        min_value = c_min_value, max_value = c_max_value;
        c_min_value = 1.0; c_max_value = -1.0;

        /* 绘制纵坐标 */
        char msg_max_value[64];
        char msg_min_value[64];
        snprintf(msg_max_value, sizeof(msg_max_value), " %.2lfmV", max_value);
        snprintf(msg_min_value, sizeof(msg_min_value), "%.2lfmV", min_value);
        atk_md0350_fill(435, 10, 432 + 50, 10 + 12, ATK_MD0350_WHITE);
        atk_md0350_show_string(435, 10, 80, 40, msg_max_value, ATK_MD0350_LCD_FONT_12, ATK_MD0350_BLACK);
        atk_md0350_fill(435, 138, 432 + 50, 138 + 12, ATK_MD0350_WHITE);
        atk_md0350_show_string(435, 138, 50, 12, msg_min_value, ATK_MD0350_LCD_FONT_12, ATK_MD0350_BLACK);
    }

    /* 寻找幅值 */
    if (value > c_max_value) c_max_value = value;
    if (value < c_min_value) c_min_value = value;

    int draw_x = scale * x++;
    int draw_y = y1 + 3 + (value - min_value) * (y2 - y1) / (max_value - min_value) * 0.95;

    /* 限制绘制幅度 */
    if (draw_x > (x2 / scale)) draw_x = x2;
    if (draw_x < x1) draw_x = x1;
    if (draw_y > y2) draw_y = y2;
    if (draw_y < y1) draw_y = y1;

    /* 绘制之前清除上一次的图形 */
    atk_md0350_fill(last_draw_x + 1, y1, draw_x + 1, y2, ATK_MD0350_WHITE);
    if (draw_x > last_draw_x || (draw_x == last_draw_x && draw_x != x1)) atk_md0350_draw_line(last_draw_x, last_draw_y, draw_x, draw_y, ATK_MD0350_RED);

    last_draw_x = draw_x;
    last_draw_y = draw_y;
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void DrawTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the semaphores(s) */
  /* creation of adsAvaiable */
  adsAvaiableHandle = osSemaphoreNew(1, 1, &adsAvaiable_attributes);

  /* creation of drawValueAvailable */
  drawValueAvailableHandle = osSemaphoreNew(1, 1, &drawValueAvailable_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of drawTask */
  drawTaskHandle = osThreadNew(DrawTask, NULL, &drawTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */

//    ads1292_use_test_signal();

    struct IIR_Handle notch_50hz;
    struct IIR_Handle notch_100hz;
    struct IIR_Handle low_pass;
    struct IIR_Handle high_pass;

    iir_filter_init(&notch_50hz, 1.0, -1.52226142, 0.88161859, 0.9408093, -1.52226142, 0.9408093);
    iir_filter_init(&notch_100hz, 1.0, -0.54871515, 0.77567951, 0.88783976, -0.54871515, 0.88783976);
    iir_filter_init(&low_pass, 1.0, -1.64745998, 0.70089678, 0.0133592, 0.0267184, 0.0133592);
    iir_filter_init(&high_pass, 1.0, -1.91119707, 0.91497583, 0.95654323, -1.91308645, 0.95654323);


    while (1) {
        if (osSemaphoreAcquire(adsAvaiableHandle, pdMS_TO_TICKS(portMAX_DELAY)) == osOK) {
            led_ds0_toggle();

            double value = ads1292_read_channel2();

            value = iir_filter_process(&notch_50hz, value);
            value = iir_filter_process(&notch_100hz, value);
            value = iir_filter_process(&low_pass, value);
            value = iir_filter_process(&high_pass, value);

            if (fabs(value) < 200) value *= 0.3;

            value = -value * 7;

            char msg[64];
            snprintf(msg, sizeof(msg), "id:%.8lf\r\n", value);
            HAL_UART_Transmit(&huart1, msg, strlen(msg), 100);


            draw_value = value;
            osSemaphoreRelease(drawValueAvailableHandle);
        }
    }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_DrawTask */
/**
* @brief Function implementing the drawTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DrawTask */
void DrawTask(void *argument)
{
  /* USER CODE BEGIN DrawTask */
  /* Infinite loop */
  for(;;)
  {
      if (osSemaphoreAcquire(drawValueAvailableHandle, pdMS_TO_TICKS(portMAX_DELAY)) == osOK) {
          lcd_draw_heartbeat(draw_value);
      }
  }
  /* USER CODE END DrawTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

