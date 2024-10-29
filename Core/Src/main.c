/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>
#include <fun5mini_interfaces/srv/eat_call.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCSOFTCHECK(fn) if (fn != RCL_RET_OK) {};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
rcl_init_options_t init_options;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;

/* Service Client*/
rcl_client_t eat_client;

/* pub */
rcl_publisher_t cmdvel_publisher;

/* Massage */
geometry_msgs__msg__Twist twist_msg;

fun5mini_interfaces__srv__EatCall_Request eat_request;

/* Start Joy Parameter */
uint16_t ADC_RawRead[200] = { 0 };
uint16_t x_axis = 0;
uint16_t y_axis = 0;

float linear_velocity = 0.0f;
float angular_velocity = 0.0f;

/* button */
GPIO_PinState bButtonState = GPIO_PIN_RESET;
GPIO_PinState bPrevButton = GPIO_PIN_RESET;

/* End Joy Parameter */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* Start MicroRos Function*/
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);
/* End MicroRos Function*/

/* Start my function */

void ReadADC_AVERAGE();
void SentCMDVEL();

/* End my function*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
	if (timer != NULL) {
		/* Code here*/
		ReadADC_AVERAGE();
		SentCMDVEL();

		HAL_IWDG_Refresh(&hiwdg);
	}
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &hlpuart1,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app
  allocator = rcl_get_default_allocator();

  //create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCSOFTCHECK(rcl_init_options_init(&init_options, allocator));
  RCSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 26)); //Set Domain ID

  //create init_options
  rclc_support_init_with_options(
  			&support, 0,
  			NULL,
  			&init_options,
  			&allocator
  	);

  // create node
  	rclc_node_init_default(
  			&node,
  			"cubemx_node",
  			"ling",
  			&support
  	); //Node name

  	// create cmd_vel publisher
  	rclc_publisher_init_default(
  			&cmdvel_publisher,
  			&node,
  			ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  			"cmd_vel"
  		);

  	rclc_client_init_default(
  			&eat_client,
  			&node,
  			ROSIDL_GET_SRV_TYPE_SUPPORT(fun5mini_interfaces, srv, EatCall),
  			"call_eat"
  		);

  	// create Timer
  	rclc_timer_init_default(
  			&timer, &support,
  			RCL_MS_TO_NS(10),
  			timer_callback
  		);

  	// create executer
  	executor = rclc_executor_get_zero_initialized_executor();
  	rclc_executor_init(&executor, &support.context, 1, &allocator);
  	rclc_executor_add_timer(&executor, &timer);
  	rclc_executor_spin(&executor); //ต้องเรียกก่อนถึงจะเริ่มทำงาน

  for(;;)
  {
    osDelay(10);
  }
  /* USER CODE END 5 */
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
  MX_LPUART1_UART_Init();
  MX_ADC1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, ADC_RawRead, 200);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ReadADC_AVERAGE() {
	uint32_t temp_1 = 0;
	uint32_t temp_2 = 0;
	for (int i = 0; i < 200; i++) {
		if (i % 2 == 0) {
			temp_1 += ADC_RawRead[i];
		} else if (i % 2 == 1) {
			temp_2 += ADC_RawRead[i];
		}
	}
	x_axis = (temp_1 / 100);
	y_axis = (temp_2 / 100);
}


void SentCMDVEL(){
	/* Call Velocity */
	linear_velocity =  (y_axis - 2048) / 2048.0f;  // Normalize -1.0 to 1.0
	angular_velocity = -1.0f * (x_axis - 2048) / 2048.0f; // Normalize -1.0 to 1.0

	/* Check DEADZONE*/
	if (fabs(linear_velocity) < 0.015f) {
		linear_velocity = 0.0f;
	}

	if (fabs(angular_velocity) < 0.015f) {
		angular_velocity = 0.0f;
	}

	twist_msg.linear.x = linear_velocity;
	twist_msg.angular.z = angular_velocity;

	RCSOFTCHECK(rcl_publish(&cmdvel_publisher, &twist_msg, NULL));
}

void CheckButtonB() {
    bButtonState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

    if (bButtonState == GPIO_PIN_RESET && bPrevButton == GPIO_PIN_SET) {

    	fun5mini_interfaces__srv__EatCall_Request__init(&eat_request);

    	eat_request.call = true;

    	int64_t sequence_number;
        RCSOFTCHECK(rcl_send_request(&eat_client, &eat_request, &sequence_number));
    }
    HAL_Delay(10);

    bPrevButton = bButtonState;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
