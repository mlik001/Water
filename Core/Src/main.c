/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "lwip.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "relay.h"          // <--- 添加继电器头文件
#include "buzzer.h"         // <--- 蜂鸣器（温度报警）
#include "bsp_delay.h"
#include "flowmeter.h"
#include "pressure_sensor.h"
#include "ds18b20_dual.h"
#include "app_host.h"
#include "app_tcp.h"      // <--- TCP 上位机通讯（JSON 协议）
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

/* USER CODE BEGIN PV */
uint8_t ETH_MAC_ADDR[6] = {0x00, 0x80, 0xE1, 0x00, 0x00, 0x00};
extern struct netif gnetif;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
extern ETH_HandleTypeDef heth;
void MX_ETH_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
 printf("\r\n===== BOOT =====\r\n");
  printf("PD3 reset...\r\n");

  // 标准复位时序
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(2000);  // 筿 PHY 自协商，比之前长丿炿

  printf("PD3 released, calling MX_ETH_Init...\r\n");

  MX_ETH_Init();

  printf("Calling MX_LWIP_Init...\r\n");
  MX_LWIP_Init();
  HAL_ETH_Start_IT(&heth);   /* 中断模式接收（低层 ethernetif 已 Start_IT，此处保持一致） */
  // 打印 MAC / 链路状态（DHCP 模式下 IP 此时尚未分配，先显示 0.0.0.0）
  struct netif *n = &gnetif;
  printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
         ETH_MAC_ADDR[0], ETH_MAC_ADDR[1], ETH_MAC_ADDR[2],
         ETH_MAC_ADDR[3], ETH_MAC_ADDR[4], ETH_MAC_ADDR[5]);
  printf("netif up=%d, link=%d\r\n", netif_is_up(n), netif_is_link_up(n));
  printf("===== DHCP started, waiting for router to assign IP =====\r\n");

  // <--- 新增：初始化继电器（默认全部断开）
  Relay_Init();
  printf("Relay initialized (all OFF).\r\n");

  // <--- 新增：初始化蜂鸣器（温度报警用，默认不响）
  Buzzer_Init();
	
	
  // <--- 新增：初始化各传感器（DWT 延时必须最先，供 1-Wire / 软 I2C 用）
  BSP_Delay_Init();
  FlowMeter_Init();
  PressureSensor_Init();
	PressureSensor_CalibrateZero();
  DS18B20_Dual_Init();
  printf("Sensors initialized.\r\n");

  // <--- 新增：启动 TIM6 定时采集 + UART4 接收（上位机模拟）
  AppHost_Init();

  // <--- 新增：启动 TCP 客户端（主动连接后端服务器，JSON 协议）
  AppTcp_Init();
  printf("TCP client connecting to 192.168.1.1:8088 ...\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MX_LWIP_Process();
	   static int arp_sent = 0;
    if (netif_default && (netif_default->flags & NETIF_FLAG_LINK_UP) && !arp_sent) {
        ip4_addr_t dst;
        IP4_ADDR(&dst, 192, 168, 1, 11);
        arp_sent = 1;
        etharp_request(netif_default, &dst);
    }
	    static uint32_t tick = 0;
    if (HAL_GetTick() - tick > 2000)
    {
        tick = HAL_GetTick();

          uint32_t bsr = 0;
        HAL_ETH_ReadPHYRegister(&heth, 0, PHY_BSR, &bsr);
        printf("alive... up=%d link=%d ip=%s phy_bsr=0x%08lX\r\n",
               netif_is_up(&gnetif),
               netif_is_link_up(&gnetif),
               ip4addr_ntoa(netif_ip4_addr(&gnetif)),
               bsr);
    }

    // <--- 温度状态机：非阻塞推进（DS18B20 转换 750ms 期间主循环照常跑）
    DS18B20_Dual_Process();

    // <--- 上位机模拟：串口命令处理（调试/回退通道）
    AppHost_Process();

    // <--- TCP 客户端：断线重连驱动（主循环调用）
    AppTcp_Process();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#include <stdio.h>

extern UART_HandleTypeDef huart4;

int fputc(int ch, FILE *stream)
{
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

void MX_ETH_Init(void)
{
    heth.Instance = ETH;
    heth.Init.MACAddr = ETH_MAC_ADDR;
    heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

    printf(">>> HAL_ETH_Init start...\r\n");

    if (HAL_ETH_Init(&heth) != HAL_OK)
    {
        printf(">>> HAL_ETH_Init FAILED!\r\n");
        Error_Handler();
    }

    printf(">>> HAL_ETH_Init OK!\r\n");
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