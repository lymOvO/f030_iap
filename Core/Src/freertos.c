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
#include "uart_device.h"
#include "modbus.h"
#include "errno.h"
#include "control.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define USE_SWITCH_SENSOR 1
//#define USE_ENV_MONITOR_SENSOR 1
#define USE_TMP_HUMI_SENSOR 1



#ifdef USE_SWITCH_SENSOR

#define SLAVE_ADDR 1
#define NB_BITS             5
#define NB_INPUT_BITS       3 
#define NB_REGISTERS        1
#define NB_INPUT_REGISTERS  0  

#endif

#ifdef USE_ENV_MONITOR_SENSOR
extern ADC_HandleTypeDef hadc;

#define SLAVE_ADDR 2
#define NB_BITS             5
#define NB_INPUT_BITS       0 
#define NB_REGISTERS        1
#define NB_INPUT_REGISTERS  2  

#endif

#ifdef USE_TMP_HUMI_SENSOR

void AHT20Task(void *argument);
static void aht20_get_datas(uint16_t *temp, uint16_t *humi);

#define SLAVE_ADDR 3
#define NB_BITS             5
#define NB_INPUT_BITS       0 
#define NB_REGISTERS        1
#define NB_INPUT_REGISTERS  2  

#endif

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
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#ifdef USE_TMP_HUMI_SENSOR
  osThreadNew(AHT20Task, NULL, &defaultTask_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */

#ifdef USE_TMP_HUMI_SENSOR

//**********************************************************//
//CRC校验类型�? CRC8
//多项式： X8+X5+X4+1
//Poly:0011 0001 0x31
unsigned char Calc_CRC8(unsigned char *message,unsigned char Num)
{
    unsigned char i;
    unsigned char byte;
    unsigned char crc =0xFF;
    for (byte = 0;byte<Num;byte++)
    {
        crc^=(message[byte]);
        for(i=8;i>0;--i)
        {
            if(crc&0x80)
                crc=(crc<<1)^0x31;
            else
                crc=(crc<<1);
        }
    }
    return crc;
}//

static uint32_t g_temp, g_humi;

static void aht20_get_datas(uint16_t *temp, uint16_t *humi)
{
	*temp = g_temp;
	*humi = g_humi;
}

void AHT20Task(void *argument)
{
	uint8_t cmd[] = { 0xAC, 0x33, 0x00};	
	uint8_t datas[7];	
	uint8_t crc;	
	
	extern I2C_HandleTypeDef hi2c1;
	
	vTaskDelay(10); /* wait for ready */

	while (1)
	{
		if (HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, 0x70, cmd, 3, 100))
		{
			vTaskDelay(100); /* wait for ready */
			if (HAL_OK == HAL_I2C_Master_Receive(&hi2c1, 0x70, datas, 7, 100))
			{
				/* cal crc */
				crc = Calc_CRC8(datas, 6);
				if (crc == datas[6])
				{
					/* ok */
					g_humi = ((uint32_t)datas[1] << 12) | ((uint32_t)datas[2] << 4) | ((uint32_t)datas[3] >> 4);
					g_temp = (((uint32_t)datas[3] & 0x0f) << 16) | ((uint32_t)datas[4] << 8) | ((uint32_t)datas[5]);

					g_humi = g_humi * 100 * 10/ 0x100000; /* 0.1% */
					g_temp = g_temp * 200 * 10/ 0x100000 - 500; /* 0.1C */
				}
			}
		}

		vTaskDelay(20);		
	}
}

#endif

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* 无限循环 */
    GPIO_PinState val;
	uint8_t *query;
	modbus_t *ctx;
	int rc;
	modbus_mapping_t *mb_mapping;
    int err;

#ifdef USE_ENV_MONITOR_SENSOR
    // 启动ADC校准
    HAL_ADCEx_Calibration_Start(&hadc);
#endif

    // 创建Modbus RTU上下文，使用UART1通信，波特率115200，无校验，8数据位，1停止位
	ctx = modbus_new_st_rtu("uart1", 115200, 'N', 8, 1);
	// 设置Modbus从机地址
	modbus_set_slave(ctx, SLAVE_ADDR);
	// 为Modbus查询分配内存
	query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);

	// 创建Modbus寄存器映射，初始化相关寄存器
	mb_mapping = modbus_mapping_new_start_address(0,
												  NB_BITS,
												  0,
												  NB_INPUT_BITS,
												  0,
												  NB_REGISTERS,
												  0,
												  NB_INPUT_REGISTERS);
	
	// 建立Modbus连接
	rc = modbus_connect(ctx);
	if (rc == -1) {
		// 如果连接失败，释放内存并删除任务
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

    // 初始化寄存器值
    mb_mapping->tab_registers[0] = MODBUS_PRIVATE_STATUS_IDLE;

	for (;;) {
		do {
			// 接收Modbus请求
			rc = modbus_receive(ctx, query);
			/* 过滤掉返回值为0的请求 */
		} while (rc == 0);
 
		/* 如果收到的请求是无效的（例如CRC错误），则不关闭连接 */
		if (rc == -1 && errno != EMBBADCRC) {
			/* 如果出现错误，继续 */
			continue;
		}

        /* 开关量模块3个key */
        /* a. 读取GPIO状态 */
        /* b. 更新寄存器 */
#ifdef USE_SWITCH_SENSOR
        // 读取按键1的状态并更新寄存器
        val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
        if (val == GPIO_PIN_RESET)
        {
            mb_mapping->tab_input_bits[0] = 1;
        }
        else
        {
            mb_mapping->tab_input_bits[0] = 0;
        }

        // 读取按键2的状态并更新寄存器
        val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
        if (val == GPIO_PIN_RESET)
        {
            mb_mapping->tab_input_bits[1] = 1;
        }
        else
        {
            mb_mapping->tab_input_bits[1] = 0;
        }

        // 读取按键3的状态并更新寄存器
        val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
        if (val == GPIO_PIN_RESET)
        {
            mb_mapping->tab_input_bits[2] = 1;
        }
        else
        {
            mb_mapping->tab_input_bits[2] = 0;
        }
#endif

        /* 环境监测模块2个电压 */
        /* a. 读取ADC值（光敏电阻电压值、可调电阻电压值） */
        /* b. 更新寄存器 */
#ifdef USE_ENV_MONITOR_SENSOR
        /* 读取ADC数据并更新输入寄存器 */
        for (int i = 0; i < 2; i++)
        {
            HAL_ADC_Start(&hadc); // 启动ADC转换
             if (HAL_OK == HAL_ADC_PollForConversion(&hadc, 100)) // 等待ADC转换完成
             {
                mb_mapping->tab_input_registers[i] = HAL_ADC_GetValue(&hadc); // 获取ADC值并存储
             }
        }
#endif

        /* 温湿度模块2个传感器 */
        /* a. 获得温度和湿度数据 */
        /* b. 更新寄存器 */
#ifdef USE_TMP_HUMI_SENSOR
		/* 获取温湿度传感器数据 */
		uint16_t temp, humi;
		aht20_get_datas(&temp, &humi);  // 获取温度和湿度数据
		mb_mapping->tab_input_registers[0] = temp;  // 存储温度值到输入寄存器
		mb_mapping->tab_input_registers[1] = humi;  // 存储湿度值到输入寄存器
#endif
#if 1      
        /* 处理紧急命令 */
        err = process_emergency_cmd(ctx, query, rc, mb_mapping);
        if (err)
        {
            modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE); // 回复异常
            continue;
        }
        
        /* 处理文件记录 */
        err = process_file_record(query, rc);
        if (err)
        {
            modbus_reply_exception(ctx, query, MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE); // 回复异常
            continue;
        }
#endif
        
		// 发送Modbus响应
		rc = modbus_reply(ctx, query, rc, mb_mapping);
		if (rc == -1) {
			// 如果发送失败，可以在这里处理
		}

        /* 控制开关量模块2个继电器、3个LED */
        /* a. 读寄存器数据 */
        /* b. 写GPIO */
#ifdef USE_SWITCH_SENSOR
        /* 控制继电器和LED的状态 */
        // 控制switch1
		if (mb_mapping->tab_bits[0])
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); // 打开switch1
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); // 关闭switch1

        // 控制switch2
		if (mb_mapping->tab_bits[1])
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); // 打开switch2
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); // 关闭switch2

		// 控制LED1
		if (mb_mapping->tab_bits[2])
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // 关闭LED1
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // 打开LED1

		// 控制LED2
		if (mb_mapping->tab_bits[3])
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // 关闭LED2
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // 打开LED2

		// 控制LED3
		if (mb_mapping->tab_bits[4])
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // 关闭LED3
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // 打开LED3
#endif

        /* 控制环境监测模块2个蜂鸣器、3个LED */
        /* a. 读寄存器数据 */
        /* b. 写GPIO */
#ifdef USE_ENV_MONITOR_SENSOR
            /* 控制蜂鸣器和LED的状态 */
            // 控制beep1
            if (mb_mapping->tab_bits[0])
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // 打开beep1
            else
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // 关闭beep1
    
            // 控制beep2
            if (mb_mapping->tab_bits[1])
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // 打开beep2
            else
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // 关闭beep2
    
            // 控制LED1
            if (mb_mapping->tab_bits[2])
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); // 关闭LED1
            else
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); // 打开LED1
    
            // 控制LED2
            if (mb_mapping->tab_bits[3])
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // 关闭LED2
            else
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // 打开LED2
    
            // 控制LED3
            if (mb_mapping->tab_bits[4])
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // 关闭LED3
            else
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // 打开LED3
#endif
        /* 控制温湿度模块2个蜂鸣器、3个LED */
        /* a. 读寄存器数据 */
        /* b. 写GPIO */
#ifdef USE_TMP_HUMI_SENSOR
		/* beep1 */
		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //beep1
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //beep1

		/* beep2 */
		if (mb_mapping->tab_bits[1])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); //beep2
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); //beep2

		if (mb_mapping->tab_bits[2])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); //LED1
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); //LED1

		if (mb_mapping->tab_bits[3])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //LED2
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //LED2

		if (mb_mapping->tab_bits[4])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); //LED3
		else
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); //LED3

#endif

	}

	modbus_mapping_free(mb_mapping);
	vPortFree(query);
	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
    
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

