/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "pid.h"
#include "BSP_Can.h"
#include "BSP_Usart.h"

#include "Cloud_Control.h"
#include "Steer_Omni_Chassis.h"
#include "Protocol_Judgement.h"
#include "Protocol_UpperComputer.h"

#include "DT7.h"
#include "Task_LED.h"
#include "BSP_Test.h"
#include "Saber_C3.h"
#include "MA600_use.h"
#include "N100.h"
//#include "UI.h"
#include "PowerControl.h"
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

/***********Queues************/
QueueHandle_t CAN1_ReceiveHandle;
QueueHandle_t CAN2_ReceiveHandle;
QueueHandle_t CAN_SendHandle;
QueueHandle_t Communicate_ReceivefromPCHandle;
/***********Tasks*************/
osThreadId Task_Can1MsgRecHandle;
osThreadId Task_Can2MsgRecHandle;
osThreadId Task_CanSendHandle;
osThreadId Move_DataHandle;
osThreadId led_RGB_flow_handle;
osThreadId Robot_Control_Handle;
//osThreadId Robot_UI_Handle;
osThreadId Task_OffLineCheck_Handle;
osThreadId Task_VofaAssistHandle;
osThreadId Task_CommunicateFromPC_Handle;
osThreadId Task_CommunicateToPC_Handle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId StartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void Can1Receives(void const *argument);
extern void Can2Receives(void const *argument);
extern void AllCanSend(void const *argument);
extern void Robot_Control(void const *argument);
extern void USBCommunicateTask_Receive(void const *argument);
extern void USBCommunicateTask_Send(void const *argument);
extern void Show_Data(void const *argument);
//extern void Robot_UI(void const *argument);
extern void Off_Line_Check(void const *argument);
extern void Vofa_Assist(void const *argument);


/* USER CODE END FunctionPrototypes */
void StartDefaultTask(void const * argument);
void ALL_Init(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END Init */
/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE BEGIN FD */
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
	 /* definition and creation of CAN1_Receive */
    CAN1_ReceiveHandle = xQueueCreate(32, sizeof(Can_Export_Data_t));

    /* definition and creation of CAN2_Receive */
    CAN2_ReceiveHandle = xQueueCreate(64, sizeof(Can_Export_Data_t));

    /* definition and creation of CAN_Send */
    CAN_SendHandle = xQueueCreate(32, sizeof(Can_Send_Data_t));
		
		/* definition and creation of Communicate_PC */
    Communicate_ReceivefromPCHandle = xQueueCreate(16, UpperCom_MAX_BUF);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of StartTask */
  osThreadDef(StartTask, ALL_Init, osPriorityRealtime, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Can1_ReceiveTask, Can1Receives, osPriorityRealtime, 0, 128);
  Task_Can1MsgRecHandle = osThreadCreate(osThread(Can1_ReceiveTask), NULL);

  /* definition and creation of Can1ReceiveTask */
  osThreadDef(Can2_ReceiveTask, Can2Receives, osPriorityRealtime, 0, 256);
  Task_Can2MsgRecHandle = osThreadCreate(osThread(Can2_ReceiveTask), NULL);
	
  /* definition and creation of CanSendTask */
  osThreadDef(Can_SendTask, AllCanSend, osPriorityRealtime, 0, 256);
  Task_CanSendHandle = osThreadCreate(osThread(Can_SendTask), NULL);
	
	/* definition and creation of LED_RGB_Flow_Task */
  osThreadDef(led, led_RGB_flow_task, osPriorityAboveNormal, 0, 128);
  led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
	
	/* definition and creation of Robot_Control_Task */
  osThreadDef(Robot_Control_Task, Robot_Control, osPriorityRealtime, 0, 256);
  Robot_Control_Handle = osThreadCreate(osThread(Robot_Control_Task), NULL);
		

	/* definition and creation of Task_CommunicateToPC_Handle */
  osThreadDef(Task_CommunicateToPC_Handle,USBCommunicateTask_Send, osPriorityAboveNormal, 0, 128);
  Task_CommunicateToPC_Handle = osThreadCreate(osThread(Task_CommunicateToPC_Handle), NULL);
  
	/* definition and creation of Task_CommunicateFromPC_Handle */
  osThreadDef(Task_CommunicateFromPC_Handle,USBCommunicateTask_Receive, osPriorityRealtime, 0, 128);
  Task_CommunicateFromPC_Handle = osThreadCreate(osThread(Task_CommunicateFromPC_Handle), NULL);
		
	
//	/* definition and creation of Robot_UI_Task */
//    osThreadDef(Robot_UI_Task, Robot_UI, osPriorityHigh, 0, 128);
//    Robot_UI_Handle = osThreadCreate(osThread(Robot_UI_Task), NULL);		
//		
	/* definition and creation of Task_OffLineCheck_Handle */
    osThreadDef(Task_OffLineCheck_Handle, Off_Line_Check, osPriorityHigh, 0, 128);
    Task_OffLineCheck_Handle = osThreadCreate(osThread(Task_OffLineCheck_Handle), NULL);
		
	/* definition and creation of vofa_Task */
	  osThreadDef(VofaAssistTask, Vofa_Assist, osPriorityRealtime, 0, 128);
    Task_VofaAssistHandle = osThreadCreate(osThread(VofaAssistTask), NULL);
		


  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  // MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ALL_Init */
/**
  * @brief  Function implementing the StartTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ALL_Init */
void ALL_Init(void const * argument)
{
  /* USER CODE BEGIN ALL_Init */
  /* Infinite loop */
	for(;;)
	{
    MX_USB_DEVICE_Init();
		taskENTER_CRITICAL();                 //�����ٽ���
	    /* CAN�жϳ�ʼ�� */
        Can_Fun.CAN_IT_Init(&hcan1, Can1_Type);
        Can_Fun.CAN_IT_Init(&hcan2, Can2_Type);
		  /*���̳�ʼ��*/
		   Chassis_Init();
		  /*ң������ʼ��*/
		   DT7_Init();
	    /* PID��ʼ�� */
		    /**Yaw轴电机PID初始化**/
//        Position_PIDInit(&J6006s_YawIPID, 0.02f, 0.001f, 1.3f, 0, 10, 4 , 2);
//        Position_PIDInit(&J6006s_YawOPID, 0.02f, 0.000001f, 0.001f, 0, 20, 7 , 7);
		 Position_PIDInit(&J6006s_YawIPID, 0.13f, 0.001f, 1.9f, 0, 10, 4 , 2);
        Position_PIDInit(&J6006s_YawOPID, 0.06f, 0.000001f, 0.004f, 0, 20, 7 , 7);
		    /**AimYaw电机PID初始化*/
        Position_PIDInit(&AutoAim_J6006s_YawIPID, 0.13f, 0.04f, 0.53f, 0.03f, 10, 4 ,2);
        Position_PIDInit(&AutoAim_J6006s_YawOPID, 0.01f, 0.000001f, 0.001f, 0, 20, 7 , 7);
		/*�豸��ʼ��*/
        Cloud_Init();						  //云台初始化
		
		/* ��������ϵͳ */
		JudgeSystem_USART_Receive_DMA(&huart6);
		Saber_Init();							//Saber IMU初始化
									
//		UI_FUN.UI_init();
		vTaskDelete(StartTaskHandle);         //删除启动任务
    taskEXIT_CRITICAL();                  //退出临界区	
	}
  /* USER CODE END ALL_Init */
}


/* USER CODE END FD */
/* USER CODE END Application */
///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * File Name          : freertos.c
//  * Description        : Code for freertos applications
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2023 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */

///* Includes ------------------------------------------------------------------*/
//#include "FreeRTOS.h"
//#include "task.h"
//#include "main.h"
//#include "cmsis_os.h"

///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "pid.h"
//#include "BSP_Can.h"
//#include "BSP_Usart.h"

//#include "Cloud_Control.h"
//#include "Steer_Omni_Chassis.h"
//#include "Protocol_Judgement.h"

//#include "DT7.h"
//#include "Task_LED.h"
//#include "BSP_Test.h"
//#include "Saber_C3.h"
//#include "MA600_use.h"
//#include "N100.h"
//#include "UI.h"
//#include "PowerControl.h"
///* USER CODE END Includes */
//#define Uppercom_MAX_BUF 64

///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */

///* USER CODE END PTD */

///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */

///* USER CODE END PD */

///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */

///* USER CODE END PM */

///* Private variables ---------------------------------------------------------*/
///* USER CODE BEGIN Variables */

///***********Queues************/
//QueueHandle_t CAN1_ReceiveHandle;
//QueueHandle_t CAN2_ReceiveHandle;
//QueueHandle_t CAN_SendHandle;
//QueueHandle_t Communicate_ReceivefromPCHandle;
///***********Tasks*************/
//osThreadId Task_Can1MsgRecHandle;
//osThreadId Task_Can2MsgRecHandle;
//osThreadId Task_CanSendHandle;
//osThreadId Move_DataHandle;
//osThreadId led_RGB_flow_handle;
//osThreadId Robot_Control_Handle;
//osThreadId Robot_UI_Handle;
//osThreadId Task_OffLineCheck_Handle;
//osThreadId Task_VofaAssistHandle;
//osThreadId Task_CommunicateFromPC_Handle;
//osThreadId Task_CommunicateToPC_Handle;

///* USER CODE END Variables */
//osThreadId StartTaskHandle;

///* Private function prototypes -----------------------------------------------*/
///* USER CODE BEGIN FunctionPrototypes */
//extern void Can1Receives(void const *argument);
//extern void Can2Receives(void const *argument);
//extern void AllCanSend(void const *argument);
//extern void Robot_Control(void const *argument);
//extern void Show_Data(void const *argument);
//extern void Robot_UI(void const *argument);
//extern void Off_Line_Check(void const *argument);
//extern void Vofa_Assist(void const *argument);

//// 只保留函数声明
//void USBCommunicateTask_Send(void const *argument);
//void USBCommunicateTask_Receive(void const *argument);
//void VOFA_Handle(void const *argument);
//void J4310_onlineCheck(void const *argument);

///* USER CODE END FunctionPrototypes */

//void ALL_Init(void const * argument);

//extern void MX_USB_DEVICE_Init(void);
//void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

///* GetIdleTaskMemory prototype (linked to static allocation support) */
//void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

///* GetTimerTaskMemory prototype (linked to static allocation support) */
//void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

///* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
//static StaticTask_t xIdleTaskTCBBuffer;
//static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

//void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
//{
//  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
//  *ppxIdleTaskStackBuffer = &xIdleStack[0];
//  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
//  /* place for user code */
//}
///* USER CODE END GET_IDLE_TASK_MEMORY */

///* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
//static StaticTask_t xTimerTaskTCBBuffer;
//static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

//void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
//{
//  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
//  *ppxTimerTaskStackBuffer = &xTimerStack[0];
//  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
//  /* place for user code */
//}
///* USER CODE END Init */
///**
//  * @brief  FreeRTOS initialization
//  * @param  None
//  * @retval None
//  */
//void MX_FREERTOS_Init(void) {
//  /* USER CODE BEGIN Init */
///* Private application code --------------------------------------------------*/
///* USER CODE BEGIN Application */
///* USER CODE BEGIN FD */
//  /* USER CODE END Init */

//  /* USER CODE BEGIN RTOS_MUTEX */
//  /* add mutexes, ... */
//  /* USER CODE END RTOS_MUTEX */

//  /* USER CODE BEGIN RTOS_SEMAPHORES */
//  /* add semaphores, ... */
//  /* USER CODE END RTOS_SEMAPHORES */

//  /* USER CODE BEGIN RTOS_TIMERS */
//  /* start timers, add new ones, ... */
//  /* USER CODE END RTOS_TIMERS */

//  /* USER CODE BEGIN RTOS_QUEUES */
//  /* add queues, ... */
//	 /* definition and creation of CAN1_Receive */
//    CAN1_ReceiveHandle = xQueueCreate(32, sizeof(Can_Export_Data_t));

//    /* definition and creation of CAN2_Receive */
//    CAN2_ReceiveHandle = xQueueCreate(64, sizeof(Can_Export_Data_t));

//    /* definition and creation of CAN_Send */
//    CAN_SendHandle = xQueueCreate(32, sizeof(Can_Send_Data_t));
//		
//		/* definition and creation of Communicate_PC */
//    Communicate_ReceivefromPCHandle = xQueueCreate(16, Uppercom_MAX_BUF);
//  /* USER CODE END RTOS_QUEUES */

//  /* Create the thread(s) */
//  /* definition and creation of StartTask */
//  osThreadDef(StartTask, ALL_Init, osPriorityRealtime, 0, 128);
//  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

//  /* USER CODE BEGIN RTOS_THREADS */
//  /* add threads, ... */
//   /* definition and creation of Can1ReceiveTask */
//    osThreadDef(Can1_ReceiveTask, Can1Receives, osPriorityRealtime, 0, 128);
//    Task_Can1MsgRecHandle = osThreadCreate(osThread(Can1_ReceiveTask), NULL);

//    /* definition and creation of Can1ReceiveTask */
//    osThreadDef(Can2_ReceiveTask, Can2Receives, osPriorityRealtime, 4, 256);
//    Task_Can2MsgRecHandle = osThreadCreate(osThread(Can2_ReceiveTask), NULL);
//	
//    /* definition and creation of CanSendTask */
//    osThreadDef(Can_SendTask, AllCanSend, osPriorityRealtime, 0, 256);
//    Task_CanSendHandle = osThreadCreate(osThread(Can_SendTask), NULL);
//	
//	/* definition and creation of LED_RGB_Flow_Task */
//    osThreadDef(led, led_RGB_flow_task, osPriorityAboveNormal, 0, 128);
//    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
//	
//	/* definition and creation of Robot_Control_Task */
//    osThreadDef(Robot_Control_Task, Robot_Control, osPriorityRealtime, 0, 256);
//    Robot_Control_Handle = osThreadCreate(osThread(Robot_Control_Task), NULL);
//		
//	/* definition and creation of Task_CommunicateToPC_Handle */
//    osThreadDef(Task_CommunicateToPC_Handle,USBCommunicateTask_Send, osPriorityAboveNormal, 0, 128);
//    Task_CommunicateToPC_Handle = osThreadCreate(osThread(Task_CommunicateToPC_Handle), NULL);
//	
//	/* definition and creation of Robot_Control_Task */
//    osThreadDef(Robot_UI_Task, Robot_UI, osPriorityHigh, 0, 128);
//    Robot_UI_Handle = osThreadCreate(osThread(Robot_UI_Task), NULL);		
//		
//	/* definition and creation of Task_OffLineCheck_Handle */
//    osThreadDef(Task_OffLineCheck_Handle, Off_Line_Check, osPriorityHigh, 0, 128);
//    Task_OffLineCheck_Handle = osThreadCreate(osThread(Task_OffLineCheck_Handle), NULL);
//		
//	/* definition and creation of vofa_Task */
//	  osThreadDef(VofaAssistTask, Vofa_Assist, osPriorityRealtime, 0, 128);
//    Task_VofaAssistHandle = osThreadCreate(osThread(VofaAssistTask), NULL);
//		


//  /* USER CODE END RTOS_THREADS */

//}

///* USER CODE BEGIN Header_ALL_Init */
///**
//  * @brief  Function implementing the StartTask thread.
//  * @param  argument: Not used
//  * @retval None
//  */
///* USER CODE END Header_ALL_Init */
//void ALL_Init(void const * argument)
//{
//  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
//  /* USER CODE BEGIN ALL_Init */
//  /* Infinite loop */
//	for(;;)
//	{
//		taskENTER_CRITICAL();                 //进入临界区
//	    /* CAN中断初始化 */
//        Can_Fun.CAN_IT_Init(&hcan1, Can1_Type);
//        Can_Fun.CAN_IT_Init(&hcan2, Can2_Type);
//		  /*底盘初始化*/
//		   Chassis_Init();
//		  /*遥控器初始化*/
//		   DT7_Init();
//	    /* PID初始化 */
//		    /**Yaw轴电机PID初始化**/
////        Position_PIDInit(&J6006s_YawIPID, 0.02f, 0.001f, 1.3f, 0, 10, 4 , 2);
////        Position_PIDInit(&J6006s_YawOPID, 0.02f, 0.000001f, 0.001f, 0, 20, 7 , 7);
//		 Position_PIDInit(&J6006s_YawIPID, 0.05f, 0.001f, 1.3f, 0, 10, 4 , 2);
//        Position_PIDInit(&J6006s_YawOPID, 0.03f, 0.000001f, 0.002f, 0, 20, 7 , 7);
//		    /**AimYaw电机PID初始化*/
//        Position_PIDInit(&AutoAim_J6006s_YawIPID, 0.13f, 0.04f, 0.53f, 0.03f, 10, 4 ,2);
//        Position_PIDInit(&AutoAim_J6006s_YawOPID, 0.01f, 0.000001f, 0.001f, 0, 20, 7 , 7);
//		/*设备初始化*/
//        Cloud_Init();						  //云台初始化
//		
//		/* 裁判系统系统 */
//		JudgeSystem_USART_Receive_DMA(&huart6);
//		Saber_Init();							//Saber IMU初始化
//									
//		UI_FUN.UI_init();
//		vTaskDelete(StartTaskHandle);         //删除启动任务
//    taskEXIT_CRITICAL();                  //退出临界区	
//	}
//  /* USER CODE END ALL_Init */
//}

///* USER CODE END FD */
///* USER CODE END Application */
