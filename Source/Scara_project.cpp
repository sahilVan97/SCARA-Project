/**
  ******************************************************************************
    Main Project
  * Project : SCARA Project  
  * @file   Scara_project.cpp
  * @author Sahil Vanarase
  * 

	FreeRTOS - Licensing
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <kinematics.h> /// Include My library
#include <task.h>      /// Include for task management for FreeRTOS
#include <string.h>
#include "fatfs.h"
#include "fatfs_sd.h"


/* Private typedef -----------------------------------------------------------*/
static bool request_output; 
struct IO_handle {
	int GPIO_BASE;
	int GPIO_PIN;
	GPIO_PinState value;
};
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t task_handleLED = NULL; /// task handler for LED blinker
TaskHandle_t task_handleRobot = NULL; /// task handler for Robot
TaskHandle_t task_HandleSafetyTask = NULL; /// task handler for safety and IO scanning
TaskHandle_t task_HandleComms = NULL; /// task handler for communicaitons

/*
 Defining outputs
 */
static IO_handle OP0 = { GPIOA_BASE, GPIO_PIN_15, GPIO_PIN_RESET };
static IO_handle OP1 = { GPIOB_BASE, GPIO_PIN_7, GPIO_PIN_RESET };
static IO_handle OP2 = { GPIOC_BASE, GPIO_PIN_13, GPIO_PIN_RESET };
static IO_handle OP3 = { GPIOC_BASE, GPIO_PIN_14, GPIO_PIN_RESET };
static IO_handle OP4 = { GPIOC_BASE, GPIO_PIN_15, GPIO_PIN_RESET };
static IO_handle OP5 = { GPIOA_BASE, GPIO_PIN_0, GPIO_PIN_RESET };
static IO_handle OP6 = { GPIOA_BASE, GPIO_PIN_1, GPIO_PIN_RESET };
static IO_handle OP7 = { GPIOA_BASE, GPIO_PIN_4, GPIO_PIN_RESET };
static IO_handle OP8 = { GPIOB_BASE, GPIO_PIN_0, GPIO_PIN_RESET };
static IO_handle OP9 = { GPIOH_BASE, GPIO_PIN_1, GPIO_PIN_RESET };
/** 
Defining a Robot
*/
static double link_lengths[10] = { 200.0, 150.0, 70.0 };
static axis_data axis0 = { (double)(3200.0 * 20.5) / 360.0, 180.0, -180.0, GPIOC_BASE, GPIO_PIN_14, GPIOC_BASE, GPIO_PIN_14, GPIOB_BASE, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_10, 20, 100, 100, 2000, 0 }; //(3200 * 20.0) / 360.0
static axis_data axis1 = { (double)(3200.0 * 5.5) / 360.0, 140.0, -140.0, GPIOA_BASE, GPIO_PIN_4, GPIOA_BASE, GPIO_PIN_1, GPIOB_BASE, GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_3, 50, 100, 100, 2000, 1 }; //3200 * 12 / 360.0
static axis_data axis2 = { (double)(3200.0 * 1.0) / 360.0, 150.0, 0.0, GPIOA_BASE, GPIO_PIN_0, GPIOC_BASE, GPIO_PIN_15, GPIOB_BASE, GPIO_PIN_4, GPIO_PIN_15, GPIO_PIN_14, 50, 50, 50, 2000, 0 }; //5.1/360.0
static axis_data axis3 = { (double)(3200.0 * 1.0) / 360.0, 360.0, -360.0, GPIOA_BASE, GPIO_PIN_12, GPIOA_BASE, GPIO_PIN_11, GPIOC_BASE, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_6, 50, 100, 100, 2000, 1 };
static axis_data axis4 = { 1000.0, 180.0, -180.0, GPIOA_BASE, GPIO_PIN_0, GPIO_PIN_0, GPIOA_BASE, GPIO_PIN_2, GPIO_PIN_10, GPIO_PIN_3, 50, 200, 100, 2000 };
static axis_data axis5 = { 1000.0, 180.0, -180.0, GPIOA_BASE, GPIO_PIN_0, GPIO_PIN_0, GPIOC_BASE, GPIO_PIN_1, GPIO_PIN_3, GPIO_PIN_2, 50, 200, 100, 2000 };
static robot_data scara_robot_data = { axis0, axis1, axis2, axis3, axis4, axis5, 50.0, 300.0, 300.0 };
static robot scara(0, link_lengths, scara_robot_data);

target_point TARGETS_J[50];
target_point TARGETS_L[50];
target_point TARGETS_C[50][2];

extern "C" void HardFault_Handler(); /// Hard fault finder to trigger a breakpoint

/* Private function prototypes -----------------------------------------------*/
void LEDBlinkTask(void *pvParameters);
void RobotKernel(void *pvParameters);
void SafetyIOTask(void *pvParameters);
void CommsTask(void *pvParameters);
extern "C" {
	void vApplicationIdleHook(void);
}
extern "C" void SysTick_Handler();
/* Private functions ---------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
#endif
void SystemClock_Config(void);

//--------------------------------------- SD Card setup ------------------------------------------------------------
FATFS fs;  // file system
FIL fil;  // file
FRESULT fresult;  // to store the result
char buffer[1024]; // to store data
UINT br, bw;   // file read/write count
/* capacity related variables */
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
int bufsize(char *buf);
void bufclear(void);
//------------------------------------------------------------------------------------------------------------------
bool test1_io_stat;
int main(void){
	/* STM32F4xx HAL library initialization:
	     - Configure the Flash prefetch, instruction and Data caches
	     - Configure the Systick to generate an interrupt each 1 msec
	     - Set NVIC Group Priority to 4
	     - Global MSP (MCU Support Package) initialization
	*/
	HAL_Init();  
	SystemClock_Config();
	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();
	__GPIOD_CLK_ENABLE();
	__GPIOH_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStructure;
	/*
	 * GPIO Init for Drive Pul+dir Connectors
	 */
	/// Inititalise GPIOA Pins 2,3,10 for connector J6
	GPIO_InitStructure.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	///  Inititalise GPIOC Pins 1,2,3 for connector J5
	GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	///  Inititalise GPIOB Pins 3,12,13 for connector J4
	GPIO_InitStructure.Pin = GPIO_PIN_3 | GPIO_PIN_12 | GPIO_PIN_13;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	///  Inititalise GPIOB Pins 4,15,14 for connector J3
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_15 | GPIO_PIN_14;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	///  Inititalise GPIOC Pins 8,9,6 for connector J2
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	///  Inititalise GPIOB Pins 1,2,10 for connector J1
	GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*
	 * GPIO Init for On Board LEDs
	 */
	/// Inititalise GPIOC Pins 10,11,12 for LEDs
	GPIO_InitStructure.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
//	/// Inititalise GPIOA Pin 13 LED - Something wrong with enabling this pin! Triggers breakpoints 
//	GPIO_InitStructure.Pin = GPIO_PIN_13;
//	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
//	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
//	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	/// Inititalise GPIOD Pin 2 LED
	GPIO_InitStructure.Pin = GPIO_PIN_2;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
	/*
	 * GPIO Init for Input Connector J8
	 */
	/// Inititalise GPIOA Pins 0,1,4,15 
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_15 | GPIO_PIN_5;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	/// Inititalise GPIOB Pins 0,7 
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	/// Inititalise GPIOC Pins 13,14,15 
	GPIO_InitStructure.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	/// Inititalise GPIOH Pin 1
	GPIO_InitStructure.Pin = GPIO_PIN_1;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);
	/*
	 * GPIO Init for Output Connector J9
	 */
	/// Inititalise GPIOA Pins 8,9,11,12 
	GPIO_InitStructure.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	/// Initialise GPIOB Pins 5,8,9
	GPIO_InitStructure.Pin = GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	/// Initialise GPIOC Pins 4,5,7
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
	/*
	 * SPI1 Communications Pin Init
	 */
	GPIO_InitStructure.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Pull      = GPIO_PULLUP;
	GPIO_InitStructure.Speed     = GPIO_SPEED_LOW;
	GPIO_InitStructure.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin  = GPIO_PIN_6;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SystemCoreClockUpdate();
	/// Create a task and fill out nessesary information for RTOS to handle the task
	/// "RobotKernel" task has a high stack allocation and a high priority
	xTaskCreate(RobotKernel, "RobotKernel", 4096, NULL, 1, &task_handleRobot);
	/// "LED_blinker" task has minimum stack size and lowest priority 
	xTaskCreate(LEDBlinkTask, "LED_blinker", configMINIMAL_STACK_SIZE, NULL, 1, &task_handleLED);
	/// Safety and IO scanning task
	xTaskCreate(SafetyIOTask, "SafetyIOTask", 2048, NULL, 1, &task_HandleSafetyTask);
	/// Communications task
	xTaskCreate(CommsTask, "CommsTask", 1024, NULL, 1, &task_HandleComms);
	vTaskStartScheduler();
	/* We should never get here as control is now taken by the scheduler */
	for (;;) ;
}

void SysTick_Handler(void){
	HAL_IncTick();
	osSystickHandler();
	HAL_SYSTICK_IRQHandler();
}

/**
  * @brief  Toggle LED1
  * @param  thread not used
  * @retval None
  */
void LEDBlinkTask(void *pvParameters){
	
	while (1){
		// Delay and turn on
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_Delay(1000);
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_Delay(1000);
	}
	vTaskDelete(NULL);
};


/**
  * @brief  Robot Kernel Handle
  * @param  argument not used
  * @retval None
  */
void RobotKernel(void *pvParameters) {
	scara.trajecotry_mode = 1;
	scara.activate();
    vTaskDelay(1000);
	while (!scara.HOME_COMPLETE) {
		scara.home(); // Home the robot first
	}
	TickType_t xLastWakeTime;
	static bool ledState = true;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	for(;;){
		// Load Moves into the movebuffer
//		scara.MOVEJ(TARGETS_J[0], &scara.buffer_pt, 0);
//		scara.MOVEJ(TARGETS_J[1], &scara.buffer_pt, 0); 
//		scara.MOVEJ(TARGETS_J[2], &scara.buffer_pt, 0);
//		scara.MOVEJ(TARGETS_J[3], &scara.buffer_pt, 0);
		scara.MOVEL(TARGETS_L[0], &scara.buffer_pt);
		scara.MOVEL(TARGETS_L[1], &scara.buffer_pt);
		scara.MOVEL(TARGETS_L[0], &scara.buffer_pt);
		scara.MOVEL(TARGETS_L[2], &scara.buffer_pt);
		scara.MOVEL(TARGETS_L[3], &scara.buffer_pt);
		scara.MOVEL(TARGETS_L[2], &scara.buffer_pt);
		
		scara.handle(); // Run the Robot Kernel
		if ((xTaskGetTickCount() - xLastWakeTime) > 100) { // Blink LED to indicate task is alive
			xLastWakeTime = xTaskGetTickCount();
			if (ledState) {
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
				ledState = false;
			}
			else{
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
				ledState = true;
			}
		}
	}
	vTaskDelete(NULL);
};

void SafetyIOTask(void *pvParameters){
	TickType_t xLastWakeTime1;
	const TickType_t xFrequency = 10;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime1 = xTaskGetTickCount();
	static int ESTOP_in;
	static int IN1, IN2, IN3, IN4, IN5, IN6, IN7, IN8, IN9; 
	ESTOP_in = GPIO_PIN_0;
	IN1 = GPIO_PIN_15; // On Port B
	IN2 = GPIO_PIN_7; // On Port C
	IN3 = GPIO_PIN_13; // On Port A
	IN4 = GPIO_PIN_14; // On Port A
	IN5 = GPIO_PIN_15; // On Port B
	IN6 = GPIO_PIN_0; // On Port C
	IN7 = GPIO_PIN_1;// On Port A
	IN8 = GPIO_PIN_4;// On Port A
	IN9 = GPIO_PIN_11; // On Port C
	
	static bool ledState = true;
	static bool enable_state = false;
	static bool last_enable_state = false;
	static bool _enable = true;
	// Initialise the xLastWakeTime variable with the current time.
	
	// Setup Robot I/O
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	
	vTaskDelay(5000);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	static bool estop_status, IN1_status, IN2_status, IN3_status, IN4_status, IN5_status, IN6_status, IN7_status, IN8_status, IN9_status, test1_stat, test2_stat;
	for(;;){
		// Wait for the next cycle.
		/// Scan for EStop and other inputs

		estop_status = HAL_GPIO_ReadPin(GPIOB, ESTOP_in);
		IN1_status = HAL_GPIO_ReadPin(GPIOA, IN1);
		IN2_status = HAL_GPIO_ReadPin(GPIOB, IN2);
		IN3_status = HAL_GPIO_ReadPin(GPIOC, IN3);
		IN4_status = HAL_GPIO_ReadPin(GPIOC, IN4);
		IN5_status = HAL_GPIO_ReadPin(GPIOC, IN5);
		IN6_status = HAL_GPIO_ReadPin(GPIOA, IN6);
		IN7_status = HAL_GPIO_ReadPin(GPIOA, IN7);
		IN8_status = HAL_GPIO_ReadPin(GPIOA, IN8);
		IN9_status = HAL_GPIO_ReadPin(GPIOH, IN9);
		
		scara.j2_fs_status = IN7_status;
		scara.j2_rs_status = IN7_status;
		
		if (IN1_status) enable_state = true;
		else if(!IN1_status) enable_state = false;
		if (enable_state != last_enable_state){
			if (_enable && enable_state) {
				scara.activate();
				_enable = false;
			}
			else if (!_enable && enable_state) {
				scara.deactivate();
				_enable = true;
			}
			last_enable_state = enable_state;
		}
		
		
		if (!estop_status) { // If ESTOP input is 0, meaning its off, meaning someone's pressed it
			scara.EmergencyStop(); // Ask the robot kernel to stop the robot
		}

		if(request_output){ // If some output is requested to be set, do it
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP0.GPIO_BASE), OP0.GPIO_PIN, OP0.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP1.GPIO_BASE), OP1.GPIO_PIN, OP1.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP2.GPIO_BASE), OP2.GPIO_PIN, OP2.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP3.GPIO_BASE), OP3.GPIO_PIN, OP3.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP4.GPIO_BASE), OP4.GPIO_PIN, OP4.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP5.GPIO_BASE), OP5.GPIO_PIN, OP5.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP6.GPIO_BASE), OP6.GPIO_PIN, OP6.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP7.GPIO_BASE), OP7.GPIO_PIN, OP7.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP8.GPIO_BASE), OP8.GPIO_PIN, OP8.value);
			HAL_GPIO_WritePin(((GPIO_TypeDef *) OP9.GPIO_BASE), OP9.GPIO_PIN, OP9.value);
			request_output = false;
		}
		
		if ((xTaskGetTickCount() - xLastWakeTime1) > 200) { // Blink LED to indicate task is alive
			xLastWakeTime1 = xTaskGetTickCount();
			if (ledState) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);
				ledState = false;
			}
			else {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
				ledState = true;
			}
		}
	}
	vTaskDelete(NULL);
};


/*Chip Select pins for all SPI slaves
 *
 * SD Card Module : PC5  - GPIOC GPIO_PIN_5
 * HMI Device     : PB6  - GPIOB GPIO_PIN_6
 * Arduino 33     : PB11 - GPIOB GPIO_PIN_11
 * 
 **/
void CommsTask(void *pvParameters){
	__SPI1_CLK_ENABLE();
	SPI_HandleTypeDef spi = { .Instance = SPI1 };
	spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	spi.Init.Direction = SPI_DIRECTION_2LINES;
	spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
	spi.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi.Init.NSS = SPI_NSS_SOFT;
	spi.Init.TIMode = SPI_TIMODE_DISABLED;
	spi.Init.Mode = SPI_MODE_MASTER; 
	bool handshake_iot, handshake_hmi;
	handshake_iot = false;
	handshake_hmi = false;
	if (HAL_SPI_Init(&spi) != HAL_OK){
		asm("bkpt 255");
	}
	// Initialise SD card
//	MX_FATFS_Init();
//	fresult = f_mount(&fs, "", 0);
//	/*************** Card capacity details ********************/
//	   /* Check free space */
//	f_getfree("", &fre_clust, &pfs);
//	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5); /*************** Card capacity details ********************/
//	bufclear();
//	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
//	/************* The following operation is using PUTS and GETS *********************/
//        /* Open file to write/ create a file if it doesn't exist */
//	fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//	/* Writing text */
//	int fresult_int = f_puts("This data is from the First FILE\n\n", &fil);
//	/* Close file */
//	fresult = f_close(&fil);
	//-------------------
	
	char message_sd[] =  "hsSD99";
	char message_hmi[] = "hsHMI89";
	char message_iot[] = "hsIOT79";
	char message_iot2[] = "IO";
	char message_iot3[] = "T7";
	char message_iot4[] = "9";
	TARGETS_J[0] = { 30, 0, 0, 0, 0, 0 };
	TARGETS_J[1] = { 30, 30, 0, 0, 0, 0 };
	TARGETS_J[2] = { 30, 30, 50, 0, 0, 0 };
	TARGETS_J[3] = { -90, -90, 80, 0, 0, 0 };
	
	TARGETS_L[0] = { 200,  150,  200, 0, 0, 0 };
	TARGETS_L[1] = { 200,  150,  0, 0, 0, 0 };
	TARGETS_L[2] = { 200, -150,  200, 0, 0, 0 };
	TARGETS_L[3] = { 200, -150,  0, 0, 0, 0 };
	
	uint8_t received_data_iot, received_data_hmi, revceived_data_sd;
	// PB6  - Arduino Esplora CS
	// PA11 - Arduino 33 IOT CS
	// PC5  - DFR0229 SD Card Reader CS
	while (!handshake_hmi || !handshake_iot){
		// send handshake to Arduino 33 IoT
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&spi, (uint8_t *)message_iot, strlen(message_iot), HAL_MAX_DELAY);
		HAL_SPI_Receive(&spi, &received_data_iot, 8, HAL_MAX_DELAY);
		//HAL_SPI_TransmitReceive(&spi, (uint8_t *) message_iot, (uint8_t *) received_data_iot, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		
		// send handshake to Arduino Esplora
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_SPI_Transmit(&spi, (uint8_t *)message_hmi, strlen(message_hmi), HAL_MAX_DELAY);
		//HAL_SPI_Receive(&spi, &received_data_hmi, 7, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		
		if (received_data_iot > 1){
			handshake_iot = true;
		}
		if(received_data_hmi > 1){
			handshake_hmi = true;
		}
	}
	
	
	
	for (;;) { // Toogle between Arduino Slave and SD card module to get nesesary data transfer! 
		// REMEMBER!!!! You have changed the SPI1 GPIO_PIN_5 to GPIO_PIN_4 to see the LED blinking.... return it back to what it was, to make SPI work! 
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&spi, (uint8_t *)message_iot, strlen(message_iot), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		vTaskDelay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&spi, (uint8_t *)message_iot2, strlen(message_iot2), HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		vTaskDelay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&spi, (uint8_t *)message_iot3, strlen(message_iot3), HAL_MAX_DELAY);		//HAL_SPI_Receive(&spi, &received_data_iot, 7, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		vTaskDelay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&spi, (uint8_t *)message_iot4, strlen(message_iot4), HAL_MAX_DELAY);
		//HAL_SPI_Receive(&spi, &revceived_data_sd, 7, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

		vTaskDelay(1000);
	}
	vTaskDelete(NULL);
};


void vApplicationIdleHook(void) {
	int i = 0;
	for (;;) {
		i++;    //In my application, this will never be called
	}
}




#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	 /* Infinite loop */
	while (1){
	}
}
#endif

void Error_Handler(){
	HardFault_Handler();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
void SystemClock_Config(void){

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
 
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = 4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
	SystemCoreClockUpdate();
 
	if (HAL_GetREVID() == 0x1001)
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}

/* to find the size of data in the buffer */
int bufsize(char *buf){
	int i = 0;
	while (*buf++ != '\0') i++;
	return i;
}

void bufclear(void){  // clear buffer
	for (int i = 0; i < 1024; i++){
		buffer[i] = '\0';
	}
}