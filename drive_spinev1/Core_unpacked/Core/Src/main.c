/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "constants.h"
#include "leg_message.h"
#include "structs.h"
#include <string.h>
#include <stdio.h>

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
CANRxMessage can1_rx, can2_rx;
CANTxMessage a1_can, a2_can, h1_can, h2_can, k1_can, k2_can; // команды шести приводам платы


spi_data_t spi_data;       // data from spine to up
spi_command_t spi_command; // data from up to spine
leg_state l1_state, l2_state;
leg_control l1_control, l2_control;

uint16_t rx_buff[RX_LEN];
uint16_t tx_buff[TX_LEN];

uint8_t Serial2RxBuffer[1];

int enabled = 0;
int counter = 0;

/* Один полный кадр @6 МГц ~0.2 ms. 50 ms = оборванный/рассинхронный
 * трансфер, а не нормальная пауза между кадрами @~100 Гц. */
#define SPI_XFER_TIMEOUT_MS 50u
#define DIAG_REPLY_FRESH_MS 100u
#define DIAG_ERROR_RECENT_MS 1000u
#define MOTOR_CURRENT_FULL_SCALE 40.0f

typedef struct
{
  uint32_t can_reply_ms[3];
  uint32_t can_tx_error_ms;
} leg_diagnostics_t;

static leg_diagnostics_t leg_diagnostics[2];
static uint32_t spi_valid_ms;
static uint32_t spi_checksum_error_ms;
static uint32_t spi_timeout_ms;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void write_can_message(CAN_HandleTypeDef *can, CANTxMessage *message,
                              uint32_t leg_index);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void PackAll()
{
	pack_cmd(&a1_can, l1_control.a);
	pack_cmd(&h1_can, l1_control.h);
	pack_cmd(&k1_can, l1_control.k);
	pack_cmd(&a2_can, l2_control.a);
	pack_cmd(&h2_can, l2_control.h);
	pack_cmd(&k2_can, l2_control.k);
}
void WriteAll()
{
//	EnterMotorMode(&a1_can);
	write_can_message(&CAN_H1, &a1_can, 0u);
//    wait(.00002);
	write_can_message(&CAN_H2, &a2_can, 1u);
//    wait(.00002);

	write_can_message(&CAN_H1, &h1_can, 0u);
//    wait(.00002);
	write_can_message(&CAN_H2, &h2_can, 1u);
//    wait(.00002);

	write_can_message(&CAN_H1, &k1_can, 0u);
//    wait(.00002);
	write_can_message(&CAN_H2, &k2_can, 1u);
//    wait(.00002);
}
uint32_t xor_checksum(uint32_t *data, size_t len)
{
    uint32_t t = 0;
    for (int i = 0; i < len; i++)
        t = t ^ data[i];
    return t;
}

static int diag_recent(uint32_t now, uint32_t then, uint32_t window_ms)
{
    return then != 0u && (uint32_t)(now - then) <= window_ms;
}

void diag_record_can_reply(const leg_state *leg, uint8_t id)
{
    uint32_t leg_index;
    if (leg == &l1_state)
    {
        leg_index = 0u;
    }
    else if (leg == &l2_state)
    {
        leg_index = 1u;
    }
    else
    {
        return;
    }

    if (id >= 1u && id <= 3u)
    {
        leg_diagnostics[leg_index].can_reply_ms[id - 1u] = HAL_GetTick();
    }
}

static void diag_record_can_tx(uint32_t leg_index, HAL_StatusTypeDef status)
{
    if (status != HAL_OK)
    {
        leg_diagnostics[leg_index].can_tx_error_ms = HAL_GetTick();
    }
}

static uint32_t diag_flags_for_leg(uint32_t leg_index)
{
    const uint32_t now = HAL_GetTick();
    uint32_t flags = SPINE_DIAG_MAGIC;
    for (uint32_t id = 0u; id < 3u; ++id)
    {
        if (diag_recent(now, leg_diagnostics[leg_index].can_reply_ms[id], DIAG_REPLY_FRESH_MS))
        {
            flags |= SPINE_DIAG_REPLY_ID1 << id;
        }
    }
    if (diag_recent(now, leg_diagnostics[leg_index].can_tx_error_ms, DIAG_ERROR_RECENT_MS))
    {
        flags |= SPINE_DIAG_CAN_TX_ERROR;
    }
    if (diag_recent(now, spi_valid_ms, DIAG_REPLY_FRESH_MS))
    {
        flags |= SPINE_DIAG_SPI_VALID;
    }
    if (diag_recent(now, spi_checksum_error_ms, DIAG_ERROR_RECENT_MS))
    {
        flags |= SPINE_DIAG_SPI_CHECKSUM_ERROR;
    }
    if (diag_recent(now, spi_timeout_ms, DIAG_ERROR_RECENT_MS))
    {
        flags |= SPINE_DIAG_SPI_TIMEOUT;
    }
    if (enabled != 0)
    {
        flags |= SPINE_DIAG_MOTOR_MODE_ENABLED;
    }
    return flags;
}

static void diag_refresh_flags(void)
{
    spi_data.flags[0] = (int32_t)diag_flags_for_leg(0u);
    spi_data.flags[1] = (int32_t)diag_flags_for_leg(1u);
}

static void set_motor_current_led(void)
{
    static uint32_t last_toggle_ms;
    static GPIO_PinState led_state;
    float max_current = fabsf(l1_state.a.t);
    const float currents[] = {
        l1_state.h.t, l1_state.k.t, l2_state.a.t, l2_state.h.t, l2_state.k.t
    };
    for (uint32_t i = 0u; i < sizeof(currents) / sizeof(currents[0]); ++i)
    {
        const float current = fabsf(currents[i]);
        if (current > max_current)
        {
            max_current = current;
        }
    }

    if (enabled == 0)
    {
        led_state = GPIO_PIN_RESET;
        last_toggle_ms = HAL_GetTick();
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, led_state);
        return;
    }

    if (max_current > MOTOR_CURRENT_FULL_SCALE)
    {
        max_current = MOTOR_CURRENT_FULL_SCALE;
    }
    /* 0.5 Hz at zero feedback current, up to 8 Hz at full-scale current. */
    const uint32_t half_period_ms =
        (uint32_t)(1000.0f / (1.0f + 15.0f * max_current / MOTOR_CURRENT_FULL_SCALE));
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_toggle_ms) >= half_period_ms)
    {
        led_state = led_state == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET;
        last_toggle_ms = now;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, led_state);
    }
}

static void write_can_message(CAN_HandleTypeDef *can, CANTxMessage *message, uint32_t leg_index)
{
    uint32_t mailbox;
    diag_record_can_tx(leg_index,
                       HAL_CAN_AddTxMessage(can, &message->tx_header, message->data, &mailbox));
}

int softstop_joint(joint_state state, joint_control *control, float limit_p, float limit_n)
{
    if ((state.p) >= limit_p)
    {
        //control->p_des = limit_p;
        control->v_des = 0.0f;
        control->kp = 0;
        control->kd = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP * (limit_p - state.p);
        return 1;
    }
    else if ((state.p) <= limit_n)
    {
        //control->p_des = limit_n;
        control->v_des = 0.0f;
        control->kp = 0;
        control->kd = KD_SOFTSTOP;
        control->t_ff += KP_SOFTSTOP * (limit_n - state.p);
        return 1;
    }
    return 0;
}

/* 1 — Enter/Exit уже ушли в CAN, spi_isr не должен следом PackAll. */
static int control(void)
{
    if (((spi_command.flags[0] & 0x1) == 1) && (enabled == 0))
    {
        enabled = 1;
        EnterMotorMode(&a1_can);
        EnterMotorMode(&a2_can);
        EnterMotorMode(&k1_can);
        EnterMotorMode(&k2_can);
        EnterMotorMode(&h1_can);
        EnterMotorMode(&h2_can);
        WriteAll();
//        printf("e\n\r");
        return 1;
    }
    else if ((((spi_command.flags[0] & 0x1)) == 0) && (enabled == 1))
    {
        enabled = 0;
        /* Иначе spi_isr сделает PackAll от прошлого PD и hip снова зажмётся. */
        memset(&l1_control, 0, sizeof(l1_control));
        memset(&l2_control, 0, sizeof(l2_control));
        ExitMotorMode(&a1_can);
        ExitMotorMode(&a2_can);
        ExitMotorMode(&h1_can);
        ExitMotorMode(&h2_can);
        ExitMotorMode(&k1_can);
        ExitMotorMode(&k2_can);
        WriteAll();
//        printf("x\n\r");
        return 1;
    }

    spi_data.q_abad[0] = l1_state.a.p;
    spi_data.q_hip[0] = l1_state.h.p;
    spi_data.q_knee[0] = l1_state.k.p;
    spi_data.qd_abad[0] = l1_state.a.v;
    spi_data.qd_hip[0] = l1_state.h.v;
    spi_data.qd_knee[0] = l1_state.k.v;

    spi_data.q_abad[1] = l2_state.a.p;
    spi_data.q_hip[1] = l2_state.h.p;
    spi_data.q_knee[1] = l2_state.k.p;
    spi_data.qd_abad[1] = l2_state.a.v;
    spi_data.qd_hip[1] = l2_state.h.v;
    spi_data.qd_knee[1] = l2_state.k.v;
    spi_data.i_abad[0] = l1_state.a.t;
    spi_data.i_hip[0] = l1_state.h.t;
    spi_data.i_knee[0] = l1_state.k.t;
    spi_data.i_abad[1] = l2_state.a.t;
    spi_data.i_hip[1] = l2_state.h.t;
    spi_data.i_knee[1] = l2_state.k.t;

//	led = 0;

	memset(&l1_control, 0, sizeof(l1_control));
	memset(&l2_control, 0, sizeof(l2_control));

	l1_control.a.p_des = spi_command.q_des_abad[0];
	l1_control.a.v_des = spi_command.qd_des_abad[0];
	l1_control.a.kp = spi_command.kp_abad[0];
	l1_control.a.kd = spi_command.kd_abad[0];
	l1_control.a.t_ff = spi_command.tau_abad_ff[0];

	l1_control.h.p_des = spi_command.q_des_hip[0];
	l1_control.h.v_des = spi_command.qd_des_hip[0];
	l1_control.h.kp = spi_command.kp_hip[0];
	l1_control.h.kd = spi_command.kd_hip[0];
	l1_control.h.t_ff = spi_command.tau_hip_ff[0];

	l1_control.k.p_des = spi_command.q_des_knee[0];
	l1_control.k.v_des = spi_command.qd_des_knee[0];
	l1_control.k.kp = spi_command.kp_knee[0];
	l1_control.k.kd = spi_command.kd_knee[0];
	l1_control.k.t_ff = spi_command.tau_knee_ff[0];

	l2_control.a.p_des = spi_command.q_des_abad[1];
	l2_control.a.v_des = spi_command.qd_des_abad[1];
	l2_control.a.kp = spi_command.kp_abad[1];
	l2_control.a.kd = spi_command.kd_abad[1];
	l2_control.a.t_ff = spi_command.tau_abad_ff[1];

	l2_control.h.p_des = spi_command.q_des_hip[1];
	l2_control.h.v_des = spi_command.qd_des_hip[1];
	l2_control.h.kp = spi_command.kp_hip[1];
	l2_control.h.kd = spi_command.kd_hip[1];
	l2_control.h.t_ff = spi_command.tau_hip_ff[1];

	l2_control.k.p_des = spi_command.q_des_knee[1];
	l2_control.k.v_des = spi_command.qd_des_knee[1];
	l2_control.k.kp = spi_command.kp_knee[1];
	l2_control.k.kd = spi_command.kd_knee[1];
	l2_control.k.t_ff = spi_command.tau_knee_ff[1];

//	spi_data.flags[0] |= softstop_joint(l1_state.a, &l1_control.a, A_LIM_P, A_LIM_N);
//	spi_data.flags[0] |= (softstop_joint(l1_state.h, &l1_control.h, H_LIM_P, H_LIM_N)) << 1;
	//spi_data.flags[0] |= (softstop_joint(l1_state.k, &l1_control.k, K_LIM_P, K_LIM_N))<<2;
//	spi_data.flags[1] |= softstop_joint(l2_state.a, &l2_control.a, A_LIM_P, A_LIM_N);
//	spi_data.flags[1] |= (softstop_joint(l2_state.h, &l2_control.h, H_LIM_P, H_LIM_N)) << 1;
	//spi_data.flags[1] |= (softstop_joint(l2_state.k, &l2_control.k, K_LIM_P, K_LIM_N))<<2;

//	spi_data.flags[0] = //0xabcd;
//	spi_data.flags[1] = 0x1234; //0xbeef;
	//PackAll();
	//WriteAll();


    spi_data.checksum = xor_checksum((uint32_t *)&spi_data, 20);
    for (int i = 0; i < DATA_LEN; i++)
    {
        tx_buff[i] = ((uint16_t *)(&spi_data))[i];
    }
    return 0;
}

/* После оборванного CE/короткого кадра HAL_SPI_TransmitReceive с
 * HAL_MAX_DELAY зависал навсегда → MISO молчит, нужен был power-cycle.
 * Теперь: timeout → abort/reinit → следующий кадр снова принимаем. */
static void spi_bus_recover(void)
{
  HAL_SPI_Abort(&hspi1);
  __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
  HAL_SPI_DeInit(&hspi1);
  MX_SPI1_Init();
}

static void spi_refresh_tx_buff(void)
{
  spi_data.checksum = xor_checksum((uint32_t *)&spi_data, 20);
  for (int i = 0; i < DATA_LEN; i++)
  {
    tx_buff[i] = ((uint16_t *)(&spi_data))[i];
  }
}

void spi_isr(void)
{
  HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
      &hspi1, (uint8_t *)tx_buff, (uint8_t *)rx_buff, TX_LEN, SPI_XFER_TIMEOUT_MS);
  if (st != HAL_OK)
  {
    spi_timeout_ms = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    spi_bus_recover();
    return;
  }

  uint32_t calc_checksum = xor_checksum((uint32_t *)rx_buff, 32);
  for (int i = 0; i < CMD_LEN; i++)
  {
    ((uint16_t *)(&spi_command))[i] = rx_buff[i];
  }

  /* Битый пакет: не гоняем мусор в CAN. Только помечаем и обновляем TX. */
  if (calc_checksum != (uint32_t)spi_command.checksum)
  {
    spi_checksum_error_ms = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
    diag_refresh_flags();
    spi_refresh_tx_buff();
    return;
  }

  /* Enter/Exit уже отправили свой кадр. PackAll здесь затирал Exit прошлым PD. */
  if (control() == 0)
  {
    PackAll();
    WriteAll();
  }
  spi_valid_ms = HAL_GetTick();
  set_motor_current_led();
  diag_refresh_flags();
  spi_refresh_tx_buff();
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  can_rx_init(&can1_rx, &CAN_H1);
  can_rx_init(&can2_rx, &CAN_H2);
  can_tx_init(&a1_can,ABAD_ID);
  can_tx_init(&h1_can,HIP_ID);
  can_tx_init(&k1_can,KNEE_ID);
  can_tx_init(&a2_can,ABAD_ID);
  can_tx_init(&h2_can,HIP_ID);
  can_tx_init(&k2_can,KNEE_ID);
  HAL_CAN_Start(&CAN_H1); //start CAN
  HAL_CAN_Start(&CAN_H2); //start CAN
  HAL_UART_Receive_IT(&huart2, (uint8_t *)Serial2RxBuffer, 1);

  printf("SPIne QV3\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	counter++;
	spi_isr();
	for(int i = 0; i<3; i++){
		int no_mesage1 = HAL_CAN_GetRxMessage(&CAN_H1, CAN_RX_FIFO0, &can1_rx.rx_header, can1_rx.data);	// Read CAN
		int no_mesage2 = HAL_CAN_GetRxMessage(&CAN_H2, CAN_RX_FIFO0, &can2_rx.rx_header, can2_rx.data);	// Read CAN
		if(!no_mesage1){
			unpack_reply(can1_rx, &l1_state);
		}
		if(!no_mesage2){
			unpack_reply(can2_rx, &l2_state);
		}
	}
	if(counter>100){
//		printf("%.3f %.3f %.3f   %.3f %.3f %.3f\n\r", l1_state.a.p, l1_state.h.p, l1_state.k.p, l2_state.a.p, l2_state.h.p, l2_state.k.p);
		counter = 0;
	}
//	HAL_GPIO_WritePin(INDICATOR, GPIO_PIN_SET);
//	HAL_Delay(100);
//	HAL_GPIO_WritePin(INDICATOR, GPIO_PIN_RESET);
//	HAL_Delay(100);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
