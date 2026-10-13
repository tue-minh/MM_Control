/**
 * @file    main.c
 * @brief   Example: Poll the full feedback block of a servo every 200 ms.
 *
 * Registers 0x38..0x3F (current position, speed, load, voltage, temperature)
 * are contiguous, so a single READ of 8 bytes starting at
 * STS3215_REG_CURRENT_POS retrieves the whole feedback block in one
 * round trip: no need for five separate reads.
 *
 * Copy each USER CODE block into the matching section of your
 * STM32CubeIDE-generated Core/Src/main.c. See examples/README.md.
 */

/* USER CODE BEGIN Includes */
#include "sts3215_regs.h"
#include "sts3215_protocol.h"
#include "sts3215_hal.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static STS3215_HAL_Handle_t hservo;
static uint8_t tx_frame[STS3215_TX_BUF_SIZE];

/* Latest decoded feedback, updated from the reply callback */
static volatile int16_t  g_position_steps;
static volatile int16_t  g_speed_steps_s;
static volatile int16_t  g_load_raw;      /* 0.001 units, signed (direction) */
static volatile float    g_voltage_v;
static volatile uint8_t  g_temperature_c;
static volatile bool     g_feedback_valid = false;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void on_reply(const STS3215_Reply_t *reply, uint8_t idx, STS3215_Status_t status, void *ctx)
{
	(void)idx;
	(void)ctx;

	if (status != STS3215_OK || reply->data_len < 8U) {
		g_feedback_valid = false;
		return;
	}

	g_position_steps = STS3215_UnpackS16LE(&reply->data[0]); /* 0x38 */
	g_speed_steps_s  = STS3215_UnpackS16LE(&reply->data[2]); /* 0x3A */
	g_load_raw       = STS3215_UnpackS16LE(&reply->data[4]); /* 0x3C */
	g_voltage_v      = STS3215_RawToVolts(reply->data[6]);   /* 0x3E */
	g_temperature_c  = reply->data[7];                       /* 0x3F, raw = °C */
	g_feedback_valid = true;
}

static void on_error(STS3215_HAL_Error_t err, void *ctx)
{
	(void)err;
	(void)ctx;
	g_feedback_valid = false;
}
/* USER CODE END 0 */

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();

	/* USER CODE BEGIN 2 */
	STS3215_HAL_Init(&hservo, &huart2, 10U, on_reply, on_error, NULL);
	STS3215_HAL_RegisterInstance(&hservo);

	uint32_t last_poll_ms = HAL_GetTick();
	/* USER CODE END 2 */

	while (1)
	{
		/* USER CODE BEGIN WHILE */
		STS3215_HAL_Process(&hservo);

		bool bus_is_free = STS3215_HAL_IsIdle(&hservo);
		bool poll_due     = (HAL_GetTick() - last_poll_ms) >= 200U;

		if (bus_is_free && poll_due) {
			last_poll_ms = HAL_GetTick();

			int16_t len = STS3215_BuildRead(tx_frame, sizeof(tx_frame), 1U,
					STS3215_REG_CURRENT_POS, 8U);
			if (len > 0) {
				STS3215_HAL_SendFrame(&hservo, tx_frame, (uint16_t)len, false, 1U);
			}
		}

		if (g_feedback_valid) {
			/* e.g. forward g_position_steps / g_speed_steps_s / g_load_raw /
			 * g_voltage_v / g_temperature_c to a logger, a debug UART, or a
			 * higher-level control loop. STS3215_StepsToDeg(g_position_steps)
			 * converts the raw position to degrees if needed. */
		}
		/* USER CODE END WHILE */
	}
}
