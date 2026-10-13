/**
 * @file    main.c
 * @brief   Example: PING a servo and report whether it answered.
 *
 * The simplest possible use of the driver: send a PING to servo ID 1 and
 * inspect the result in the reply callback. Useful as a first smoke test
 * when wiring up a new bus.
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

static volatile bool g_servo_alive = false;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void on_reply(const STS3215_Reply_t *reply, uint8_t idx, STS3215_Status_t status, void *ctx)
{
	(void)idx;
	(void)ctx;

	/* PING carries no data: a reply at all (even STS3215_ERR_SERVO_FAULT,
	 * which just means the servo reported a non-zero fault byte) proves
	 * the servo is present and answering on the bus. */
	(void)reply;
	(void)status;
	g_servo_alive = true;
}

static void on_error(STS3215_HAL_Error_t err, void *ctx)
{
	(void)ctx;

	/* STS3215_HAL_ERR_TIMEOUT here means no reply was received at all:
	 * check wiring, baud rate, and the servo ID. */
	if (err == STS3215_HAL_ERR_TIMEOUT) {
		g_servo_alive = false;
	}
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

	int16_t len = STS3215_BuildPing(tx_frame, sizeof(tx_frame), 1U);
	if (len > 0) {
		STS3215_HAL_SendFrame(&hservo, tx_frame, (uint16_t)len, false, 1U);
	}
	/* USER CODE END 2 */

	while (1)
	{
		/* USER CODE BEGIN WHILE */
		STS3215_HAL_Process(&hservo);

		if (STS3215_HAL_IsIdle(&hservo)) {
			/* g_servo_alive now reflects the PING result: e.g. light an
			 * LED, log over a debug UART, or gate the rest of your
			 * application on it before doing anything else. */
		}
		/* USER CODE END WHILE */
	}
}
