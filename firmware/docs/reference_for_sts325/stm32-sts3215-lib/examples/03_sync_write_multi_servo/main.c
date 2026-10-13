/**
 * @file    main.c
 * @brief   Example: Move 3 servos to different positions in one frame.
 *
 * SYNC_WRITE is broadcast: a single frame carries a per-servo payload for
 * every listed ID, and all targeted servos start moving on receipt of the
 * *same* frame: there's no per-servo timing skew like there would be with
 * three sequential WRITE commands. No reply is expected (broadcast).
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
#define NUM_SERVOS (3U)

static STS3215_HAL_Handle_t hservo;
static uint8_t tx_frame[STS3215_TX_BUF_SIZE];

static const uint8_t servo_ids[NUM_SERVOS] = { 1U, 2U, 3U };
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void on_reply(const STS3215_Reply_t *reply, uint8_t idx, STS3215_Status_t status, void *ctx)
{
	/* SYNC_WRITE is a broadcast instruction: servos never reply to it,
	 * so this callback is not invoked for the frame this example sends. */
	(void)reply;
	(void)idx;
	(void)status;
	(void)ctx;
}

static void on_error(STS3215_HAL_Error_t err, void *ctx)
{
	(void)err;
	(void)ctx;
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

	uint32_t last_move_ms = HAL_GetTick();
	uint8_t  toggle       = 0U;
	/* USER CODE END 2 */

	while (1)
	{
		/* USER CODE BEGIN WHILE */
		STS3215_HAL_Process(&hservo);

		bool bus_is_free      = STS3215_HAL_IsIdle(&hservo);
		bool time_has_elapsed = (HAL_GetTick() - last_move_ms) >= 2000U;

		if (bus_is_free && time_has_elapsed)
		{
			last_move_ms = HAL_GetTick();
			toggle ^= 1U;

			/* Every servo gets its own target position; speed/acceleration
			 * stay at their currently configured values since SYNC_WRITE
			 * here only touches STS3215_REG_TARGET_POS (2 bytes/servo).
			 * Use STS3215_BuildMotionCmd per servo first if you also need
			 * per-servo speed/acceleration in the same cycle. */
			STS3215_SyncEntry_t entries[NUM_SERVOS];
			for (uint8_t i = 0U; i < NUM_SERVOS; i++) {
				int16_t target = toggle ? (int16_t)(1024 + i * 256) : (int16_t)(3072 - i * 256);

				entries[i].id      = servo_ids[i];
				entries[i].data[0] = (uint8_t)((uint16_t)target & 0xFFU);
				entries[i].data[1] = (uint8_t)(((uint16_t)target >> 8U) & 0xFFU);
			}

			int16_t len = STS3215_BuildSyncWrite(tx_frame, sizeof(tx_frame),
					STS3215_REG_TARGET_POS, 2U, entries, NUM_SERVOS);

			if (len > 0) {
				/* Broadcast: is_broadcast=true, no reply expected */
				STS3215_HAL_SendFrame(&hservo, tx_frame, (uint16_t)len, true, 0U);
			}
		}
		/* USER CODE END WHILE */
	}
}
