/**
 * @file    main.c
//  * @brief   Example: Persist a new servo ID to EEPROM.
 *
 * EEPROM-area registers (like STS3215_REG_ID) require the write-lock to be
 * opened before the write, and it should be closed again afterwards to
 * guard against accidental future writes. This example runs the sequence
 * once at boot: UnlockEEPROM -> WRITE new ID -> LockEEPROM.
 *
 * WARNING: this permanently changes the servo's bus ID. Run this with only
 * one servo on the bus to avoid re-addressing the wrong unit, and remove
 * or guard this code after it has run once: on every boot it will keep
 * trying to rename OLD_ID to NEW_ID, which will silently do nothing once
 * the servo has already been renamed (it no longer answers to OLD_ID).
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
#define OLD_SERVO_ID (1U)
#define NEW_SERVO_ID (5U)

static STS3215_HAL_Handle_t hservo;
static uint8_t tx_frame[STS3215_TX_BUF_SIZE];

typedef enum {
	CFG_STEP_UNLOCK = 0,
	CFG_STEP_WRITE_ID,
	CFG_STEP_LOCK,
	CFG_STEP_DONE,
} cfg_step_t;

static volatile cfg_step_t g_step = CFG_STEP_UNLOCK;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void on_reply(const STS3215_Reply_t *reply, uint8_t idx, STS3215_Status_t status, void *ctx)
{
	(void)idx;
	(void)ctx;

	if (status == STS3215_OK || status == STS3215_ERR_SERVO_FAULT) {
		/* reply->id: for CFG_STEP_UNLOCK / CFG_STEP_WRITE_ID this is still
		 * OLD_SERVO_ID; the servo only starts answering to NEW_SERVO_ID
		 * after the WRITE_ID step completes. */
		(void)reply;
		g_step++;
	}
}

static void on_error(STS3215_HAL_Error_t err, void *ctx)
{
	(void)err;
	(void)ctx;
	/* A timeout here most likely means the wrong ID was addressed for the
	 * current step (see the WARNING above): the sequence will not advance. */
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
	/* USER CODE END 2 */

	while (1)
	{
		/* USER CODE BEGIN WHILE */
		STS3215_HAL_Process(&hservo);

		if (STS3215_HAL_IsIdle(&hservo))
		{
			int16_t len = -1;

			switch (g_step)
			{
			case CFG_STEP_UNLOCK:
				len = STS3215_BuildUnlockEEPROM(tx_frame, sizeof(tx_frame), OLD_SERVO_ID);
				break;

			case CFG_STEP_WRITE_ID:
				len = STS3215_BuildWrite1B(tx_frame, sizeof(tx_frame), OLD_SERVO_ID,
						STS3215_REG_ID, NEW_SERVO_ID);
				break;

			case CFG_STEP_LOCK:
				/* The servo now answers to NEW_SERVO_ID */
				len = STS3215_BuildLockEEPROM(tx_frame, sizeof(tx_frame), NEW_SERVO_ID);
				break;

			case CFG_STEP_DONE:
			default:
				break;
			}

			if (len > 0) {
				STS3215_HAL_SendFrame(&hservo, tx_frame, (uint16_t)len, false, 1U);
			}
		}
		/* USER CODE END WHILE */
	}
}
