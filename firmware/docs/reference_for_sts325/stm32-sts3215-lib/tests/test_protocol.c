/**
 * @file    test_protocol.c
 * @brief   Host-side unit tests for the STS3215 protocol layer.
 *
 * Pure C99, zero STM32 dependency: exercises sts3215_protocol.c only
 * (packet builders, reply parser, unit-conversion helpers).
 *
 * Build & run:
 *   cd tests && make test
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sts3215_protocol.h"
#include "sts3215_regs.h"

/* =========================================================================
 * Minimal test framework
 * ========================================================================= */

static int g_checks    = 0;
static int g_failures  = 0;
static const char *g_current_test = NULL;

#define CHECK(cond) \
	do { \
		g_checks++; \
		if (!(cond)) { \
			g_failures++; \
			fprintf(stderr, "  FAIL [%s] %s:%d: %s\n", g_current_test, __FILE__, __LINE__, #cond); \
		} \
	} while (0)

#define TEST(name) static void test_##name(void)
#define RUN(name) \
	do { \
		g_current_test = #name; \
		printf("RUN  %s\n", #name); \
		test_##name(); \
	} while (0)

/* =========================================================================
 * Helpers
 * ========================================================================= */

/**
 * Recompute the Feetech checksum over a built TX frame and compare it
 * against the trailing byte: independently of the library's own formula.
 */
static uint8_t frame_checksum(const uint8_t *frame, uint16_t len)
{
	uint32_t sum = 0U;
	for (uint16_t i = 2U; i < (uint16_t)(len - 1U); i++) {
		sum += frame[i];
	}
	return (uint8_t)(~sum & 0xFFU);
}

/** Common header/length/instruction/checksum assertions for a TX frame. */
static void check_frame_header(const uint8_t *frame, uint16_t len, uint8_t id, uint8_t instr)
{
	CHECK(frame[0] == STS3215_HEADER_BYTE);
	CHECK(frame[1] == STS3215_HEADER_BYTE);
	CHECK(frame[2] == id);
	CHECK(frame[3] == (uint8_t)(len - 4U)); /* length field = n_params + 2 */
	CHECK(frame[4] == instr);
	CHECK(frame[len - 1U] == frame_checksum(frame, len));
}

/** Hand-assemble a raw reply frame, mirroring the wire format. */
static uint16_t build_raw_reply(uint8_t *buf, uint8_t id, uint8_t error,
		const uint8_t *data, uint8_t data_len)
{
	uint8_t  length = data_len + 2U;
	uint32_t sum    = (uint32_t)id + (uint32_t)length + (uint32_t)error;

	buf[0] = STS3215_HEADER_BYTE;
	buf[1] = STS3215_HEADER_BYTE;
	buf[2] = id;
	buf[3] = length;
	buf[4] = error;
	for (uint8_t i = 0U; i < data_len; i++) {
		buf[5U + i] = data[i];
		sum += data[i];
	}
	buf[5U + data_len] = (uint8_t)(~sum & 0xFFU);

	return (uint16_t)(6U + data_len);
}

static int float_eq(float a, float b, float eps)
{
	return fabsf(a - b) <= eps;
}

/* =========================================================================
 * Packet builder tests
 * ========================================================================= */

TEST(build_ping)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t len = STS3215_BuildPing(buf, sizeof(buf), 1U);

	CHECK(len == 6);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_PING);
}

TEST(build_ping_null_and_small_buffer)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	CHECK(STS3215_BuildPing(NULL, sizeof(buf), 1U) == STS3215_ERR_NULL_PTR);
	CHECK(STS3215_BuildPing(buf, 3U, 1U) == STS3215_ERR_BUFFER_SMALL);
}

TEST(build_read)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t len = STS3215_BuildRead(buf, sizeof(buf), 1U, STS3215_REG_CURRENT_POS, 2U);

	CHECK(len == 8);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_READ_DATA);
	CHECK(buf[5] == STS3215_REG_CURRENT_POS);
	CHECK(buf[6] == 2U);
}

TEST(build_read_zero_len_rejected)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	CHECK(STS3215_BuildRead(buf, sizeof(buf), 1U, STS3215_REG_CURRENT_POS, 0U) == STS3215_ERR_INVALID_PARAM);
}

TEST(build_write1b)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t len = STS3215_BuildWrite1B(buf, sizeof(buf), 1U, STS3215_REG_TORQUE_SWITCH, 1U);

	CHECK(len == 8);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_WRITE_DATA);
	CHECK(buf[5] == STS3215_REG_TORQUE_SWITCH);
	CHECK(buf[6] == 1U);
}

TEST(build_write2b_little_endian)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	int16_t len = STS3215_BuildWrite2B(buf, sizeof(buf), 1U, STS3215_REG_TARGET_POS, 0x1234U);

	CHECK(len == 9);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_WRITE_DATA);
	CHECK(buf[5] == STS3215_REG_TARGET_POS);
	CHECK(buf[6] == 0x34U); /* low byte first */
	CHECK(buf[7] == 0x12U); /* high byte second */
}

TEST(build_write_raw)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	uint8_t data[3] = { 0xAAU, 0xBBU, 0xCCU };
	int16_t len = STS3215_BuildWriteRaw(buf, sizeof(buf), 1U, 0x10U, data, 3U);

	CHECK(len == 10);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_WRITE_DATA);
	CHECK(buf[5] == 0x10U);
	CHECK(memcmp(&buf[6], data, 3U) == 0);
}

TEST(build_write_raw_null_and_oversized)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	uint8_t data[1] = { 0U };

	CHECK(STS3215_BuildWriteRaw(NULL, sizeof(buf), 1U, 0x10U, data, 1U) == STS3215_ERR_NULL_PTR);
	CHECK(STS3215_BuildWriteRaw(buf, sizeof(buf), 1U, 0x10U, NULL, 1U) == STS3215_ERR_NULL_PTR);
	CHECK(STS3215_BuildWriteRaw(buf, sizeof(buf), 1U, 0x10U, data, 0U) == STS3215_ERR_INVALID_PARAM);
	CHECK(STS3215_BuildWriteRaw(buf, sizeof(buf), 1U, 0x10U, data, (uint8_t)(STS3215_PARAM_MAX + 1U)) == STS3215_ERR_INVALID_PARAM);
}

TEST(build_motion_cmd)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	STS3215_MotionCmd_t cmd = {
			.acceleration  = 50U,
			.target_pos    = 1024,
			.running_time  = 0U,
			.running_speed = 1000U,
	};
	int16_t len = STS3215_BuildMotionCmd(buf, sizeof(buf), 1U, &cmd);

	CHECK(len == 14);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_WRITE_DATA);
	CHECK(buf[5] == STS3215_REG_ACCELERATION);
	CHECK(buf[6] == 50U);
	CHECK(STS3215_UnpackS16LE(&buf[7]) == 1024);
	CHECK(STS3215_UnpackU16LE(&buf[9]) == 0U);
	CHECK(STS3215_UnpackU16LE(&buf[11]) == 1000U);
}

TEST(build_motion_cmd_null_ptr)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	CHECK(STS3215_BuildMotionCmd(buf, sizeof(buf), 1U, NULL) == STS3215_ERR_NULL_PTR);
}

TEST(build_reg_write_and_action)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	uint8_t data[2] = { 0x01U, 0x02U };
	int16_t len = STS3215_BuildRegWrite(buf, sizeof(buf), 1U, 0x2AU, data, 2U);

	CHECK(len == 9);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_REG_WRITE_DATA);

	len = STS3215_BuildAction(buf, sizeof(buf), STS3215_BROADCAST_ID);
	CHECK(len == 6);
	check_frame_header(buf, (uint16_t)len, STS3215_BROADCAST_ID, STS3215_INSTR_ACTION);
}

TEST(build_sync_write)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	STS3215_SyncEntry_t entries[2] = {
			{ .id = 1U, .data = { 0x00U, 0x04U } },
			{ .id = 2U, .data = { 0x00U, 0x08U } },
	};
	int16_t len = STS3215_BuildSyncWrite(buf, sizeof(buf), STS3215_REG_TARGET_POS, 2U, entries, 2U);

	CHECK(len == 14); /* header/id/len/instr(5) + reg + data_len + 2*(id+2B) + checksum */
	check_frame_header(buf, (uint16_t)len, STS3215_BROADCAST_ID, STS3215_INSTR_SYNC_WRITE_DATA);
	CHECK(buf[5] == STS3215_REG_TARGET_POS);
	CHECK(buf[6] == 2U);
	CHECK(buf[7] == 1U);  /* entry 0 id  */
	CHECK(buf[10] == 2U); /* entry 1 id  */
}

TEST(build_sync_write_invalid_params)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	STS3215_SyncEntry_t entries[1] = { { .id = 1U, .data = { 0U } } };

	CHECK(STS3215_BuildSyncWrite(buf, sizeof(buf), 0x2AU, 2U, entries, 0U) == STS3215_ERR_INVALID_PARAM);
	CHECK(STS3215_BuildSyncWrite(buf, sizeof(buf), 0x2AU, 0U, entries, 1U) == STS3215_ERR_INVALID_PARAM);
	CHECK(STS3215_BuildSyncWrite(buf, sizeof(buf), 0x2AU, (uint8_t)(STS3215_SYNC_DATA_MAX_LEN + 1U), entries, 1U) == STS3215_ERR_INVALID_PARAM);
}

TEST(build_sync_read)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];
	uint8_t ids[3] = { 1U, 2U, 3U };
	int16_t len = STS3215_BuildSyncRead(buf, sizeof(buf), STS3215_REG_CURRENT_POS, 2U, ids, 3U);

	CHECK(len == 11);
	check_frame_header(buf, (uint16_t)len, STS3215_BROADCAST_ID, STS3215_INSTR_SYNC_READ_DATA);
	CHECK(buf[5] == STS3215_REG_CURRENT_POS);
	CHECK(buf[6] == 2U);
	CHECK(memcmp(&buf[7], ids, 3U) == 0);
}

TEST(build_torque_and_center_helpers)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];

	STS3215_BuildEnableTorque(buf, sizeof(buf), 1U);
	CHECK(buf[5] == STS3215_REG_TORQUE_SWITCH);
	CHECK(buf[6] == (uint8_t)STS3215_TORQUE_ON);

	STS3215_BuildDisableTorque(buf, sizeof(buf), 1U);
	CHECK(buf[6] == (uint8_t)STS3215_TORQUE_OFF);

	STS3215_BuildCenterServo(buf, sizeof(buf), 1U);
	CHECK(buf[6] == (uint8_t)STS3215_TORQUE_CENTER);
}

TEST(build_eeprom_lock_helpers)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];

	STS3215_BuildUnlockEEPROM(buf, sizeof(buf), 1U);
	CHECK(buf[5] == STS3215_REG_LOCK_MARK);
	CHECK(buf[6] == (uint8_t)STS3215_LOCK_OPEN);

	STS3215_BuildLockEEPROM(buf, sizeof(buf), 1U);
	CHECK(buf[6] == (uint8_t)STS3215_LOCK_CLOSED);
}

TEST(build_reset_helpers)
{
	uint8_t buf[STS3215_TX_BUF_SIZE];

	int16_t len = STS3215_BuildReset(buf, sizeof(buf), 1U);
	CHECK(len == 6);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_RESET);

	len = STS3215_BuildResetTurns(buf, sizeof(buf), 1U);
	CHECK(len == 6);
	check_frame_header(buf, (uint16_t)len, 1U, STS3215_INSTR_RESET_TURNS);
}

/* =========================================================================
 * Reply parser tests
 * ========================================================================= */

TEST(parse_reply_ok)
{
	uint8_t buf[STS3215_RX_BUF_SIZE];
	uint8_t data[2] = { 0x00U, 0x04U };
	uint16_t len = build_raw_reply(buf, 1U, 0U, data, 2U);

	STS3215_Reply_t reply;
	STS3215_Status_t status = STS3215_ParseReply(buf, len, &reply);

	CHECK(status == STS3215_OK);
	CHECK(reply.id == 1U);
	CHECK(reply.error == 0U);
	CHECK(reply.data_len == 2U);
	CHECK(memcmp(reply.data, data, 2U) == 0);
}

TEST(parse_reply_servo_fault)
{
	uint8_t buf[STS3215_RX_BUF_SIZE];
	uint16_t len = build_raw_reply(buf, 1U, STS3215_FAULT_OVERLOAD, NULL, 0U);

	STS3215_Reply_t reply;
	STS3215_Status_t status = STS3215_ParseReply(buf, len, &reply);

	CHECK(status == STS3215_ERR_SERVO_FAULT);
	CHECK(reply.error == STS3215_FAULT_OVERLOAD);
}

TEST(parse_reply_bad_header)
{
	uint8_t buf[STS3215_RX_BUF_SIZE] = { 0x00U, 0xFFU, 1U, 2U, 0U, 0xFDU };
	STS3215_Reply_t reply;

	CHECK(STS3215_ParseReply(buf, 6U, &reply) == STS3215_ERR_BAD_HEADER);
}

TEST(parse_reply_bad_checksum)
{
	uint8_t buf[STS3215_RX_BUF_SIZE];
	uint16_t len = build_raw_reply(buf, 1U, 0U, NULL, 0U);
	buf[len - 1U] ^= 0xFFU; /* corrupt the checksum byte */

	STS3215_Reply_t reply;
	CHECK(STS3215_ParseReply(buf, len, &reply) == STS3215_ERR_BAD_CHECKSUM);
}

TEST(parse_reply_frame_short)
{
	uint8_t buf[STS3215_RX_BUF_SIZE];
	uint16_t len = build_raw_reply(buf, 1U, 0U, NULL, 0U);
	STS3215_Reply_t reply;

	CHECK(STS3215_ParseReply(buf, (uint16_t)(len - 1U), &reply) == STS3215_ERR_FRAME_SHORT);
	CHECK(STS3215_ParseReply(buf, 3U, &reply) == STS3215_ERR_FRAME_SHORT);
}

TEST(parse_reply_null_ptr)
{
	uint8_t buf[STS3215_RX_BUF_SIZE] = { 0 };
	STS3215_Reply_t reply;

	CHECK(STS3215_ParseReply(NULL, 6U, &reply) == STS3215_ERR_NULL_PTR);
	CHECK(STS3215_ParseReply(buf, 6U, NULL) == STS3215_ERR_NULL_PTR);
}

/* =========================================================================
 * Unit conversion tests
 * ========================================================================= */

TEST(unpack_le)
{
	uint8_t buf[2] = { 0x34U, 0x12U };
	CHECK(STS3215_UnpackU16LE(buf) == 0x1234U);
	CHECK(STS3215_UnpackS16LE(buf) == 0x1234);

	uint8_t neg[2] = { 0xFFU, 0xFFU };
	CHECK(STS3215_UnpackS16LE(neg) == -1);
}

TEST(steps_deg_rad_roundtrip)
{
	CHECK(float_eq(STS3215_StepsToDeg(2048), 180.0f, 0.01f));
	CHECK(float_eq(STS3215_StepsToRad(2048), 3.14159f, 0.01f));
	CHECK(STS3215_DegToSteps(180.0f) == 2048);
}

TEST(raw_current_voltage_conversion)
{
	CHECK(float_eq(STS3215_RawToMilliamps(100U), 650.0f, 0.001f));
	CHECK(float_eq(STS3215_RawToVolts(80U), 8.0f, 0.001f));
}

/* =========================================================================
 * Runner
 * ========================================================================= */

int main(void)
{
	RUN(build_ping);
	RUN(build_ping_null_and_small_buffer);
	RUN(build_read);
	RUN(build_read_zero_len_rejected);
	RUN(build_write1b);
	RUN(build_write2b_little_endian);
	RUN(build_write_raw);
	RUN(build_write_raw_null_and_oversized);
	RUN(build_motion_cmd);
	RUN(build_motion_cmd_null_ptr);
	RUN(build_reg_write_and_action);
	RUN(build_sync_write);
	RUN(build_sync_write_invalid_params);
	RUN(build_sync_read);
	RUN(build_torque_and_center_helpers);
	RUN(build_eeprom_lock_helpers);
	RUN(build_reset_helpers);

	RUN(parse_reply_ok);
	RUN(parse_reply_servo_fault);
	RUN(parse_reply_bad_header);
	RUN(parse_reply_bad_checksum);
	RUN(parse_reply_frame_short);
	RUN(parse_reply_null_ptr);

	RUN(unpack_le);
	RUN(steps_deg_rad_roundtrip);
	RUN(raw_current_voltage_conversion);

	printf("\n%d checks, %d failures\n", g_checks, g_failures);
	return (g_failures == 0) ? 0 : 1;
}
