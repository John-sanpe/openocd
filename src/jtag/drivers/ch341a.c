// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright(c) 2021 John Sanpe <sanpeqf@gmail.com>
 */

/* project specific includes */
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include "libusb_helper.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#define CH341A_USB_VENDOR           	0x1a86
#define CH341A_USB_PRODUCT          	0x5512

#define CH341A_PACKET_LENGTH        	32U
#define CH341A_MAX_PACKETS          	256U
#define CH341A_BULK_TIMEOUT         	1000U

#define CH341A_BULK_READ_ENDPOINT   	0x82
#define CH341A_BULK_WRITE_ENDPOINT  	0x02
#define CH341A_VENDOR_READ          	0xc0
#define CH341A_VENDOR_VERSION       	0x5f

#define CH341A_CMD_UIO_STREAM       	0xab
#define CH341A_UIO_CMD_STM_IN       	0x00
#define CH341A_UIO_CMD_STM_DIR      	0x40
#define CH341A_UIO_CMD_STM_OUT      	0x80
#define CH341A_UIO_CMD_STM_END      	0x20

enum ch341a_pin_num {
	CH341A_PIN_D0       = 0,
	CH341A_PIN_D1       = 1,
	CH341A_PIN_D2       = 2,
	CH341A_PIN_D3       = 3,
	CH341A_PIN_D4       = 4,
	CH341A_PIN_D5       = 5,
	CH341A_PIN_D6       = 6,
	CH341A_PIN_D7       = 7,
	CH341A_PIN_NULL,
};

static const char * const ch341a_pin_name[] = {
	[CH341A_PIN_D0]     = "D0",
	[CH341A_PIN_D1]     = "D1",
	[CH341A_PIN_D2]     = "D2",
	[CH341A_PIN_D3]     = "D3",
	[CH341A_PIN_D4]     = "D4",
	[CH341A_PIN_D5]     = "D5",
	[CH341A_PIN_D6]     = "D6",
	[CH341A_PIN_D7]     = "D7",
};

static uint16_t ch341a_vid = CH341A_USB_VENDOR;
static uint16_t ch341a_pid = CH341A_USB_PRODUCT;
static struct libusb_device_handle *ch341a_adapter;

static unsigned int ch341a_tck_gpio  = CH341A_PIN_D3;
static unsigned int ch341a_tms_gpio  = CH341A_PIN_D0;
static unsigned int ch341a_tdo_gpio  = CH341A_PIN_D7;
static unsigned int ch341a_tdi_gpio  = CH341A_PIN_D5;
static unsigned int ch341a_trst_gpio = CH341A_PIN_D1;
static unsigned int ch341a_srst_gpio = CH341A_PIN_D2;

static uint8_t *ch341a_port_buffer;
static unsigned long ch341a_port_buffer_curr;
static unsigned long ch341a_port_buffer_size = 4096;

static enum ch341a_pin_num ch341a_name_to_pin(const char *name)
{
	unsigned int count;

	for (count = 0; count < ARRAY_SIZE(ch341a_pin_name); ++count)
		if (!strcmp(ch341a_pin_name[count], name))
			return count;

	return CH341A_PIN_NULL;
}

static bool ch341a_pin_is_read(enum ch341a_pin_num num)
{
	return num <= CH341A_PIN_D7;
}

static bool ch341a_pin_is_write(enum ch341a_pin_num num)
{
	return num <= CH341A_PIN_D5;
}

COMMAND_HANDLER(ch341a_handle_vid_pid_command)
{
	if (CMD_ARGC > 2) {
		LOG_WARNING("ignoring extra IDs in ch341a_vid_pid "
					"(maximum is 1 pair)");
		CMD_ARGC = 2;
	}
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], ch341a_vid);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], ch341a_pid);
	} else {
		LOG_WARNING("incomplete ch341a_vid_pid configuration");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(ch341a_handle_jtag_nums_command)
{
	if (CMD_ARGC != 8)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch341a_tck_gpio     = ch341a_name_to_pin(CMD_ARGV[0]);
	ch341a_tms_gpio     = ch341a_name_to_pin(CMD_ARGV[1]);
	ch341a_tdo_gpio     = ch341a_name_to_pin(CMD_ARGV[2]);
	ch341a_tdi_gpio     = ch341a_name_to_pin(CMD_ARGV[3]);
	ch341a_trst_gpio    = ch341a_name_to_pin(CMD_ARGV[4]);
	ch341a_srst_gpio    = ch341a_name_to_pin(CMD_ARGV[5]);

	if (!ch341a_pin_is_write(ch341a_tck_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	if (!ch341a_pin_is_write(ch341a_tms_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	if (!ch341a_pin_is_read(ch341a_tdo_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	if (!ch341a_pin_is_write(ch341a_tdi_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	if (!ch341a_pin_is_write(ch341a_trst_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	if (!ch341a_pin_is_write(ch341a_srst_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	command_print(CMD, "ch341a nums: "
		"TCK = %d %s, TMS = %d %s, TDI = %d %s,"
		"TDO = %d %s, TRST = %d %s, SRST = %d %s",
		ch341a_tck_gpio,  ch341a_pin_name[ch341a_tck_gpio],
		ch341a_tms_gpio,  ch341a_pin_name[ch341a_tms_gpio],
		ch341a_tdo_gpio,  ch341a_pin_name[ch341a_tdo_gpio],
		ch341a_tdi_gpio,  ch341a_pin_name[ch341a_tdi_gpio],
		ch341a_trst_gpio, ch341a_pin_name[ch341a_trst_gpio],
		ch341a_srst_gpio, ch341a_pin_name[ch341a_srst_gpio]
	);

	return ERROR_OK;
}

COMMAND_HANDLER(ch341a_handle_tck_num_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch341a_tck_gpio = ch341a_name_to_pin(CMD_ARGV[0]);

	if (!ch341a_pin_is_write(ch341a_tck_gpio))
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "ch341a num: TCK = %d %s",
				  ch341a_tck_gpio, ch341a_pin_name[ch341a_tck_gpio]);

	return ERROR_OK;
}

COMMAND_HANDLER(ch341a_handle_tms_num_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch341a_tms_gpio = ch341a_name_to_pin(CMD_ARGV[0]);

	if (!ch341a_pin_is_write(ch341a_tms_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	command_print(CMD, "ch341a num: TMS = %d %s",
				  ch341a_tms_gpio, ch341a_pin_name[ch341a_tms_gpio]);

	return ERROR_OK;
}

COMMAND_HANDLER(ch341a_handle_tdo_num_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch341a_tdo_gpio = ch341a_name_to_pin(CMD_ARGV[0]);

	if (!ch341a_pin_is_read(ch341a_tdo_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	command_print(CMD, "ch341a num: TDO = %d %s",
				  ch341a_tdo_gpio, ch341a_pin_name[ch341a_tdo_gpio]);

	return ERROR_OK;
}

COMMAND_HANDLER(ch341a_handle_tdi_num_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch341a_tdi_gpio = ch341a_name_to_pin(CMD_ARGV[0]);

	if (!ch341a_pin_is_write(ch341a_tdi_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	command_print(CMD, "ch341a num: TDI = %d %s",
				  ch341a_tdi_gpio, ch341a_pin_name[ch341a_tdi_gpio]);

	return ERROR_OK;
}

COMMAND_HANDLER(ch341a_handle_trst_num_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch341a_trst_gpio = ch341a_name_to_pin(CMD_ARGV[0]);

	if (!ch341a_pin_is_write(ch341a_trst_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	command_print(CMD, "ch341a num: TRST = %d %s",
				  ch341a_trst_gpio, ch341a_pin_name[ch341a_trst_gpio]);

	return ERROR_OK;
}

COMMAND_HANDLER(ch341a_handle_srst_num_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	ch341a_srst_gpio = ch341a_name_to_pin(CMD_ARGV[0]);

	if (!ch341a_pin_is_write(ch341a_srst_gpio))
		return ERROR_COMMAND_CLOSE_CONNECTION;

	command_print(CMD, "ch341a num: SRST = %d %s",
				  ch341a_srst_gpio, ch341a_pin_name[ch341a_srst_gpio]);

	return ERROR_OK;
}

static int ch341a_port_direction(uint8_t dire)
{
	uint8_t transfer[3];
	int ret, received;

	transfer[0] = CH341A_CMD_UIO_STREAM;
	transfer[1] = CH341A_UIO_CMD_STM_DIR | (0x3f & dire);
	transfer[2] = CH341A_UIO_CMD_STM_END;

	ret = jtag_libusb_bulk_write(ch341a_adapter, CH341A_BULK_WRITE_ENDPOINT,
								 (void *)transfer, 3, CH341A_BULK_TIMEOUT, &received);

	if (ret < 0 || received < 0) {
		LOG_ERROR("%s: usb bulk write failed", __func__);
		exit(1);
	}

	return received;
}

static int ch341a_port_transfer(uint8_t *buff, unsigned int len, bool read)
{
	uint8_t transfer[CH341A_PACKET_LENGTH + 2];
	unsigned int xfer;
	int ret, received;

	for (received = 0; (xfer = MIN(len, CH341A_PACKET_LENGTH / (read + 1))); len -= xfer, buff += xfer) {
		unsigned int xfer_send = xfer * (read + 1) + 2;
		unsigned int count;
		int xfer_recv;

		transfer[0] = CH341A_CMD_UIO_STREAM;
		transfer[xfer_send - 1] = CH341A_UIO_CMD_STM_END;

		if (read) {
			for (count = 0; count < xfer; ++count) {
				transfer[(count + 1) * 2 - 1] = CH341A_UIO_CMD_STM_OUT | (0x3f & *(buff + count));
				transfer[(count + 1) * 2 + 0] = CH341A_UIO_CMD_STM_IN;
			}
		} else {
			for (count = 0; count < xfer; ++count)
				transfer[count + 1] = CH341A_UIO_CMD_STM_OUT | (0x3f & *(buff + count));
		}

		ret = jtag_libusb_bulk_write(ch341a_adapter, CH341A_BULK_WRITE_ENDPOINT,
									 (void *)transfer, xfer_send, CH341A_BULK_TIMEOUT, &xfer_recv);

		if (ret < 0 || xfer_recv < 0) {
			LOG_ERROR("%s: usb bulk write failed", __func__);
			exit(1);
		}

		if (read) {
			ret = jtag_libusb_bulk_read(ch341a_adapter, CH341A_BULK_READ_ENDPOINT,
										(void *)buff, xfer, CH341A_BULK_TIMEOUT, &xfer_recv);

			if (ret < 0 || xfer_recv < 0) {
				LOG_ERROR("%s: usb bulk read failed", __func__);
				exit(1);
			}
			received += xfer_recv;
		}
	}

	return received;
}

static void ch341a_port_buffer_flush(bool read)
{
	ch341a_port_transfer(ch341a_port_buffer, ch341a_port_buffer_curr, read);
	ch341a_port_buffer_curr = 0;
}

static void ch341a_port_buffer_increase(unsigned long new_size)
{
	uint8_t *new_ptr;

	if (new_size < ch341a_port_buffer_size)
		return;

	new_size = ch341a_port_buffer_size * 2;
	new_ptr = realloc(ch341a_port_buffer, new_size);
	if (!new_ptr)
		return;

	ch341a_port_buffer = new_ptr;
	ch341a_port_buffer_size = new_size;
}

static inline void ch341a_jtag_write(bool tck, bool tms, bool tdi)
{
	uint8_t value = 0xff;

	value = tck ? value | (1 << ch341a_tck_gpio) : value & ~(1 << ch341a_tck_gpio);
	value = tms ? value | (1 << ch341a_tms_gpio) : value & ~(1 << ch341a_tms_gpio);
	value = tdi ? value | (1 << ch341a_tdi_gpio) : value & ~(1 << ch341a_tdi_gpio);

	ch341a_port_buffer_increase(ch341a_port_buffer_curr);
	if (ch341a_port_buffer_curr >= ch341a_port_buffer_size) {
		LOG_ERROR("%s: buffer overflow", __func__);
		exit(1);
	}

	ch341a_port_buffer[ch341a_port_buffer_curr++] = value;
}

static inline void ch341a_jtag_reset(bool trst, bool srst)
{
	uint8_t value = 0xff;

	value = trst ? value & ~(1 << ch341a_trst_gpio) : value | (1 << ch341a_trst_gpio);
	value = srst ? value & ~(1 << ch341a_srst_gpio) : value | (1 << ch341a_srst_gpio);

	ch341a_port_buffer_increase(ch341a_port_buffer_curr);
	if (ch341a_port_buffer_curr >= ch341a_port_buffer_size) {
		LOG_ERROR("%s: buffer overflow", __func__);
		exit(1);
	}

	ch341a_port_buffer[ch341a_port_buffer_curr++] = value;
	ch341a_port_buffer_flush(false);
}

static void syncbb_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state)) {
		tap_set_end_state(state);
	} else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void syncbb_state_move(int skip)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
	bool tms = 0;
	int count;

	for (count = skip; count < tms_count; ++count) {
		tms = (tms_scan >> count) & 0x01;
		ch341a_jtag_write(0, tms, 0);
		ch341a_jtag_write(1, tms, 0);
	}

	ch341a_jtag_write(0, tms, 0);
	tap_set_state(tap_get_end_state());
}

static void syncbb_execute_tms(struct jtag_command *cmd)
{
	unsigned int num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	bool tms = 0;

	LOG_DEBUG_IO("TMS: %d bits", num_bits);

	for (unsigned int i = 0; i < num_bits; i++) {
		tms = ((bits[i / 8] >> (i % 8)) & 1);
		ch341a_jtag_write(0, tms, 0);
		ch341a_jtag_write(1, tms, 0);
	}

	ch341a_jtag_write(0, tms, 0);
}

static void syncbb_path_move(struct pathmove_command *cmd)
{
	int num_states = cmd->num_states;
	int state_count = 0;
	bool tms = 0;

	while (num_states--) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count]) {
			tms = 0;
		} else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count]) {
			tms = 1;
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		ch341a_jtag_write(0, tms, 0);
		ch341a_jtag_write(1, tms, 0);
		tap_set_state(cmd->path[state_count]);
		state_count++;
	}

	ch341a_jtag_write(0, tms, 0);
	tap_set_end_state(tap_get_state());
}

static void syncbb_runtest(unsigned int cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();
	unsigned int count;

	if (tap_get_state() != TAP_IDLE) {
		syncbb_end_state(TAP_IDLE);
		syncbb_state_move(0);
	}

	for (count = 0; count < cycles; ++count) {
		ch341a_jtag_write(0, 0, 0);
		ch341a_jtag_write(1, 0, 0);
	}

	ch341a_jtag_write(0, 0, 0);
	syncbb_end_state(saved_end_state);

	if (tap_get_state() != tap_get_end_state())
		syncbb_state_move(0);
}

static void syncbb_stableclocks(unsigned int cycles)
{
	bool tms = tap_get_state() == TAP_RESET;
	unsigned int count;

	for (count = 0; count < cycles; ++count) {
		ch341a_jtag_write(1, tms, 0);
		ch341a_jtag_write(0, tms, 0);
	}
}

static void syncbb_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	unsigned long bit_base;
	int bit_cnt;

	if (!((!ir_scan && (tap_get_state() == TAP_DRSHIFT)) || (ir_scan && (tap_get_state() == TAP_IRSHIFT)))) {
		if (ir_scan)
			syncbb_end_state(TAP_IRSHIFT);
		else
			syncbb_end_state(TAP_DRSHIFT);

		syncbb_state_move(0);
		syncbb_end_state(saved_end_state);
	}

	bit_base = ch341a_port_buffer_curr;

	for (bit_cnt = 0; bit_cnt < scan_size; ++bit_cnt) {
		int bcval, bytec;
		bool tdi, tms;

		bcval = 1 << (bit_cnt % 8);
		bytec = bit_cnt / 8;
		tms = bit_cnt == scan_size - 1;
		tdi = (type != SCAN_IN) && (buffer[bytec] & bcval);

		ch341a_jtag_write(0, tms, tdi);
		ch341a_jtag_write(1, tms, tdi);
	}

	if (tap_get_state() != tap_get_end_state())
		syncbb_state_move(1);

	ch341a_port_buffer_flush(type != SCAN_OUT);

	if (type != SCAN_OUT) {
		for (bit_cnt = 0; bit_cnt < scan_size; ++bit_cnt) {
			int bcval, bytec;
			uint8_t value;

			bcval = 1 << (bit_cnt % 8);
			bytec = bit_cnt / 8;
			value = ch341a_port_buffer[bit_base + bit_cnt * 2 + 1];

			if (value & (1 << ch341a_tdo_gpio))
				buffer[bytec] |= bcval;
			else
				buffer[bytec] &= ~bcval;
		}
	}
}

static int ch341a_jtag_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	enum scan_type type;
	int scan_size;
	uint8_t *buffer;
	int retval = ERROR_OK;

	while (cmd) {
		switch (cmd->type) {
			case JTAG_RESET:
				LOG_DEBUG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				if (cmd->cmd.reset->trst == 1 ||
					(cmd->cmd.reset->srst &&
					(jtag_get_reset_config() & RESET_SRST_PULLS_TRST))) {
					tap_set_state(TAP_RESET);
				}
				ch341a_jtag_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_RUNTEST:
				LOG_DEBUG_IO("runtest %i cycles, end in %s", cmd->cmd.runtest->num_cycles,
					tap_state_name(cmd->cmd.runtest->end_state));
				syncbb_end_state(cmd->cmd.runtest->end_state);
				syncbb_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STABLECLOCKS:
				syncbb_stableclocks(cmd->cmd.stableclocks->num_cycles);
				break;

			case JTAG_TLR_RESET:
				LOG_DEBUG_IO("statemove end in %s", tap_state_name(cmd->cmd.statemove->end_state));
				syncbb_end_state(cmd->cmd.statemove->end_state);
				syncbb_state_move(0);
				break;

			case JTAG_PATHMOVE:
				LOG_DEBUG_IO("pathmove: %i states, end in %s", cmd->cmd.pathmove->num_states,
					tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
				syncbb_path_move(cmd->cmd.pathmove);
				break;

			case JTAG_SCAN:
				LOG_DEBUG_IO("%s scan end in %s",  (cmd->cmd.scan->ir_scan) ? "IR" : "DR",
					tap_state_name(cmd->cmd.scan->end_state));
				syncbb_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				syncbb_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				free(buffer);
				break;

			case JTAG_SLEEP:
				LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			case JTAG_TMS:
				syncbb_execute_tms(cmd);
				break;

			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}

		if (ch341a_port_buffer_curr)
			ch341a_port_buffer_flush(false);

		cmd = cmd->next;
	}

	return retval;
}

static int ch341a_init(void)
{
	uint16_t avids[] = {ch341a_vid, 0};
	uint16_t apids[] = {ch341a_pid, 0};
	uint8_t desc[0x12];

	if (jtag_libusb_open(avids, apids, &ch341a_adapter, NULL)) {
		const char *ch341a_serial_desc = adapter_get_required_serial();
		LOG_ERROR("ch341a not found: vid=%04x, pid=%04x, serial=%s\n",
			ch341a_vid, ch341a_pid, (!ch341a_serial_desc) ? "[any]" : ch341a_serial_desc);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (libusb_kernel_driver_active(ch341a_adapter, 0)) {
		if (libusb_detach_kernel_driver(ch341a_adapter, 0)) {
			LOG_ERROR("Failed to detach kernel driver");
			return ERROR_JTAG_INIT_FAILED;
		}
	}

	if (libusb_claim_interface(ch341a_adapter, 0)) {
		LOG_ERROR("Failed to claim interface 0");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (libusb_get_descriptor(ch341a_adapter, LIBUSB_DT_DEVICE, 0x00, desc, 0x12) < 0) {
		LOG_ERROR("Failed to get device descriptor");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (libusb_control_transfer(ch341a_adapter, CH341A_VENDOR_READ, CH341A_VENDOR_VERSION,
								0, 0, (void *)desc, 2, CH341A_BULK_TIMEOUT) < 0) {
		LOG_ERROR("Failed to get device version");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("ch341a chip version: 0x%02x%02x", desc[1], desc[0]);

	ch341a_port_buffer = malloc(ch341a_port_buffer_size);
	if (!ch341a_port_buffer) {
		LOG_ERROR("Unable to allocate memory for the buffer");
		return ERROR_JTAG_INIT_FAILED;
	}

	ch341a_port_direction((1 << ch341a_tdi_gpio) | (1 << ch341a_tck_gpio) |
						  (1 << ch341a_tms_gpio) | (1 << ch341a_trst_gpio) |
						  (1 << ch341a_srst_gpio));

	return ERROR_OK;
}

static int ch341a_quit(void)
{
	ch341a_port_direction(0);
	jtag_libusb_close(ch341a_adapter);

	free(ch341a_port_buffer);
	ch341a_port_buffer = NULL;
	ch341a_port_buffer_curr = 0;
	ch341a_port_buffer_size = 4096;

	return ERROR_OK;
}

static const struct command_registration ch341a_subcommand_handlers[] = {
	{
		.name = "vid_pid",
		.handler = ch341a_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "USB VID and PID of the adapter",
		.usage = "vid pid",
	}, {
		.name = "jtag_nums",
		.handler = ch341a_handle_jtag_nums_command,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdo, tdi, trst, srst.",
		.usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
	}, {
		.name = "tck_num",
		.handler = ch341a_handle_tck_num_command,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
	}, {
		.name = "tms_num",
		.handler = ch341a_handle_tms_num_command,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
	}, {
		.name = "tdo_num",
		.handler = ch341a_handle_tdo_num_command,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
	}, {
		.name = "tdi_num",
		.handler = ch341a_handle_tdi_num_command,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
	}, {
		.name = "trst_num",
		.handler = ch341a_handle_trst_num_command,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
	}, {
		.name = "srst_num",
		.handler = ch341a_handle_srst_num_command,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "<D0|D1|D2|D3|D4|D5|D6|D7>",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ch341a_command_handlers[] = {
	{
		.name = "ch341a",
		.mode = COMMAND_ANY,
		.help = "perform ch341a management",
		.chain = ch341a_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface ch341a_jtag_ops = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ch341a_jtag_execute_queue,
};

struct adapter_driver ch341a_adapter_driver = {
	.name = "ch341a",
	.transports = jtag_only,
	.commands = ch341a_command_handlers,

	.init = ch341a_init,
	.quit = ch341a_quit,

	.jtag_ops = &ch341a_jtag_ops,
};
