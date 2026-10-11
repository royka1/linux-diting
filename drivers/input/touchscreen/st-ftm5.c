// SPDX-License-Identifier: GPL-2.0-only
/*
 * STMicroelectronics FingerTip FTM5 touchscreen controller, SPI interface
 *
 * Minimal driver: relies on the firmware already present in the controller's
 * flash, resets it, enables active scanning and reports multi-touch events
 * from the event FIFO. Protocol details follow the vendor fts_spi driver
 * (fts_lib) as used on the Xiaomi 12T Pro.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define FTM5_CMD_HW_REG_W		0xfa
#define FTM5_CMD_HW_REG_R		0xfb
#define FTM5_CMD_READ_FIFO		0x87
#define FTM5_CMD_SCAN_MODE		0xa0

#define FTM5_ADDR_CHIP_ID		0x20000000
#define FTM5_ADDR_FW_VER		0x20000004

#define FTM5_SCAN_MODE_ACTIVE		0x00

#define FTM5_EVENT_SIZE			8
#define FTM5_FIFO_DEPTH			32
#define FTM5_EVENTS_LEFT_MASK		0x1f

#define FTM5_EVT_NONE			0x00
#define FTM5_EVT_CONTROLLER_READY	0x03
#define FTM5_EVT_ENTER_POINT		0x13
#define FTM5_EVT_MOTION_POINT		0x23
#define FTM5_EVT_LEAVE_POINT		0x33
#define FTM5_EVT_ERROR			0xf3

#define FTM5_TOUCH_TYPE_HOVER		0x04

#define FTM5_MAX_SLOTS			10
#define FTM5_READY_TIMEOUT_MS		1000

struct ftm5 {
	struct spi_device *spi;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[2];
	/* DMA-safe transfer buffers */
	u8 tx[8];
	u8 rx[8 + FTM5_EVENT_SIZE * FTM5_FIFO_DEPTH] ____cacheline_aligned;
	u8 txbuf[8 + FTM5_EVENT_SIZE * FTM5_FIFO_DEPTH] ____cacheline_aligned;
};

static int ftm5_write(struct ftm5 *ts, const u8 *buf, size_t len)
{
	memcpy(ts->tx, buf, len);
	return spi_write(ts->spi, ts->tx, len);
}

/*
 * Reads return one dummy byte after the command. Use a single full-duplex
 * transfer so chip select stays asserted for command and data.
 */
static int ftm5_read(struct ftm5 *ts, size_t cmd_len, u8 **data, size_t len)
{
	struct spi_transfer xfer = {
		.tx_buf = ts->txbuf,
		.rx_buf = ts->rx,
		.len = cmd_len + 1 + len,
	};
	int ret;

	memset(ts->txbuf, 0, xfer.len);
	memcpy(ts->txbuf, ts->tx, cmd_len);
	ret = spi_sync_transfer(ts->spi, &xfer, 1);
	if (ret)
		return ret;

	*data = &ts->rx[cmd_len + 1];
	return 0;
}

static int ftm5_read_hw_reg(struct ftm5 *ts, u32 addr, u8 **data, size_t len)
{
	ts->tx[0] = FTM5_CMD_HW_REG_R;
	put_unaligned_be32(addr, &ts->tx[1]);
	return ftm5_read(ts, 5, data, len);
}

static int ftm5_read_fifo(struct ftm5 *ts, u8 **data, size_t events)
{
	ts->tx[0] = FTM5_CMD_READ_FIFO;
	return ftm5_read(ts, 1, data, events * FTM5_EVENT_SIZE);
}

static void ftm5_report_event(struct ftm5 *ts, const u8 *ev)
{
	unsigned int slot = ev[1] >> 4;
	unsigned int type = ev[1] & 0x0f;
	unsigned int x, y;
	bool contact;

	if (slot >= FTM5_MAX_SLOTS)
		return;

	switch (ev[0]) {
	case FTM5_EVT_ENTER_POINT:
	case FTM5_EVT_MOTION_POINT:
		/* 16-bit "super resolution" coordinates */
		x = get_unaligned_le16(&ev[2]);
		y = get_unaligned_le16(&ev[4]);
		contact = type != FTM5_TOUCH_TYPE_HOVER;
		input_mt_slot(ts->input, slot);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, contact);
		if (contact)
			touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		break;
	case FTM5_EVT_LEAVE_POINT:
		input_mt_slot(ts->input, slot);
		input_mt_report_slot_inactive(ts->input);
		break;
	case FTM5_EVT_ERROR:
		dev_dbg(&ts->spi->dev, "error event %8ph\n", ev);
		break;
	default:
		break;
	}
}

static irqreturn_t ftm5_irq(int irq, void *dev_id)
{
	struct ftm5 *ts = dev_id;
	unsigned int left, i;
	u8 *ev;

	if (ftm5_read_fifo(ts, &ev, 1))
		return IRQ_HANDLED;
	if (ev[0] == FTM5_EVT_NONE)
		return IRQ_HANDLED;

	ftm5_report_event(ts, ev);

	left = min_t(unsigned int, ev[7] & FTM5_EVENTS_LEFT_MASK,
		     FTM5_FIFO_DEPTH - 1);
	if (left && !ftm5_read_fifo(ts, &ev, left)) {
		for (i = 0; i < left; i++)
			ftm5_report_event(ts, &ev[i * FTM5_EVENT_SIZE]);
	}

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);

	return IRQ_HANDLED;
}

static int ftm5_wait_ready(struct ftm5 *ts)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(FTM5_READY_TIMEOUT_MS);
	u8 *ev;
	int ret;

	do {
		ret = ftm5_read_fifo(ts, &ev, 1);
		if (ret)
			return ret;
		if (ev[0] == FTM5_EVT_CONTROLLER_READY)
			return 0;
		usleep_range(5000, 6000);
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static int ftm5_power_on(struct ftm5 *ts)
{
	struct device *dev = &ts->spi->dev;
	static const u8 scan_on[] = { FTM5_CMD_SCAN_MODE, FTM5_SCAN_MODE_ACTIVE, 0x01 };
	u8 *id;
	int ret;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);

	/* Vendor order: vdd, 1 ms, avdd, then >= 16 ms before reset release */
	ret = regulator_enable(ts->supplies[0].consumer);
	if (ret)
		return ret;
	usleep_range(1000, 2000);
	ret = regulator_enable(ts->supplies[1].consumer);
	if (ret) {
		regulator_disable(ts->supplies[0].consumer);
		return ret;
	}
	usleep_range(16000, 17000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);

	ret = ftm5_wait_ready(ts);
	if (ret) {
		dev_err(dev, "controller not ready: %d\n", ret);
		goto err_off;
	}

	ret = ftm5_read_hw_reg(ts, FTM5_ADDR_CHIP_ID, &id, 6);
	if (!ret)
		dev_info(dev, "chip id %02x%02x, fw %02x%02x\n",
			 id[0], id[1], id[5], id[4]);

	ret = ftm5_write(ts, scan_on, sizeof(scan_on));
	if (ret) {
		dev_err(dev, "failed to enable scanning: %d\n", ret);
		goto err_off;
	}

	return 0;

err_off:
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
	return ret;
}

static void ftm5_power_off(void *data)
{
	struct ftm5 *ts = data;

	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	regulator_bulk_disable(ARRAY_SIZE(ts->supplies), ts->supplies);
}

static int ftm5_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ftm5 *ts;
	int ret;

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->spi = spi;
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return ret;

	ts->supplies[0].supply = "vdd";
	ts->supplies[1].supply = "avdd";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ts->supplies), ts->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio),
				     "failed to get reset gpio\n");

	ts->input = devm_input_allocate_device(dev);
	if (!ts->input)
		return -ENOMEM;

	ts->input->name = "STM FTM5 Touchscreen";
	ts->input->id.bustype = BUS_SPI;
	input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input, EV_ABS, ABS_MT_POSITION_Y);
	touchscreen_parse_properties(ts->input, true, &ts->prop);
	if (!ts->prop.max_x || !ts->prop.max_y)
		return dev_err_probe(dev, -EINVAL,
				     "touchscreen-size-x/y are required\n");

	ret = input_mt_init_slots(ts->input, FTM5_MAX_SLOTS,
				  INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ret = ftm5_power_on(ts);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, ftm5_power_off, ts);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ftm5_irq,
					IRQF_ONESHOT, "st-ftm5", ts);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq\n");

	return input_register_device(ts->input);
}

static const struct of_device_id ftm5_of_match[] = {
	{ .compatible = "st,ftm5-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, ftm5_of_match);

static const struct spi_device_id ftm5_spi_ids[] = {
	{ "ftm5-spi" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ftm5_spi_ids);

static struct spi_driver ftm5_driver = {
	.driver = {
		.name = "st-ftm5",
		.of_match_table = ftm5_of_match,
	},
	.id_table = ftm5_spi_ids,
	.probe = ftm5_probe,
};
module_spi_driver(ftm5_driver);

MODULE_DESCRIPTION("STMicroelectronics FTM5 SPI touchscreen driver");
MODULE_LICENSE("GPL");
