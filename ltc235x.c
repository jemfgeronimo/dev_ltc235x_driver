#include <linux/device.h>
#include <linux/io.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/pwm.h>

#include <iio.h>

#define LTC235X_NUM_CHANNELS	8
#define LTC235X_NUM_BITS 	18

// TODO
#define LTC235X_SOFTSPAN_ADDR(ch_id)	(0x0428 + (ch_id) * 0x40)

#define SAMP_FREQ_MAX // todo
#define SAMP_FREQ_MIN // todo
#define SAMP_FREQ_DEFAULT // todo

struct ltc235x_state {
	struct mutex		lock;
	struct pwm_device	cnv_pwm;
	unsigned int 		samp_freq;
	void __iomem		*regs;
};

static int ltc235x_read_raw (struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info) // todo
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		// this here needs the dma buffer ?? todo
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		// todo: return current samp freq
		break;
	default:
		return -EINVAL;
	}
}

static int __ltc235x_set_sampling_freq (const struct ltc2358_state *st, unsigned int freq)
{
	struct pwm_state cnv_state;
	int ret;

	// todo
	// check freq if inside range(SAMP_FREQ_MIN, SAMP_FREQ_MAX)
	// set period of cnv_state

	return pwm_apply_state(st->cnv_pwm, &cnv_state)
}

static int ltc235x_set_sampling_freq (struct iio_dev *indio_dev, unsigned int freq)
{
	const struct ltc235x_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = __ltc235x_set_sampling_freq(st, freq);
	iio_device_release_direct_mode(indio_dev);

	return ret
}

static void axi_ltc235x_write(struct ltc235x_state *st, unsigned int addr, unsigned int writeval)
{
	return iowrite32(writeval, st->regs + addr);
}

static void axi_ltc235x_read(struct ltc235x_state *st, unsigned int addr)
{
	return ioread32(st->regs + addr);
}

static int ltc235x_set_softspan (struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int softspan)
{
	struct ltc235x_state *st = iio_priv(indio_dev);
	int ch_id = chan.channel;

	if (ch_id < 0 || ch_id > 7)
		return // todo invalid channel id
	if (softspan < 0 || softspan > 7)
		return // todo: invalid softspan

	// write softspan to a channel depending on what channel id
	axi_ltc235x_write(st, LTC235X_SOFTSPAN_ADDR(ch_id), softspan); // todo: i need the register address of softspan here
	return 0
}

static int ltc235x_write_raw (struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info) // todo
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ltc235x_set_softspan(indio_dev, chan, val); // todo
	case IIO_CHAN_INFO_SAMP_FREQ:
		return = ltc235x_set_sampling_freq(indio_dev, val); // todo
	default:
		return -EINVAL;
	}
}

static const struct iio_info ltc235x_info = {
	.read_raw = ltc235x_read_raw, // todo
	.write_raw = ltc235x_write_raw, // todo
}

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int ltc235x_probe (struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct ltc235x_state *st;
	struct iio_buffer *buffer;
	
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->info = &ltc235x_info; // todo
	indio_dev->name = "ltc235x";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->num_channels = LTC235X_NUM_CHANNELS;
	indio_dev->channels = ltc235x_channels; // todo

	buffer = devm_iio_dmaengine_buffer_alloc(&pdev->dev, "rx", &dma_buffer_ops, indio_dev); // todo
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);
	iio_device_attach_buffer(indio_dev, buffer);

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	// todo
	// set default samp frequency
	// turn on pwm
	// enable channels / setup channels
	// load config and rst of ADC CORE / enable ADC CORE
	// set gpio: PD and CS_N

	mutex_init($st->lock)

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id ltc235x_of_match [] = {
	{ .compatible = "adi, ltc2358-18" },
	/*{ .compatible = "adi, ltc2358-16" },
	{ .compatible = "adi, ltc2357-18" },
	{ .compatible = "adi, ltc2357-16" },
	{ .compatible = "adi, ltc2353-18" },
	{ .compatible = "adi, ltc2353-16" },*/
	{}
};
MODULE_DEVICE_TABLE(of, ltc235x_of_match)

static struct platform_driver ltc235x_driver = {
	.driver = {
		.name = "ltc235x",
		.of_match_table = ltc235x_of_match,
	},
	.probe = ltc235x_probe,
}
module_platform_driver (ltc235x_driver);

MODULE_AUTHOR ("Jem Geronimo <Johnerasmusmari.Geronimo@analog.com");
MODULE_DESCRIPTION ("Analog Devices LTC235x ADC");
MODULE_LICENSE ("GPL v2");