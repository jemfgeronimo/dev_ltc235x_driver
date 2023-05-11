#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>

#include <iio.h>

#define LTC235X_NUM_CHANNELS	8
#define LTC235X_NUM_BITS 	18
#define LTC235X_SOFTSPAN_ADDR // todo

struct axi_ltc235x_state {
	struct mutex	lock;
};

static int axi_ltc235x_read_raw (struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info) // todo
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		break;

	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int axi_ltc235x_set_sampling_freq(struct iio_dev *indio_dev, unsigned int freq)
{ // todo
	struct axi_ltc235x_state *st = iio_priv(indio_dev);
	int ret;


	return ret
}

static int axi_ltc235x_write_raw (struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info) // todo
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = axi_ltc235x_set_softspan(indio_dev, val, val2); // todo
		break;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = axi_ltc235x_set_sampling_freq(indio_dev, val); // todo
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static const struct iio_info axi_ltc235x_info = {
	.read_raw = axi_ltc235x_read_raw, // todo
	.write_raw = axi_ltc235x_write_raw, // todo
}

static int axi_ltc235x_probe (struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct axi_ltc235x_state *st;
	
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	indio_dev->info = &axi_ltc235x_info; // todo
	indio_dev->name = "axi-ltc235x";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = LTC235X_NUM_CHANNELS;
	indio_dev->channels = axi_ltc235x_channels; // todo


	// todo
	// enable channels / setup channels

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id axi_ltc235x_of_match [] = {
	{ .compatible = "adi, ltc2358-18" },
	/*{ .compatible = "adi, ltc2358-16" },
	{ .compatible = "adi, ltc2357-18" },
	{ .compatible = "adi, ltc2357-16" },
	{ .compatible = "adi, ltc2353-18" },
	{ .compatible = "adi, ltc2353-16" },*/
	{}
};
MODULE_DEVICE_TABLE(of, axi_ltc235x_of_match)

static struct platform_driver ltc235x_driver = {
	.driver = {
		.name = "ltc235x",
		.of_match_table = axi_ltc235x_of_match,
	},
	.probe = axi_ltc235x_probe,
}
module_platform_driver (ltc235x_driver);

MODULE_AUTHOR ("Jem Geronimo <Johnerasmusmari.Geronimo@analog.com");
MODULE_DESCRIPTION ("Analog Devices LTC235x ADC");
MODULE_LICENSE ("GPL v2");