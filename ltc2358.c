// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
/*
 * Analog Devices LTC2358 ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 * Authors: Kim Seer Paller <kimseer.paller@analog.com>
 * 	    Johnerasmusmari Geronimo <johnerasmusmari.geronimo@analog.com>
 */
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/bitfield.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/pwm.h>
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/types.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			(1 << 0)
#define ADI_MMCM_RSTN 			(1 << 1)

#define LTC2358_REG_CHAN_RAW_DATA(c)	(0x0408 + (c) * 0x40)
#define LTC2358_SOFTSPAN_ADDR(c)	(0x0428 + (c) * 0x40)
#define LTC2358_RESET_MASK		GENMASK(1, 0)

#define LTC2358_18_BIT_RESOLUTION	18
#define LTC2358_16_BIT_RESOLUTION	16

#define ADI_AXI_REG_CHAN_CTRL(c)		(0x0400 + (c) * 0x40)

// in Hz
#define SAMP_FREQ_MAX // todo
#define SAMP_FREQ_MIN // todo
#define SAMP_FREQ_DEFAULT 125000

enum ltc2358_supported_device_ids {
	ID_LTC2358_16,
	ID_LTC2358_18,
};

enum ltc2358_input_range {
	LTC2358_OFF,
	LTC2358_0V_1_5V_VREF,
	LTC2358_1_25V_VREF_1_024,
	LTC2358_1_25_VREF,
	LTC2358_0V_2_5V_VREF_1_024,
	LTC2358_0V_2_5V_VREF,
	LTC2358_2_5V_VREF_1_024,
	LTC2358_2_5V_VREF
};

struct ltc2358_state {
	struct iio_dev		*indio_dev;
	/* protect against concurrent access to the device */
	struct mutex		lock;
	void __iomem		*regs;
	unsigned int		regs_size;
	struct pwm_device	*cnv_pwm;
	struct gpio_desc	*pd_gpio;
	struct gpio_desc	*csn_gpio;
	int 				vref_mv;
	int					softspan[8];
	unsigned int		samp_freq;
};

static void ltc2358_read(struct ltc2358_state *st, unsigned int chan,
				unsigned int *data)
{
	*data = ioread32(st->regs + LTC2358_REG_CHAN_RAW_DATA(chan));
}

static int ltc2358_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct ltc2358_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ltc2358_read(st, chan->channel, val);
		mutex_unlock(&st->lock);

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv * 2;
		*val2 = 18;

		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		*val = st->samp_freq;
		mutex_unlock(&st->lock);
		return 0; // todo: use appropriate return value

	default:
		return -EINVAL;
	}
}

static const char * const ltc2358_input_range[] = {
	[LTC2358_OFF] = "off",
	[LTC2358_0V_1_5V_VREF] = "0V-1.25V*Vref",
	[LTC2358_1_25V_VREF_1_024] = "1.25V*Vref/1.024",
	[LTC2358_1_25_VREF] = "1.25V*Vref",
	[LTC2358_0V_2_5V_VREF_1_024] = "0V-2.5V*Vref",
	[LTC2358_0V_2_5V_VREF] = "0V-2.5V*Vref",
	[LTC2358_2_5V_VREF_1_024] = "2.5V*Vref/1.024",
	[LTC2358_2_5V_VREF] = "2.5V*Vref",
};

static int ltc2358_set_softspan(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int val)
{
	struct ltc2358_state *st = iio_priv(indio_dev);
	if (val > ARRAY_SIZE(ltc2358_input_range))
		return -EINVAL;

	st->softspan[chan->channel] = val;

	iowrite32(val, st->regs + LTC2358_SOFTSPAN_ADDR(chan->channel));

	return 0;
}

static int ltc2358_get_softspan(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan)
{
	struct ltc2358_state *st = iio_priv(indio_dev);

	return st->softspan[chan->channel];
}

static int _ltc2358_set_sampling_freq (struct ltc2358_state *st, unsigned int freq)
{
	struct pwm_state cnv_state;
	int ret;

	// TODO: frequency error checking; set range of min and max samp freq
	pwm_get_state(st->cnv_pwm, &cnv_state);
	cnv_state.duty_cycle = 60;
	cnv_state.period = DIV_ROUND_CLOSEST_ULL(NANO, freq);
	cnv_state.time_unit = PWM_UNIT_NSEC;

	ret = pwm_apply_state(st->cnv_pwm, &cnv_state);
	if (ret)
		return ret;

	st->samp_freq = freq;

	return 0;
}

static int ltc2358_set_sampling_freq (struct iio_dev *indio_dev, unsigned int freq)
{
	struct ltc2358_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = _ltc2358_set_sampling_freq(st, freq);
	mutex_unlock(&st->lock);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ltc2358_write_raw (struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ltc2358_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ltc2358_info = {
	.read_raw = ltc2358_read_raw,
	.write_raw = ltc2358_write_raw,
};

static const struct iio_enum ltc2358_softspan = {
	.items = ltc2358_input_range,
	.num_items = ARRAY_SIZE(ltc2358_input_range),
	.set = ltc2358_set_softspan,
	.get = ltc2358_get_softspan,
};

static const struct iio_chan_spec_ext_info ltc2358_ext_info[] = {
	IIO_ENUM("input_range", IIO_SEPARATE, &ltc2358_softspan),
	IIO_ENUM_AVAILABLE_SHARED("input_range", IIO_SHARED_BY_ALL,
						&ltc2358_softspan),
	{ },
};

#define LTC2358_CHAN(_chan) { 									\
	.type = IIO_VOLTAGE, 										\
	.indexed = 1, 												\
	.channel = _chan, 											\
	.info_mask_separate = 	BIT(IIO_CHAN_INFO_RAW) | 			\
							BIT(IIO_CHAN_INFO_SCALE), 			\
	.info_mask_shared_by_type = 								\
							BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.ext_info = ltc2358_ext_info, 								\
}

static const struct iio_chan_spec ltc2358_channels[] = {
	LTC2358_CHAN(0),
	LTC2358_CHAN(1),
	LTC2358_CHAN(2),
	LTC2358_CHAN(3),
	LTC2358_CHAN(4),
	LTC2358_CHAN(5),
	LTC2358_CHAN(6),
	LTC2358_CHAN(7),
};

static void ltc2358_regulator_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static int ltc2358_probe(struct platform_device *pdev)
{
	struct ltc2358_state *st;
	struct iio_dev *indio_dev;
	struct regulator *vref;
	struct resource *mem;
	struct device *dev = &pdev->dev;
	int ret;
	int i;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs_size = resource_size(mem);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	platform_set_drvdata(pdev, indio_dev);

	for (i=0; i<ARRAY_SIZE(ltc2358_channels); i++){
		st->softspan[i] = LTC2358_0V_1_5V_VREF;
		iowrite32(st->softspan[i], st->regs + LTC2358_SOFTSPAN_ADDR(i));
	}

	/* Reset AXI ADC Core */
	iowrite32(0, st->regs + ADI_REG_RSTN);
	iowrite32(ADI_RSTN | ADI_MMCM_RSTN, st->regs + ADI_REG_RSTN);

	/* setup gpios: PD and CS */
	st->pd_gpio = devm_gpiod_get_optional(&pdev->dev, "pd", GPIOD_OUT_LOW);
	if (IS_ERR(st->pd_gpio))
		return PTR_ERR(st->pd_gpio);
	printk("Hello pd\n");
	st->csn_gpio = devm_gpiod_get_optional(&pdev->dev, "csn", GPIOD_OUT_LOW);
	if (IS_ERR(st->csn_gpio))
		return PTR_ERR(st->csn_gpio);
	printk("Hello csn\n");

	/* setup pwm gen */
	st->samp_freq = SAMP_FREQ_DEFAULT;
	st->cnv_pwm = devm_pwm_get(dev, "axi_pwm_gen");
	if (IS_ERR(st->cnv_pwm))
		return dev_err_probe(dev, PTR_ERR(st->cnv_pwm), "Failed to find PWM GEN\n");
	ret = _ltc2358_set_sampling_freq(st, SAMP_FREQ_DEFAULT);
	if (ret){
		return ret; // TODO: use appropriate error handling
	}
	ret = pwm_enable(st->cnv_pwm);
	if (ret)
		return ret; // TODO: use appropriate error handling
	printk("Hello: pwm should be enabled\n");

	/* Enable ADC Core Channels */ // TODO
	for (i=0; i<ARRAY_SIZE(ltc2358_channels); i++)
		iowrite32(1, st->regs + ADI_AXI_REG_CHAN_CTRL(i));

	vref = devm_regulator_get_optional(&pdev->dev, "vref");
	if (IS_ERR(vref)) {
		if (PTR_ERR(vref) != -ENODEV)
			return dev_err_probe(&pdev->dev, PTR_ERR(vref),
					     "Failed to get vref regulator\n");

		/* internal reference */
		st->vref_mv = 2048;
	} else {
		ret = regulator_enable(vref);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "Failed to enable vref regulator\n");

		ret = devm_add_action_or_reset(&pdev->dev,
					       ltc2358_regulator_disable, vref);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vref);
		if (ret < 0)
			return dev_err_probe(&pdev->dev, ret,
					     "Failed to get vref\n");

		if (ret < 1250000 || ret > 2200000)
			return dev_err_probe(&pdev->dev, -EINVAL,
					     "Invalid vref voltage\n");

		st->vref_mv = (ret * 2) / 1000;
	}

	mutex_init(&st->lock);

	indio_dev->name = "ltc2358";
	indio_dev->info = &ltc2358_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ltc2358_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltc2358_channels);

	return devm_iio_device_register(&pdev->dev, indio_dev);
}

static const struct of_device_id ltc2358_of_match [] = {
	{ .compatible = "adi,ltc2358" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2358_of_match);

static struct platform_driver ltc2358_driver = {
	.driver = {
		.name = "ltc2358",
		.of_match_table = ltc2358_of_match,
	},
	.probe = ltc2358_probe,
};
module_platform_driver(ltc2358_driver);

MODULE_AUTHOR ("Kim Seer Paller <kimseer.paller@nalog.com>");
MODULE_DESCRIPTION ("Analog Devices LTC2358 ADC");
MODULE_LICENSE ("GPL v2");
