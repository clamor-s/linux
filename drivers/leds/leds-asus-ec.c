// SPDX-License-Identifier: GPL-2.0-only
/*
 * ASUS EC driver - battery LED
 */

#include <linux/leds.h>
#include <linux/mfd/asus-ec.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>

/*
 * F[5] & 0x07
 *  auto: brightness == 0
 *  bit 0: blink / charger on
 *  bit 1: amber on
 *  bit 2: green on
 */

#define ASUSEC_CTL_LED_BLINK		BIT_ULL(40)
#define ASUSEC_CTL_LED_AMBER		BIT_ULL(41)
#define ASUSEC_CTL_LED_GREEN		BIT_ULL(42)

static void asusec_led_set_brightness_amber(struct led_classdev *led,
					    enum led_brightness brightness)
{
	const struct asusec_info *ec =
				dev_get_drvdata(led->dev->parent);

	if (brightness)
		asusec_set_ctl_bits(ec, ASUSEC_CTL_LED_AMBER);
	else
		asusec_clear_ctl_bits(ec, ASUSEC_CTL_LED_AMBER);
}

static void asusec_led_set_brightness_green(struct led_classdev *led,
					    enum led_brightness brightness)
{
	const struct asusec_info *ec =
				dev_get_drvdata(led->dev->parent);

	if (brightness)
		asusec_set_ctl_bits(ec, ASUSEC_CTL_LED_GREEN);
	else
		asusec_clear_ctl_bits(ec, ASUSEC_CTL_LED_GREEN);
}

static int asusec_led_probe(struct platform_device *pdev)
{
	struct asusec_info *ec = asusec_cell_to_ec(pdev);
	struct led_classdev *amber_led, *green_led;
	int ret;

	platform_set_drvdata(pdev, ec);

	amber_led = devm_kzalloc(&pdev->dev, sizeof(*amber_led), GFP_KERNEL);
	if (!amber_led)
		return -ENOMEM;

	amber_led->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s::amber", ec->name);
	amber_led->max_brightness = 1;
	amber_led->flags = LED_CORE_SUSPENDRESUME | LED_RETAIN_AT_SHUTDOWN;
	amber_led->brightness_set = asusec_led_set_brightness_amber;

	ret = devm_led_classdev_register(&pdev->dev, amber_led);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register amber LED\n");

	green_led = devm_kzalloc(&pdev->dev, sizeof(*green_led), GFP_KERNEL);
	if (!green_led)
		return -ENOMEM;

	green_led->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s::green", ec->name);
	green_led->max_brightness = 1;
	green_led->flags = LED_CORE_SUSPENDRESUME | LED_RETAIN_AT_SHUTDOWN;
	green_led->brightness_set = asusec_led_set_brightness_green;

	ret = devm_led_classdev_register(&pdev->dev, green_led);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register green LED\n");

	return 0;
}

static const struct of_device_id asusec_led_ids[] = {
	{ .compatible = "asus,ec-led" },
	{ }
};
MODULE_DEVICE_TABLE(of, asusec_led_ids);

static struct platform_driver asusec_led_driver = {
	.driver.name = "asusec-led",
	.driver.of_match_table = of_match_ptr(asusec_led_ids),
	.probe = asusec_led_probe,
};
module_platform_driver(asusec_led_driver);

MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("ASUS Transformer's charging LED driver");
MODULE_LICENSE("GPL");
