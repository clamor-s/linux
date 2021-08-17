// SPDX-License-Identifier: GPL-2.0-only
/*
 * Hydis HV101HD1 panel driver
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct hv101hd1 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct regulator *vdd_supply;
	struct gpio_desc *enable_gpio;

	bool prepared;
};

static inline struct hv101hd1 *to_hv101hd1(struct drm_panel *panel)
{
	return container_of(panel, struct hv101hd1, panel);
}

static int hv101hd1_prepare(struct drm_panel *panel)
{
	struct hv101hd1 *hv = to_hv101hd1(panel);
	struct device *dev = &hv->dsi->dev;
	int ret;

	if (hv->prepared)
		return 0;

	ret = regulator_enable(hv->vdd_supply);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulator: %d\n", ret);
		return ret;
	}

	gpiod_set_value_cansleep(hv->enable_gpio, 1);

	hv->prepared = true;

	return 0;
}

static int hv101hd1_enable(struct drm_panel *panel)
{
	struct hv101hd1 *hv = to_hv101hd1(panel);
	struct mipi_dsi_device *dsi = hv->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	msleep(20);

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display on: %d\n", ret);
		return ret;
	}

	msleep(20);

	return 0;
}

static int hv101hd1_disable(struct drm_panel *panel)
{
	struct hv101hd1 *hv = to_hv101hd1(panel);
	struct mipi_dsi_device *dsi = hv->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display off: %d\n", ret);
		return ret;
	}

	msleep(120);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode: %d\n", ret);
		return ret;
	}

	msleep(20);

	return 0;
}

static int hv101hd1_unprepare(struct drm_panel *panel)
{
	struct hv101hd1 *hv = to_hv101hd1(panel);
	struct device *dev = &hv->dsi->dev;
	int ret;

	if (!hv->prepared)
		return 0;

	gpiod_set_value_cansleep(hv->enable_gpio, 0);
	regulator_disable(hv->vdd_supply);

	hv->prepared = false;

	return 0;
}

static const struct drm_display_mode hv101hd1_mode = {
	.clock = (1366 + 74 + 36 + 24) * (768 + 21 + 7 + 4) * 60 / 1000,

	.hdisplay = 1366,
	.hsync_start = 1366 + 74,
	.hsync_end = 1366 + 74 + 36,
	.htotal = 1366 + 74 + 36 + 24,

	.vdisplay = 768,
	.vsync_start = 768 + 21,
	.vsync_end = 768 + 21 + 7,
	.vtotal = 768 + 21 + 7 + 4,

	.width_mm = 140,
	.height_mm = 220,
};

static int hv101hd1_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &hv101hd1_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs hv101hd1_panel_funcs = {
	.prepare = hv101hd1_prepare,
	.enable = hv101hd1_enable,
	.disable = hv101hd1_disable,
	.unprepare = hv101hd1_unprepare,
	.get_modes = hv101hd1_get_modes,
};

static int hv101hd1_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct hv101hd1 *hv;
	int ret;

	hv = devm_kzalloc(dev, sizeof(*hv), GFP_KERNEL);
	if (!hv)
		return -ENOMEM;

	hv->vdd_supply = devm_regulator_get(dev, "power");
	if (IS_ERR(hv->vdd_supply))
		return dev_err_probe(dev, PTR_ERR(hv->vdd_supply),
				     "Failed to get power regulator\n");

	hv->enable_gpio = devm_gpiod_get(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(hv->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(hv->enable_gpio),
				     "Failed to get enable-gpios\n");

	hv->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, hv);

	dsi->lanes = 2;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	drm_panel_init(&hv->panel, dev, &hv101hd1_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&hv->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&hv->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&hv->panel);
		return ret;
	}

	return 0;
}

static void hv101hd1_remove(struct mipi_dsi_device *dsi)
{
	struct hv101hd1 *hv = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev,
			"Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&hv->panel);
}

static const struct of_device_id hv101hd1_of_match[] = {
	{ .compatible = "hydis,hv101hd1" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hv101hd1_of_match);

static struct mipi_dsi_driver hv101hd1_driver = {
	.driver = {
		.name = "panel-hv101hd1",
		.of_match_table = hv101hd1_of_match,
	},
	.probe = hv101hd1_probe,
	.remove = hv101hd1_remove,
};
module_mipi_dsi_driver(hv101hd1_driver);

MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("DRM driver for Hydis HV101HD1 panel");
MODULE_LICENSE("GPL v2");
