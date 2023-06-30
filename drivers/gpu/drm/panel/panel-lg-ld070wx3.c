// SPDX-License-Identifier: GPL-2.0
/*
 * LG LD070WX3-SL01 DSI panel driver
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct lg_ld070wx3 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct regulator *vdd_supply;
	struct regulator *vcc_supply;

	bool prepared;
};

static inline struct lg_ld070wx3 *to_lg_ld070wx3(struct drm_panel *panel)
{
	return container_of(panel, struct lg_ld070wx3, panel);
}

#define dsi_generic_write_seq(dsi, cmd, seq...) do {			\
		static const u8 b[] = { cmd, seq };			\
		int ret;						\
		ret = mipi_dsi_dcs_write_buffer(dsi, b, ARRAY_SIZE(b));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)

static int lg_ld070wx3_prepare(struct drm_panel *panel)
{
	struct lg_ld070wx3 *priv = to_lg_ld070wx3(panel);
	struct device *dev = &priv->dsi->dev;
	int ret;

	if (priv->prepared)
		return 0;

	ret = regulator_enable(priv->vcc_supply);
	if (ret < 0) {
		dev_err(dev, "failed to enable vcc power supply\n");
		return ret;
	}

	/* Turn on DVDD (1.8V) and then AVDD after 20ms */
	msleep(20);

	ret = regulator_enable(priv->vdd_supply);
	if (ret < 0) {
		dev_err(dev, "failed to enable vdd power supply\n");
		return ret;
	}

	/*
	 * VDD to MIPI > 100ms based on the spec. Driver already
	 * take 50ms, so having 50ms delay here.
	 */
	msleep(50);

	priv->prepared = true;
	return 0;
}

static int lg_ld070wx3_enable(struct drm_panel *panel)
{
	struct lg_ld070wx3 *priv = to_lg_ld070wx3(panel);
	struct mipi_dsi_device *dsi = priv->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to soft reset panel: %d\n", ret);
		return ret;
	}

	msleep(20);

	/* Differential input impedance selection */
	dsi_generic_write_seq(dsi, 0xAE, 0x0B);

	/* Enter test mode 1 and 2*/
	dsi_generic_write_seq(dsi, 0xEE, 0xEA);
	dsi_generic_write_seq(dsi, 0xEF, 0x5F);

	/* Increased MIPI CLK driving ability */
	dsi_generic_write_seq(dsi, 0xF2, 0x68);

	/* Exit test mode 1 and 2 */
	dsi_generic_write_seq(dsi, 0xEE, 0x00);
	dsi_generic_write_seq(dsi, 0xEF, 0x00);

	return 0;
}

static int lg_ld070wx3_unprepare(struct drm_panel *panel)
{
	struct lg_ld070wx3 *priv = to_lg_ld070wx3(panel);

	if (!priv->prepared)
		return 0;

	/* MIPI off to VDD off needs to be 50~150ms per spec */
	msleep(50);

	regulator_disable(priv->vcc_supply);
	regulator_disable(priv->vdd_supply);

	/* LCD panel VDD on needs to be at least 1s after it's off */
	msleep(1000);

	priv->prepared = false;
	return 0;
}

static const struct drm_display_mode lg_ld070wx3_mode = {
	.clock = (800 + 32 + 48 + 8) * (1280 + 5 + 3 + 1) * 60 / 1000,
	.hdisplay = 800,
	.hsync_start = 800 + 32,
	.hsync_end = 800 + 32 + 48,
	.htotal = 800 + 32 + 48 + 8,
	.vdisplay = 1280,
	.vsync_start = 1280 + 5,
	.vsync_end = 1280 + 5 + 3,
	.vtotal = 1280 + 5 + 3 + 1,
	.width_mm = 94,
	.height_mm = 151,
};

static int lg_ld070wx3_get_modes(struct drm_panel *panel,
				   struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &lg_ld070wx3_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs lg_ld070wx3_panel_funcs = {
	.prepare = lg_ld070wx3_prepare,
	.enable = lg_ld070wx3_enable,
	.unprepare = lg_ld070wx3_unprepare,
	.get_modes = lg_ld070wx3_get_modes,
};

static int lg_ld070wx3_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct lg_ld070wx3 *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->vdd_supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(priv->vdd_supply))
		return PTR_ERR(priv->vdd_supply);

	priv->vcc_supply = devm_regulator_get(dev, "vcc");
	if (IS_ERR(priv->vcc_supply))
		return PTR_ERR(priv->vcc_supply);

	priv->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, priv);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_LPM;

	drm_panel_init(&priv->panel, dev, &lg_ld070wx3_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&priv->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&priv->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to attach to DSI host: %d\n", ret);
		drm_panel_remove(&priv->panel);
		return ret;
	}

	return 0;
}

static void lg_ld070wx3_remove(struct mipi_dsi_device *dsi)
{
	struct lg_ld070wx3 *priv = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev,
			"Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&priv->panel);
}

static const struct of_device_id lg_ld070wx3_of_match[] = {
	{ .compatible = "lg,ld070wx3-sl01" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lg_ld070wx3_of_match);

static struct mipi_dsi_driver lg_ld070wx3_driver = {
	.probe = lg_ld070wx3_probe,
	.remove = lg_ld070wx3_remove,
	.driver = {
		.name = "panel-lg-ld070wx3",
		.of_match_table = lg_ld070wx3_of_match,
	},
};
module_mipi_dsi_driver(lg_ld070wx3_driver);

MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("LG LD070WX3-SL01 DSI panel driver");
MODULE_LICENSE("GPL");
