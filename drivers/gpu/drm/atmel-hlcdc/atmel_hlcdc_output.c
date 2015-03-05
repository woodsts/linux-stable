/*
 * Copyright (C) 2014 Traphandler
 * Copyright (C) 2014 Free Electrons
 * Copyright (C) 2014 Atmel
 *
 * Author: Jean-Jacques Hiblot <jjhiblot@traphandler.com>
 * Author: Boris BREZILLON <boris.brezillon@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/component.h>
#include <linux/of_graph.h>

#include <drm/drmP.h>
#include <drm/drm_panel.h>

#include "atmel_hlcdc_dc.h"

/**
 * Atmel HLCDC RGB output mode
 */
enum atmel_hlcdc_connector_rgb_mode {
	ATMEL_HLCDC_CONNECTOR_RGB444,
	ATMEL_HLCDC_CONNECTOR_RGB565,
	ATMEL_HLCDC_CONNECTOR_RGB666,
	ATMEL_HLCDC_CONNECTOR_RGB888,
};

/**
 * Atmel HLCDC Panel device structure
 *
 * This structure is specialization of the slave device structure to
 * interface with drm panels.
 *
 * @connector: DRM connector
 * @encoder: DRM encoder
 * @dpms: current DPMS mode
 * @panel: drm panel attached to this slave device
 */
struct atmel_hlcdc_panel {
	struct drm_connector connector;
	struct drm_encoder encoder;
	int dpms;
	struct drm_panel *panel;
};

static inline struct atmel_hlcdc_panel *
drm_connector_to_atmel_hlcdc_panel(struct drm_connector *connector)
{
	return container_of(connector, struct atmel_hlcdc_panel,
			    connector);
}

static inline struct atmel_hlcdc_panel *
drm_encoder_to_atmel_hlcdc_panel(struct drm_encoder *encoder)
{
	return container_of(encoder, struct atmel_hlcdc_panel, encoder);
}

static void atmel_hlcdc_panel_encoder_dpms(struct drm_encoder *encoder,
					   int mode)
{
	struct atmel_hlcdc_panel *panel =
			drm_encoder_to_atmel_hlcdc_panel(encoder);

	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;

	if (mode == panel->dpms)
		return;

	if (mode != DRM_MODE_DPMS_ON)
		drm_panel_disable(panel->panel);
	else
		drm_panel_enable(panel->panel);

	panel->dpms = mode;
}

static bool
atmel_hlcdc_panel_encoder_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted)
{
	return true;
}

static void atmel_hlcdc_panel_encoder_prepare(struct drm_encoder *encoder)
{
	atmel_hlcdc_panel_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void atmel_hlcdc_panel_encoder_commit(struct drm_encoder *encoder)
{
	atmel_hlcdc_panel_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void
atmel_hlcdc_panel_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted)
{

}

static struct drm_encoder_helper_funcs atmel_hlcdc_panel_encoder_helper_funcs = {
	.dpms = atmel_hlcdc_panel_encoder_dpms,
	.mode_fixup = atmel_hlcdc_panel_encoder_mode_fixup,
	.prepare = atmel_hlcdc_panel_encoder_prepare,
	.commit = atmel_hlcdc_panel_encoder_commit,
	.mode_set = atmel_hlcdc_panel_encoder_mode_set,
};

static void atmel_hlcdc_panel_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
	memset(encoder, 0, sizeof(*encoder));
}

static const struct drm_encoder_funcs atmel_hlcdc_panel_encoder_funcs = {
	.destroy = atmel_hlcdc_panel_encoder_destroy,
};

static int atmel_hlcdc_panel_get_modes(struct drm_connector *connector)
{
	struct atmel_hlcdc_panel *panel =
			drm_connector_to_atmel_hlcdc_panel(connector);

	return panel->panel->funcs->get_modes(panel->panel);
}

static struct drm_encoder *
atmel_hlcdc_panel_best_encoder(struct drm_connector *connector)
{
	struct atmel_hlcdc_panel *panel =
			drm_connector_to_atmel_hlcdc_panel(connector);

	return &panel->encoder;
}

static struct drm_connector_helper_funcs atmel_hlcdc_panel_connector_helper_funcs = {
	.get_modes = atmel_hlcdc_panel_get_modes,
	.best_encoder = atmel_hlcdc_panel_best_encoder,
};

static enum drm_connector_status
atmel_hlcdc_panel_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void
atmel_hlcdc_panel_connector_destroy(struct drm_connector *connector)
{
	struct atmel_hlcdc_panel *panel =
			drm_connector_to_atmel_hlcdc_panel(connector);

	drm_panel_detach(panel->panel);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs atmel_hlcdc_panel_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = atmel_hlcdc_panel_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = atmel_hlcdc_panel_connector_destroy,
};

static int atmel_hlcdc_create_panel_output(struct drm_device *dev,
					   struct of_endpoint *ep)
{
	struct device_node *np;
	struct drm_panel *p = NULL;
	struct atmel_hlcdc_panel *panel;
	int ret;

	np = of_graph_get_remote_port_parent(ep->local_node);
	if (!np)
		return -EINVAL;

	p = of_drm_find_panel(np);
	of_node_put(np);

	if (!p)
		return -EPROBE_DEFER;

	panel = devm_kzalloc(dev->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -EINVAL;

	panel->dpms = DRM_MODE_DPMS_OFF;

	drm_encoder_helper_add(&panel->encoder,
			       &atmel_hlcdc_panel_encoder_helper_funcs);
	ret = drm_encoder_init(dev, &panel->encoder,
			       &atmel_hlcdc_panel_encoder_funcs,
			       DRM_MODE_ENCODER_LVDS);
	if (ret)
		return ret;

	panel->connector.dpms = DRM_MODE_DPMS_OFF;
	panel->connector.polled = DRM_CONNECTOR_POLL_CONNECT;
	drm_connector_helper_add(&panel->connector,
				 &atmel_hlcdc_panel_connector_helper_funcs);
	ret = drm_connector_init(dev, &panel->connector,
				 &atmel_hlcdc_panel_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret)
		goto err_encoder_cleanup;

	drm_mode_connector_attach_encoder(&panel->connector,
					  &panel->encoder);
	panel->encoder.possible_crtcs = 0x1;

	drm_panel_attach(p, &panel->connector);
	panel->panel = p;

	return 0;

err_encoder_cleanup:
	drm_encoder_cleanup(&panel->encoder);

	return ret;
}

int atmel_hlcdc_output_set_rgb_mode(struct drm_device *dev)
{
	struct atmel_hlcdc_dc *dc = dev->dev_private;
	struct drm_connector *connector;
	unsigned int valid_modes = BIT(ATMEL_HLCDC_CONNECTOR_RGB444) |
				   BIT(ATMEL_HLCDC_CONNECTOR_RGB565) |
				   BIT(ATMEL_HLCDC_CONNECTOR_RGB666) |
				   BIT(ATMEL_HLCDC_CONNECTOR_RGB888);
	u32 cfg;

	/* Prepare the encoders and CRTCs before setting the mode. */
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		struct drm_display_info *info = &connector->display_info;
		unsigned int mask = 0;
		int i;

		for (i = 0; i < info->num_bus_formats; i++) {
			switch (info->bus_formats[i]) {
			case MEDIA_BUS_FMT_RGB444_1X12:
				mask |= BIT(ATMEL_HLCDC_CONNECTOR_RGB444);
				break;
			case MEDIA_BUS_FMT_RGB565_1X16:
				mask |= BIT(ATMEL_HLCDC_CONNECTOR_RGB565);
				break;
			case MEDIA_BUS_FMT_RGB666_1X18:
				mask |= BIT(ATMEL_HLCDC_CONNECTOR_RGB666);
				break;
			case MEDIA_BUS_FMT_RGB888_1X24:
				mask |= BIT(ATMEL_HLCDC_CONNECTOR_RGB888);
				break;
			default:
				break;
			}
		}

		valid_modes &= mask;
	}

	if (!valid_modes)
		return -EINVAL;

	/* TODO: choose best mode according to input format ? */
	cfg = (fls(valid_modes) - 1) << 8;

	regmap_update_bits(dc->hlcdc->regmap, ATMEL_HLCDC_CFG(5),
			   ATMEL_HLCDC_MODE_MASK, cfg);

	return 0;
}

static int atmel_hlcdc_create_panels(struct drm_device *dev)
{
	struct device_node *np = NULL, *remote = NULL;
	struct of_endpoint ep;
	int ret = 0;

	for (np = of_graph_get_next_endpoint(dev->dev->of_node, np);
	     np;
	     np = of_graph_get_next_endpoint(dev->dev->of_node, np)) {

		ret = of_graph_parse_endpoint(np, &ep);
		if (ret)
			break;

		remote = of_graph_get_remote_port_parent(ep.local_node);
		if (!remote) {
			ret = -EINVAL;
			break;
		}

		if (of_device_is_compatible(remote, "simple-panel") &&
		    of_device_is_available(remote)) {
			ret = atmel_hlcdc_create_panel_output(dev, &ep);
			if (ret)
				break;
		}

		of_node_put(remote);
	}

	of_node_put(np);

	return ret;
}

static void atmel_hlcdc_destroy_panels(struct drm_device *dev)
{

}

static int compare_of(struct device *dev, void *data)
{
	return dev->of_node == data;
}

bool atmel_hlcdc_output_panels_ready(struct device *dev)
{
	struct device_node *np = NULL, *remote = NULL;
	struct of_endpoint ep;
	bool ready = true;
	int ret = 0;

	for (np = of_graph_get_next_endpoint(dev->of_node, np);
	     np && ready;
	     np = of_graph_get_next_endpoint(dev->of_node, np)) {

		ret = of_graph_parse_endpoint(np, &ep);
		if (ret)
			break;

		remote = of_graph_get_remote_port_parent(ep.local_node);
		if (!remote) {
			ret = -EINVAL;
			break;
		}

		if (of_device_is_compatible(remote, "simple-panel") &&
		    of_device_is_available(remote)) {
			if (!of_drm_find_panel(remote))
				ready = false;
		}

		of_node_put(remote);
	}

	of_node_put(np);

	return ready;
}

int atmel_hlcdc_find_output_components(struct device *dev,
				       struct component_match **match)
{
	struct device_node *np = NULL, *remote = NULL;
	struct of_endpoint ep;
	int ret;

	for (np = of_graph_get_next_endpoint(dev->of_node, np);
	     np;
	     np = of_graph_get_next_endpoint(dev->of_node, np)) {

		ret = of_graph_parse_endpoint(np, &ep);
		if (ret)
			break;

		remote = of_graph_get_remote_port_parent(ep.local_node);
		if (!remote) {
			ret = -EINVAL;
			break;
		}

		if (!of_device_is_compatible(remote, "simple-panel"))
			component_match_add(dev, match, compare_of,
					    remote);
	}

	of_node_put(np);

	return ret;
}

int atmel_hlcdc_create_outputs(struct drm_device *ddev)
{
	return atmel_hlcdc_create_panels(ddev);
}

void atmel_hlcdc_destroy_outputs(struct drm_device *ddev)
{
	atmel_hlcdc_destroy_panels(ddev);
}


