// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 MediaTek Inc.
 */

#include <linux/extcon-provider.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

#include "extcon-mtk-usb.h"

#ifdef CONFIG_TCPC_CLASS
#include "tcpm.h"
#endif

static const unsigned int usb_extcon_cable[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

#ifdef CONFIG_LGE_USB
static const char *const dual_role_dr[] = {
	"host",
	"device",
	"none",
};

static const char *const typec_state[] = {
	"unattached",
	"attach_snk",
	"attach_src",
	"attach_audio",
	"attach_debug",
	"attach_dbgacc_snk",
	"attach_custom_src",
	"attach_norp_src",
};

static const char *const dr_swap_state[] = {
	"ufp",
	"dfp",
	"none",
};
#endif

static void mtk_usb_extcon_update_role(struct work_struct *work)
{
	struct usb_role_info *role = container_of(to_delayed_work(work),
					struct usb_role_info, dwork);
	struct mtk_extcon_info *extcon = role->extcon;
	unsigned int cur_dr, new_dr;

	cur_dr = extcon->c_role;
	new_dr = role->d_role;

	#ifdef CONFIG_LGE_USB
	dev_info(extcon->dev, "%s(): dr cur %s -> new %s\n", __func__,
			dual_role_dr[cur_dr], dual_role_dr[new_dr]);
	#else
	dev_info(extcon->dev, "cur_dr(%d) new_dr(%d)\n", cur_dr, new_dr);
	#endif

	/* none -> device */
	if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, true);
	/* none -> host */
	} else if (cur_dr == DUAL_PROP_DR_NONE &&
			new_dr == DUAL_PROP_DR_HOST) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, true);
	/* device -> none */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, false);
	/* host -> none */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_NONE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, false);
	/* device -> host */
	} else if (cur_dr == DUAL_PROP_DR_DEVICE &&
			new_dr == DUAL_PROP_DR_HOST) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB, false);
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, true);
	/* host -> device */
	} else if (cur_dr == DUAL_PROP_DR_HOST &&
			new_dr == DUAL_PROP_DR_DEVICE) {
		extcon_set_state_sync(extcon->edev, EXTCON_USB_HOST, false);
		extcon_set_state_sync(extcon->edev, EXTCON_USB, true);
	}

	/* usb role switch */
	if (extcon->role_sw) {
		if (new_dr == DUAL_PROP_DR_DEVICE)
			usb_role_switch_set_role(extcon->role_sw,
						USB_ROLE_DEVICE);
		else if (new_dr == DUAL_PROP_DR_HOST)
			usb_role_switch_set_role(extcon->role_sw,
						USB_ROLE_HOST);
		else
			usb_role_switch_set_role(extcon->role_sw,
						USB_ROLE_NONE);
	}

	extcon->c_role = new_dr;
	kfree(role);
}

static int mtk_usb_extcon_set_role(struct mtk_extcon_info *extcon,
						unsigned int role)
{
	struct usb_role_info *role_info;

	/* create and prepare worker */
	role_info = kzalloc(sizeof(*role_info), GFP_KERNEL);
	if (!role_info)
		return -ENOMEM;

	INIT_DELAYED_WORK(&role_info->dwork, mtk_usb_extcon_update_role);

	role_info->extcon = extcon;
	role_info->d_role = role;
	/* issue connection work */
	queue_delayed_work(extcon->extcon_wq, &role_info->dwork, 0);

	return 0;
}

static int mtk_usb_extcon_psy_notifier(struct notifier_block *nb,
				unsigned long event, void *data)
{
	struct power_supply *psy = data;
	struct mtk_extcon_info *extcon = container_of(nb,
					struct mtk_extcon_info, psy_nb);
	union power_supply_propval pval;
	union power_supply_propval ival;
	union power_supply_propval tval;
	int ret;

	if (event != PSY_EVENT_PROP_CHANGED || psy != extcon->usb_psy)
		return NOTIFY_DONE;

	ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get online prop\n");
		return NOTIFY_DONE;
	}

	ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_AUTHENTIC, &ival);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get authentic prop\n");
		ival.intval = 0;
	}

	ret = power_supply_get_property(psy,
				POWER_SUPPLY_PROP_TYPE, &tval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get usb type\n");
		return NOTIFY_DONE;
	}

	dev_info(extcon->dev, "online=%d, ignore_usb=%d, type=%d\n",
				pval.intval, ival.intval, tval.intval);

	if (ival.intval)
		return NOTIFY_DONE;

	if (pval.intval && (tval.intval == POWER_SUPPLY_TYPE_USB ||
			tval.intval == POWER_SUPPLY_TYPE_USB_CDP))
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
	else
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);

	return NOTIFY_DONE;
}

static int mtk_usb_extcon_psy_init(struct mtk_extcon_info *extcon)
{
	int ret = 0;
	struct device *dev = extcon->dev;
	union power_supply_propval pval;
	union power_supply_propval ival;
	union power_supply_propval tval;

	extcon->usb_psy = devm_power_supply_get_by_phandle(dev, "charger");
	if (IS_ERR_OR_NULL(extcon->usb_psy)) {
		dev_err(dev, "fail to get usb_psy\n");
		extcon->usb_psy = NULL;
		return -EINVAL;
	}

	extcon->psy_nb.notifier_call = mtk_usb_extcon_psy_notifier;
	ret = power_supply_reg_notifier(&extcon->psy_nb);
	if (ret) {
		dev_err(dev, "fail to register notifer\n");
		return ret;
	}

	ret = power_supply_get_property(extcon->usb_psy,
				POWER_SUPPLY_PROP_ONLINE, &pval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get online prop\n");
		return 0;
	}

	ret = power_supply_get_property(extcon->usb_psy,
				POWER_SUPPLY_PROP_AUTHENTIC, &ival);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get authentic prop\n");
		ival.intval = 0;
	}

	ret = power_supply_get_property(extcon->usb_psy,
				POWER_SUPPLY_PROP_USB_TYPE, &tval);
	if (ret < 0) {
		dev_info(extcon->dev, "failed to get usb type\n");
		return 0;
	}

	dev_info(extcon->dev, "online=%d, ignore_usb=%d, type=%d\n",
				pval.intval, ival.intval, tval.intval);

	if (ival.intval)
		return 0;

	if (pval.intval && (tval.intval == POWER_SUPPLY_USB_TYPE_SDP ||
			tval.intval == POWER_SUPPLY_USB_TYPE_CDP))
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);

	return 0;
}

static int mtk_usb_extcon_set_vbus(struct mtk_extcon_info *extcon,
							bool is_on)
{
	struct regulator *vbus = extcon->vbus;
	struct device *dev = extcon->dev;
	int ret;

	/* vbus is optional */
	if (!vbus || extcon->vbus_on == is_on)
		return 0;

	dev_info(dev, "vbus turn %s\n", is_on ? "on" : "off");

	if (is_on) {
		if (extcon->vbus_vol) {
			ret = regulator_set_voltage(vbus,
					extcon->vbus_vol, extcon->vbus_vol);
			if (ret) {
				dev_err(dev, "vbus regulator set voltage failed\n");
				return ret;
			}
		}

		if (extcon->vbus_cur) {
			ret = regulator_set_current_limit(vbus,
					extcon->vbus_cur, extcon->vbus_cur);
			if (ret) {
				dev_err(dev, "vbus regulator set current failed\n");
				return ret;
			}
		}

		ret = regulator_enable(vbus);
		if (ret) {
			dev_err(dev, "vbus regulator enable failed\n");
			return ret;
		}
	} else {
		regulator_disable(vbus);
	}

	extcon->vbus_on = is_on;

	return 0;
}

#ifdef CONFIG_TCPC_CLASS
static int mtk_extcon_tcpc_notifier(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct mtk_extcon_info *extcon =
			container_of(nb, struct mtk_extcon_info, tcpc_nb);
	struct device *dev = extcon->dev;
	bool vbus_on;

	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		dev_info(dev, "source vbus = %dmv\n",
				 noti->vbus_state.mv);
		vbus_on = (noti->vbus_state.mv) ? true : false;
		mtk_usb_extcon_set_vbus(extcon, vbus_on);
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		#ifdef CONFIG_LGE_USB_TYPE_C
		dev_info(dev, "%s() typec old %s -> new %s\n", __func__,
				typec_state[noti->typec_state.old_state],
				typec_state[noti->typec_state.new_state]);
		#else
		dev_info(dev, "old_state=%d, new_state=%d\n",
				noti->typec_state.old_state,
				noti->typec_state.new_state);
		#endif
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			dev_info(dev, "Type-C SRC plug in\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
		} else if (!(extcon->bypss_typec_sink) &&
			noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			dev_info(dev, "Type-C SINK plug in\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SRC ||
			noti->typec_state.old_state == TYPEC_ATTACHED_SNK) &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			dev_info(dev, "Type-C plug out\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
		}
		break;
	case TCP_NOTIFY_DR_SWAP:
		#ifdef CONFIG_LGE_USB_TYPE_C
		dev_info(dev, "%s() dr_swap, new role: %s\n",
				__func__, dr_swap_state[noti->swap_state.new_role]);
		#else
		dev_info(dev, "%s dr_swap, new role=%d\n",
				__func__, noti->swap_state.new_role);
		#endif
		if (noti->swap_state.new_role == PD_ROLE_UFP &&
				extcon->c_role == DUAL_PROP_DR_HOST) {
			dev_info(dev, "switch role to device\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_DEVICE);
		} else if (noti->swap_state.new_role == PD_ROLE_DFP &&
				extcon->c_role == DUAL_PROP_DR_DEVICE) {
			dev_info(dev, "switch role to host\n");
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
			mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
		}
		break;
	}

	return NOTIFY_OK;
}

static int mtk_usb_extcon_tcpc_init(struct mtk_extcon_info *extcon)
{
	struct tcpc_device *tcpc_dev;
	int ret;

	tcpc_dev = tcpc_dev_get_by_name("type_c_port0");

	if (!tcpc_dev) {
		dev_err(extcon->dev, "get tcpc device fail\n");
		return -ENODEV;
	}

	extcon->tcpc_nb.notifier_call = mtk_extcon_tcpc_notifier;
	ret = register_tcp_dev_notifier(tcpc_dev, &extcon->tcpc_nb,
		TCP_NOTIFY_TYPE_USB | TCP_NOTIFY_TYPE_VBUS |
		TCP_NOTIFY_TYPE_MISC);
	if (ret < 0) {
		dev_err(extcon->dev, "register notifer fail\n");
		return -EINVAL;
	}

	extcon->tcpc_dev = tcpc_dev;

	return 0;
}
#endif

static void mtk_usb_extcon_detect_cable(struct work_struct *work)
{
	struct mtk_extcon_info *extcon = container_of(to_delayed_work(work),
						    struct mtk_extcon_info,
						    wq_detcable);
	int id;

	/* check ID and update cable state */
	id = extcon->id_gpiod ?
		gpiod_get_value_cansleep(extcon->id_gpiod) : 1;

	/* at first we clean states which are no longer active */
	if (id) {
		mtk_usb_extcon_set_vbus(extcon, false);
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);
	} else {
		mtk_usb_extcon_set_vbus(extcon, true);
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
	}
}

static irqreturn_t mtk_usb_idpin_handle(int irq, void *dev_id)
{
	struct mtk_extcon_info *extcon = dev_id;

	/* issue detection work */
	queue_delayed_work(system_power_efficient_wq, &extcon->wq_detcable, 0);

	return IRQ_HANDLED;
}

static int mtk_usb_extcon_id_pin_init(struct mtk_extcon_info *extcon)
{
	int ret = 0;
	int id;

	extcon->id_gpiod = devm_gpiod_get(extcon->dev, "id", GPIOD_IN);

	if (!extcon->id_gpiod || IS_ERR(extcon->id_gpiod)) {
		dev_err(extcon->dev, "failed to get id gpio\n");
		extcon->id_gpiod = NULL;
		return -EINVAL;
	}

	extcon->id_irq = gpiod_to_irq(extcon->id_gpiod);
	if (extcon->id_irq < 0) {
		dev_err(extcon->dev, "failed to get ID IRQ\n");
		extcon->id_gpiod = NULL;
		return -EINVAL;
	}

	INIT_DELAYED_WORK(&extcon->wq_detcable, mtk_usb_extcon_detect_cable);

	ret = devm_request_threaded_irq(extcon->dev, extcon->id_irq, NULL,
			mtk_usb_idpin_handle, IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			dev_name(extcon->dev), extcon);

	if (ret < 0) {
		dev_err(extcon->dev, "failed to request handler for ID IRQ\n");
		extcon->id_gpiod = NULL;
		return ret;
	}
	enable_irq_wake(extcon->id_irq);

	// get id pin value when boot on
	id = extcon->id_gpiod ?	gpiod_get_value_cansleep(extcon->id_gpiod) : 1;
	dev_info(extcon->dev, "id value : %d\n", id);
	if (!id) {
		mtk_usb_extcon_set_vbus(extcon, true);
		mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_HOST);
	}

	return 0;
}

static int mtk_usb_extcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_extcon_info *extcon;
	struct platform_device *conn_pdev;
	struct device_node *conn_np;
	int ret;

	extcon = devm_kzalloc(&pdev->dev, sizeof(*extcon), GFP_KERNEL);
	if (!extcon)
		return -ENOMEM;

	extcon->dev = dev;

	/* extcon */
	extcon->edev = devm_extcon_dev_allocate(dev, usb_extcon_cable);
	if (IS_ERR(extcon->edev)) {
		dev_err(dev, "failed to allocate extcon device\n");
		return -ENOMEM;
	}

	ret = devm_extcon_dev_register(dev, extcon->edev);
	if (ret < 0) {
		dev_info(dev, "failed to register extcon device\n");
		return ret;
	}

	/* usb role switch */
	conn_np = of_parse_phandle(dev->of_node, "dev-conn", 0);
	if (!conn_np) {
		dev_info(dev, "failed to get dev-conn node\n");
		return -EINVAL;
	}

	conn_pdev = of_find_device_by_node(conn_np);
	if (!conn_pdev) {
		dev_info(dev, "failed to get dev-conn pdev\n");
		return -EINVAL;
	}

	extcon->dev_conn.endpoint[0] = kasprintf(GFP_KERNEL,
				"%s-role-switch", dev_name(&conn_pdev->dev));
	extcon->dev_conn.endpoint[1] = dev_name(extcon->dev);
	extcon->dev_conn.id = "usb-role-switch";
	device_connection_add(&extcon->dev_conn);

	extcon->role_sw = usb_role_switch_get(extcon->dev);
	if (IS_ERR(extcon->role_sw)) {
		dev_err(dev, "failed to get usb role\n");
		return PTR_ERR(extcon->role_sw);
	}

	/* vbus */
	extcon->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(extcon->vbus)) {
		dev_err(dev, "failed to get vbus\n");
		return PTR_ERR(extcon->vbus);
	}

	if (!of_property_read_u32(dev->of_node, "vbus-voltage",
					&extcon->vbus_vol))
		dev_info(dev, "vbus-voltage=%d", extcon->vbus_vol);

	if (!of_property_read_u32(dev->of_node, "vbus-current",
					&extcon->vbus_cur))
		dev_info(dev, "vbus-current=%d", extcon->vbus_cur);

	extcon->bypss_typec_sink =
		of_property_read_bool(dev->of_node,
			"mediatek,bypss-typec-sink");

	extcon->extcon_wq = create_singlethread_workqueue("extcon_usb");
	if (!extcon->extcon_wq)
		return -ENOMEM;

	extcon->c_role = DUAL_PROP_DR_DEVICE;

	/* default initial role */
	mtk_usb_extcon_set_role(extcon, DUAL_PROP_DR_NONE);

	/* default turn off vbus */
	mtk_usb_extcon_set_vbus(extcon, false);

	/*get id resources*/
	ret = mtk_usb_extcon_id_pin_init(extcon);
	if (ret < 0)
		dev_info(dev, "failed to init id pin\n");

	/* power psy */
	ret = mtk_usb_extcon_psy_init(extcon);
	if (ret < 0)
		dev_err(dev, "failed to init psy\n");

#ifdef CONFIG_TCPC_CLASS
	/* tcpc */
	ret = mtk_usb_extcon_tcpc_init(extcon);
	if (ret < 0)
		dev_err(dev, "failed to init tcpc\n");
#endif

	platform_set_drvdata(pdev, extcon);

	return 0;
}

static int mtk_usb_extcon_remove(struct platform_device *pdev)
{
	struct mtk_extcon_info *extcon = platform_get_drvdata(pdev);

	if (extcon->dev_conn.id)
		device_connection_remove(&extcon->dev_conn);

	return 0;
}

static void mtk_usb_extcon_shutdown(struct platform_device *pdev)
{
	struct mtk_extcon_info *extcon = platform_get_drvdata(pdev);

	if (extcon->c_role == DUAL_PROP_DR_HOST) {
		dev_info(extcon->dev, "set host vbus off when shutdown\n");
		mtk_usb_extcon_set_vbus(extcon, false);
	}
}

static const struct of_device_id mtk_usb_extcon_of_match[] = {
	{ .compatible = "mediatek,extcon-usb", },
	{ },
};
MODULE_DEVICE_TABLE(of, mtk_usb_extcon_of_match);

static struct platform_driver mtk_usb_extcon_driver = {
	.probe		= mtk_usb_extcon_probe,
	.remove		= mtk_usb_extcon_remove,
	.shutdown	= mtk_usb_extcon_shutdown,
	.driver		= {
		.name	= "mtk-extcon-usb",
		.of_match_table = mtk_usb_extcon_of_match,
	},
};

static int __init mtk_usb_extcon_init(void)
{
	return platform_driver_register(&mtk_usb_extcon_driver);
}
late_initcall(mtk_usb_extcon_init);

static void __exit mtk_usb_extcon_exit(void)
{
	platform_driver_unregister(&mtk_usb_extcon_driver);
}
module_exit(mtk_usb_extcon_exit);

