// SPDX-License-Identifier: GPL-2.0-only
/* Shared load boundary for the ESP32-S31 radio runtime and IDF payload. */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/esp32s31-radio.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/stringify.h>

#include "esp32s31-radio-internal.h"

const char s31_radio_modinfo_anchor[] __used __section(".modinfo") =
	"esp32s31_radio_core_abi=" __stringify(ESP32S31_RADIO_CORE_ABI_VERSION);

static bool s31_radio_started;
static bool s31_radio_suspended;
static u32 s31_radio_configured_features;
static char *s31_radio_mode = "combo";
static bool s31_radio_direct_hci = true;
module_param_named(mode, s31_radio_mode, charp, 0400);
MODULE_PARM_DESC(mode, "radio profile: wifi, bt, or combo");
module_param_named(direct_hci, s31_radio_direct_hci, bool, 0400);
MODULE_PARM_DESC(direct_hci, "expose /dev/s31-hci instead of Linux hci0");

static ssize_t radio_health_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct esp32s31_radio_health health;
	int ret;

	ret = esp32s31_radio_get_health(&health);
	if (ret)
		return ret;
	return sysfs_emit(buf,
		"abi=%u state=%u wifi_init=%d bt_init=%d bt_enable=%d "
		"ticks=%u worker_passes=%u commands=%u irqs=%u "
		"heap_used=%u heap_peak=%u heap_total=%u "
		"wifi_rx_dropped=%u wifi_tx_dropped=%u "
		"hci_rx_dropped=%u hci_tx_dropped=%u "
		"acl_rx_packets=%llu acl_rx_bytes=%llu "
		"acl_tx_packets=%llu acl_tx_bytes=%llu "
		"acl_packet_type_events=%u acl_packet_type=0x%04x "
		"coex_cmd_completes=%u coex_last_status=%d "
		"coex_bt_status=0x%02x\n",
		ESP32S31_RADIO_CORE_ABI_VERSION, health.state,
		health.wifi_init_result, health.bt_init_result,
		health.bt_enable_result, health.tick_irqs,
		health.worker_passes, health.commands_completed,
		health.radio_irqs, health.heap_used, health.heap_peak,
		health.heap_total, health.wifi_rx_dropped,
		health.wifi_tx_dropped, health.hci_rx_dropped,
		health.hci_tx_dropped,
		(unsigned long long)health.hci_acl_rx_packets,
		(unsigned long long)health.hci_acl_rx_bytes,
		(unsigned long long)health.hci_acl_tx_packets,
		(unsigned long long)health.hci_acl_tx_bytes,
		health.acl_packet_type_events, health.acl_packet_type,
		health.coex_cmd_completes, health.coex_last_status,
		health.coex_bt_status);
}
static DEVICE_ATTR_RO(radio_health);

static int s31_radio_parse_mode(bool *enable_wifi, bool *enable_bt)
{
	if (!strcmp(s31_radio_mode, "wifi")) {
		*enable_wifi = true;
		*enable_bt = false;
	} else if (!strcmp(s31_radio_mode, "bt")) {
		*enable_wifi = false;
		*enable_bt = true;
	} else if (!strcmp(s31_radio_mode, "combo")) {
		*enable_wifi = true;
		*enable_bt = true;
	} else {
		return -EINVAL;
	}
	return 0;
}

static int esp32s31_radio_probe(struct platform_device *pdev)
{
	bool enable_bt, enable_wifi;
	int ret;

	ret = s31_radio_parse_mode(&enable_wifi, &enable_bt);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "invalid mode=%s (expected wifi, bt, or combo)\n",
				     s31_radio_mode);
	if (s31_radio_started)
		return dev_err_probe(&pdev->dev, -EBUSY,
				     "radio runtime already started\n");

	s31_radio_configured_features =
		(enable_wifi ? S31_RADIO_FEATURE_WIFI : 0) |
		(enable_bt ? S31_RADIO_FEATURE_BLUETOOTH : 0);
	ret = s31_radio_fw_load(&pdev->dev);
	if (ret)
		goto clear_core;
	ret = s31_radio_runtime_init(enable_wifi, enable_bt);
	if (ret)
		goto unload_firmware;
	s31_linux_pmu_radio_vote(true);
	s31_radio_started = true;
	ret = device_create_file(&pdev->dev, &dev_attr_radio_health);
	if (ret)
		dev_warn(&pdev->dev, "cannot create radio health attribute: %d\n",
			 ret);
	if (enable_wifi) {
		ret = s31_radio_wifi_frontend_init(&pdev->dev);
		if (ret)
			goto stop_runtime;
	}
	if (enable_bt) {
		ret = s31_radio_btdm_frontend_init(&pdev->dev,
						   s31_radio_direct_hci);
		if (ret)
			goto detach_wifi;
	}
	dev_info(&pdev->dev, "single module ABI %u mode=%s direct_hci=%u ready for:%s%s\n",
		 ESP32S31_RADIO_CORE_ABI_VERSION,
		 s31_radio_mode, s31_radio_direct_hci,
		 enable_wifi ? " wifi" : "", enable_bt ? " bluetooth" : "");
	return 0;

detach_wifi:
	if (enable_wifi)
		s31_radio_wifi_frontend_exit();
stop_runtime:
	device_remove_file(&pdev->dev, &dev_attr_radio_health);
	s31_radio_started = false;
	s31_linux_pmu_radio_vote(false);
	s31_radio_runtime_shutdown();
unload_firmware:
	s31_radio_fw_unload();
clear_core:
	s31_radio_configured_features = 0;
	return ret;
}

static void esp32s31_radio_remove(struct platform_device *pdev)
{
	if (s31_radio_started) {
		if (s31_radio_configured_features & S31_RADIO_FEATURE_WIFI)
			s31_radio_wifi_frontend_suspend();
		if (s31_radio_configured_features & S31_RADIO_FEATURE_BLUETOOTH)
			s31_radio_btdm_frontend_suspend();
		s31_radio_runtime_shutdown();
	}
	if (s31_radio_configured_features & S31_RADIO_FEATURE_BLUETOOTH)
		s31_radio_btdm_frontend_exit();
	if (s31_radio_configured_features & S31_RADIO_FEATURE_WIFI)
		s31_radio_wifi_frontend_exit();
	device_remove_file(&pdev->dev, &dev_attr_radio_health);
	s31_radio_started = false;
	s31_radio_suspended = false;
	s31_linux_pmu_radio_vote(false);
	s31_radio_fw_unload();
	s31_radio_configured_features = 0;
}

static int esp32s31_radio_resume(struct device *dev)
{
	bool wifi = s31_radio_configured_features & S31_RADIO_FEATURE_WIFI;
	bool bt = s31_radio_configured_features & S31_RADIO_FEATURE_BLUETOOTH;
	int ret;

	if (!s31_radio_suspended)
		return 0;
	ret = s31_radio_fw_reset();
	if (ret)
		return ret;
	s31_linux_pmu_radio_vote(true);
	ret = s31_radio_runtime_init(wifi, bt);
	if (ret)
		goto power_down;
	s31_radio_started = true;
	ret = s31_radio_runtime_wait_ready();
	if (!ret && wifi)
		ret = s31_radio_wifi_frontend_resume();
	if (!ret && bt)
		ret = s31_radio_btdm_frontend_resume();
	if (ret) {
		/* Re-detach any frontend already restored before freeing rings. */
		if (wifi)
			s31_radio_wifi_frontend_suspend();
		if (bt)
			s31_radio_btdm_frontend_suspend();
		s31_radio_runtime_shutdown();
		s31_radio_started = false;
		goto power_down;
	}
	s31_radio_suspended = false;
	dev_info(dev, "radio runtime restored; wireless links reconnect after suspend\n");
	return 0;
power_down:
	s31_linux_pmu_radio_vote(false);
	return dev_err_probe(dev, ret, "radio resume failed; interfaces remain detached\n");
}

static int esp32s31_radio_suspend(struct device *dev)
{
	bool wifi = s31_radio_configured_features & S31_RADIO_FEATURE_WIFI;
	bool bt = s31_radio_configured_features & S31_RADIO_FEATURE_BLUETOOTH;
	int ret, recover;

	if (!s31_radio_started)
		return 0;
	if (bt) {
		ret = s31_radio_btdm_frontend_suspend();
		if (ret)
			return ret;
	}
	if (wifi)
		s31_radio_wifi_frontend_suspend();
	/* Deinit the closed MAC/controller, stop all compatibility tasks, remove
	 * device IRQs, and free DMA rings before allowing APPWR to remove power.
	 * Keep frontend objects and the pristine payload data for warm restart. */
	ret = s31_radio_runtime_shutdown();
	s31_radio_started = false;
	s31_radio_suspended = true;
	if (ret) {
		recover = esp32s31_radio_resume(dev);
		if (recover)
			dev_err(dev, "radio rollback also failed: %d\n", recover);
		return -EIO;
	}
	s31_linux_pmu_radio_vote(false);
	return 0;
}

static void esp32s31_radio_shutdown(struct platform_device *pdev)
{
	if (s31_radio_started) {
		if (s31_radio_configured_features & S31_RADIO_FEATURE_WIFI)
			s31_radio_wifi_frontend_suspend();
		if (s31_radio_configured_features & S31_RADIO_FEATURE_BLUETOOTH)
			s31_radio_btdm_frontend_suspend();
		s31_radio_runtime_shutdown();
		s31_radio_started = false;
		s31_linux_pmu_radio_vote(false);
	}
}

static DEFINE_SIMPLE_DEV_PM_OPS(esp32s31_radio_pm_ops,
					esp32s31_radio_suspend, esp32s31_radio_resume);

static const struct of_device_id esp32s31_radio_of_match[] = {
	{ .compatible = "espressif,esp32s31-radio" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_radio_of_match);

static struct platform_driver esp32s31_radio_driver = {
	.probe = esp32s31_radio_probe,
	.remove = esp32s31_radio_remove,
	.shutdown = esp32s31_radio_shutdown,
	.driver = {
		.name = "esp32s31-radio",
		.of_match_table = esp32s31_radio_of_match,
		.pm = pm_sleep_ptr(&esp32s31_radio_pm_ops),
	},
};
module_platform_driver(esp32s31_radio_driver);

MODULE_DESCRIPTION("ESP32-S31 reloadable Wi-Fi/Bluetooth radio module");
MODULE_AUTHOR("S31 Linux contributors");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("4");
