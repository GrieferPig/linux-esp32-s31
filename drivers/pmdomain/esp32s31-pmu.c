// SPDX-License-Identifier: GPL-2.0-only
/*
 * ESP32-S31 PMU generic power domains.
 *
 * The radio payload still performs the IDF active-state PMU initialization.
 * Linux therefore owns only HPCNNT transitions; the remaining domains are
 * published for topology and attachment but deliberately kept on.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/esp32s31-radio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/soc/espressif/esp32s31-pmu.h>
#include <linux/sysfs.h>

#include <dt-bindings/power/esp32s31-power.h>

#define PMU_PD_TOP_CTRL		0x120
#define PMU_PD_HPALIVE_CTRL	0x124
#define PMU_PD_MODEMPWR_CTRL	0x128
#define PMU_PD_HPCPU_CTRL	0x12c
#define PMU_PD_HPCNNT_CTRL	0x130
#define PMU_PD_MODEM_CTRL	0x134
#define PMU_PD_LP_PERI_CTRL	0x138

#define PMU_FORCE_RESET		BIT(0)
#define PMU_FORCE_ISO		BIT(1)
#define PMU_FORCE_PU		BIT(2)
#define PMU_FORCE_NO_RESET	BIT(3)
#define PMU_FORCE_NO_ISO	BIT(4)
#define PMU_FORCE_PD		BIT(5)

struct esp32s31_pmu;

struct esp32s31_domain {
	struct generic_pm_domain genpd;
	struct esp32s31_pmu *pmu;
	u16 reg;
	bool controllable;
	bool powered;
	u32 power_on_count;
	u32 power_off_count;
	u32 reclaim_count;
	u32 transition_errors;
};

struct esp32s31_pmu {
	struct device *dev;
	void __iomem *base;
	struct mutex lock;
	struct esp32s31_domain domains[ESP32S31_PD_NR];
	struct generic_pm_domain *domain_ptrs[ESP32S31_PD_NR];
	struct genpd_onecell_data onecell;
	bool allow_hpcnnt_off;
	bool radio_active;
};

static struct esp32s31_pmu *esp32s31_pmu_owner;

#define to_esp32s31_domain(_genpd) \
	container_of(_genpd, struct esp32s31_domain, genpd)

static void esp32s31_pmu_update(struct esp32s31_pmu *pmu, u16 reg,
			       u32 clear, u32 set)
{
	u32 value = readl(pmu->base + reg);

	value &= ~clear;
	value |= set;
	writel(value, pmu->base + reg);
	readl(pmu->base + reg);
}

static int __esp32s31_domain_power_on(struct esp32s31_domain *domain)
{
	struct esp32s31_pmu *pmu = domain->pmu;
	u32 ctrl;

	/* Power first, then remove isolation, finally release reset. */
	esp32s31_pmu_update(pmu, domain->reg, PMU_FORCE_PD, PMU_FORCE_PU);
	udelay(2);
	esp32s31_pmu_update(pmu, domain->reg, PMU_FORCE_ISO,
			    PMU_FORCE_NO_ISO);
	udelay(1);
	esp32s31_pmu_update(pmu, domain->reg, PMU_FORCE_RESET,
			    PMU_FORCE_NO_RESET);
	ctrl = readl(pmu->base + domain->reg);
	if ((ctrl & (PMU_FORCE_PU | PMU_FORCE_NO_ISO |
		     PMU_FORCE_NO_RESET)) !=
		    (PMU_FORCE_PU | PMU_FORCE_NO_ISO | PMU_FORCE_NO_RESET) ||
	    (ctrl & (PMU_FORCE_PD | PMU_FORCE_ISO | PMU_FORCE_RESET))) {
		domain->transition_errors++;
		dev_err(pmu->dev, "%s power-on force readback failed: 0x%08x\n",
			domain->genpd.name, ctrl);
		return -EIO;
	}
	domain->powered = true;
	return 0;
}

static int __esp32s31_domain_power_off(struct esp32s31_domain *domain)
{
	struct esp32s31_pmu *pmu = domain->pmu;
	u32 ctrl;

	/* Reset before isolation; only then request power-down. */
	esp32s31_pmu_update(pmu, domain->reg, PMU_FORCE_NO_RESET,
			    PMU_FORCE_RESET);
	udelay(1);
	esp32s31_pmu_update(pmu, domain->reg, PMU_FORCE_NO_ISO,
			    PMU_FORCE_ISO);
	udelay(1);
	esp32s31_pmu_update(pmu, domain->reg, PMU_FORCE_PU, PMU_FORCE_PD);
	ctrl = readl(pmu->base + domain->reg);
	if ((ctrl & (PMU_FORCE_PD | PMU_FORCE_ISO | PMU_FORCE_RESET)) !=
		    (PMU_FORCE_PD | PMU_FORCE_ISO | PMU_FORCE_RESET) ||
	    (ctrl & (PMU_FORCE_PU | PMU_FORCE_NO_ISO |
		     PMU_FORCE_NO_RESET))) {
		domain->transition_errors++;
		dev_err(pmu->dev, "%s power-off force readback failed: 0x%08x\n",
			domain->genpd.name, ctrl);
		return -EIO;
	}
	domain->powered = false;
	return 0;
}

static int esp32s31_domain_power_on(struct generic_pm_domain *genpd)
{
	struct esp32s31_domain *domain = to_esp32s31_domain(genpd);
	int ret;

	if (!domain->controllable)
		return 0;

	mutex_lock(&domain->pmu->lock);
	ret = __esp32s31_domain_power_on(domain);
	if (!ret)
		domain->power_on_count++;
	mutex_unlock(&domain->pmu->lock);

	return ret;
}

static int esp32s31_domain_power_off(struct generic_pm_domain *genpd)
{
	struct esp32s31_domain *domain = to_esp32s31_domain(genpd);
	int ret;

	if (!domain->controllable)
		return -EBUSY;

	mutex_lock(&domain->pmu->lock);
	/* The linked IDF Wi-Fi/BT payload accesses HPCNNT outside a device. */
	if (domain->pmu->radio_active) {
		mutex_unlock(&domain->pmu->lock);
		return -EBUSY;
	}
	ret = __esp32s31_domain_power_off(domain);
	if (!ret)
		domain->power_off_count++;
	mutex_unlock(&domain->pmu->lock);

	return ret;
}

/*
 * The linked IDF payload calls pmu_init() once immediately before radio init.
 * It clears every force field, including HPCNNT. Re-apply the Linux-owned
 * state after that one global initialization so genpd and firmware cannot
 * silently disagree about the physical domain state.
 */
void s31_linux_pmu_reclaim_after_radio_init(void)
{
	struct esp32s31_pmu *pmu = READ_ONCE(esp32s31_pmu_owner);
	struct esp32s31_domain *domain;

	if (!pmu)
		return;

	domain = &pmu->domains[ESP32S31_PD_HPCNNT];
	mutex_lock(&pmu->lock);
	domain->reclaim_count++;
	if (domain->powered)
		(void)__esp32s31_domain_power_on(domain);
	else
		(void)__esp32s31_domain_power_off(domain);
	mutex_unlock(&pmu->lock);
}
EXPORT_SYMBOL_GPL(s31_linux_pmu_reclaim_after_radio_init);

void s31_linux_pmu_radio_vote(bool active)
{
	struct esp32s31_pmu *pmu = READ_ONCE(esp32s31_pmu_owner);

	if (!pmu)
		return;
	mutex_lock(&pmu->lock);
	pmu->radio_active = active;
	if (active)
		(void)__esp32s31_domain_power_on(
			&pmu->domains[ESP32S31_PD_HPCNNT]);
	mutex_unlock(&pmu->lock);
}
EXPORT_SYMBOL_GPL(s31_linux_pmu_radio_vote);

static const char * const esp32s31_domain_names[ESP32S31_PD_NR] = {
	[ESP32S31_PD_TOP] = "top",
	[ESP32S31_PD_HPALIVE] = "hp-alive",
	[ESP32S31_PD_MODEMPWR] = "modem-power",
	[ESP32S31_PD_HPCPU] = "hp-cpu",
	[ESP32S31_PD_HPCNNT] = "hp-connectivity",
	[ESP32S31_PD_MODEM] = "modem",
	[ESP32S31_PD_LP_PERI] = "lp-peripheral",
};

static const u16 esp32s31_domain_regs[ESP32S31_PD_NR] = {
	[ESP32S31_PD_TOP] = PMU_PD_TOP_CTRL,
	[ESP32S31_PD_HPALIVE] = PMU_PD_HPALIVE_CTRL,
	[ESP32S31_PD_MODEMPWR] = PMU_PD_MODEMPWR_CTRL,
	[ESP32S31_PD_HPCPU] = PMU_PD_HPCPU_CTRL,
	[ESP32S31_PD_HPCNNT] = PMU_PD_HPCNNT_CTRL,
	[ESP32S31_PD_MODEM] = PMU_PD_MODEM_CTRL,
	[ESP32S31_PD_LP_PERI] = PMU_PD_LP_PERI_CTRL,
};

static const char *esp32s31_force_state(u32 ctrl)
{
	if (ctrl & PMU_FORCE_PD)
		return "off";
	if (ctrl & PMU_FORCE_PU)
		return "on";
	return "firmware-auto";
}

static ssize_t domains_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct esp32s31_pmu *pmu = dev_get_drvdata(dev);
	ssize_t len = 0;
	int i;

	mutex_lock(&pmu->lock);
	for (i = 0; i < ESP32S31_PD_NR; i++) {
		struct esp32s31_domain *domain = &pmu->domains[i];
		u32 ctrl = readl(pmu->base + domain->reg);

		len += sysfs_emit_at(buf, len,
			"%s policy=%s software=%s hardware=%s force=0x%02x on=%u off=%u reclaim=%u errors=%u radio-vote=%u\n",
			domain->genpd.name,
			(domain->genpd.flags & GENPD_FLAG_ALWAYS_ON) ?
				"always-on" : "automatic",
			domain->powered ? "on" : "off",
			esp32s31_force_state(ctrl),
			(unsigned int)(ctrl & GENMASK(5, 0)),
			domain->power_on_count,
			domain->power_off_count, domain->reclaim_count,
			domain->transition_errors,
			i == ESP32S31_PD_HPCNNT && pmu->radio_active);
	}
	mutex_unlock(&pmu->lock);

	return len;
}
static DEVICE_ATTR_RO(domains);

static struct attribute *esp32s31_pmu_attrs[] = {
	&dev_attr_domains.attr,
	NULL,
};

static const struct attribute_group esp32s31_pmu_attr_group = {
	.attrs = esp32s31_pmu_attrs,
};

static int esp32s31_pmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct esp32s31_pmu *pmu;
	struct resource *res;
	bool allow_hpcnnt_off;
	int i, ret;

	pmu = devm_kzalloc(dev, sizeof(*pmu), GFP_KERNEL);
	if (!pmu)
		return -ENOMEM;

	pmu->dev = dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -EINVAL;
	/* Child regulator nodes own non-overlapping PMU words in this aperture. */
	pmu->base = devm_ioremap(dev, res->start, resource_size(res));
	if (!pmu->base)
		return -ENOMEM;
	mutex_init(&pmu->lock);

	allow_hpcnnt_off = of_property_read_bool(dev->of_node,
						 "espressif,allow-hpcnnt-power-off");
	pmu->allow_hpcnnt_off = allow_hpcnnt_off;
	pmu->radio_active = false;

	for (i = 0; i < ESP32S31_PD_NR; i++) {
		struct esp32s31_domain *domain = &pmu->domains[i];

		domain->pmu = pmu;
		domain->reg = esp32s31_domain_regs[i];
		domain->powered = true;
		domain->controllable = i == ESP32S31_PD_HPCNNT;
		domain->genpd.name = esp32s31_domain_names[i];
		domain->genpd.power_on = esp32s31_domain_power_on;
		domain->genpd.power_off = esp32s31_domain_power_off;
		if (!domain->controllable || !allow_hpcnnt_off)
			domain->genpd.flags |= GENPD_FLAG_ALWAYS_ON;

		ret = pm_genpd_init(&domain->genpd, NULL, false);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to initialize domain %s\n",
					     domain->genpd.name);
		pmu->domain_ptrs[i] = &domain->genpd;
	}

	pmu->onecell.domains = pmu->domain_ptrs;
	pmu->onecell.num_domains = ESP32S31_PD_NR;
	ret = of_genpd_add_provider_onecell(dev->of_node, &pmu->onecell);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register power-domain provider\n");

	/* TOP is the parent of the HP domains; MODEM depends on MODEMPWR. */
	pm_genpd_add_subdomain(&pmu->domains[ESP32S31_PD_TOP].genpd,
			       &pmu->domains[ESP32S31_PD_HPALIVE].genpd);
	pm_genpd_add_subdomain(&pmu->domains[ESP32S31_PD_TOP].genpd,
			       &pmu->domains[ESP32S31_PD_MODEMPWR].genpd);
	pm_genpd_add_subdomain(&pmu->domains[ESP32S31_PD_TOP].genpd,
			       &pmu->domains[ESP32S31_PD_HPCPU].genpd);
	pm_genpd_add_subdomain(&pmu->domains[ESP32S31_PD_TOP].genpd,
			       &pmu->domains[ESP32S31_PD_HPCNNT].genpd);
	pm_genpd_add_subdomain(&pmu->domains[ESP32S31_PD_MODEMPWR].genpd,
			       &pmu->domains[ESP32S31_PD_MODEM].genpd);

	/* Take explicit force ownership of HPCNNT before consumers attach. */
	mutex_lock(&pmu->lock);
	ret = __esp32s31_domain_power_on(
			&pmu->domains[ESP32S31_PD_HPCNNT]);
	if (!ret)
		pmu->domains[ESP32S31_PD_HPCNNT].power_on_count++;
	mutex_unlock(&pmu->lock);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to take HPCNNT force ownership\n");

	platform_set_drvdata(pdev, pmu);
	ret = devm_device_add_group(dev, &esp32s31_pmu_attr_group);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add domain state attributes\n");
	WRITE_ONCE(esp32s31_pmu_owner, pmu);
	dev_info(dev,
		 "HPCNNT runtime power-off %s, radio vote %s; modem domains firmware-owned\n",
		 allow_hpcnnt_off ? "enabled" : "guarded",
		 pmu->radio_active ? "active" : "inactive");

	return 0;
}

static const struct of_device_id esp32s31_pmu_of_match[] = {
	{ .compatible = "espressif,esp32s31-pmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, esp32s31_pmu_of_match);

static struct platform_driver esp32s31_pmu_driver = {
	.probe = esp32s31_pmu_probe,
	.driver = {
		.name = "esp32s31-pmu",
		.of_match_table = esp32s31_pmu_of_match,
	},
};
builtin_platform_driver(esp32s31_pmu_driver);

MODULE_DESCRIPTION("ESP32-S31 PMU power domains");
MODULE_LICENSE("GPL");
