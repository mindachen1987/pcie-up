// SPDX-License-Identifier: GPL-2.0+
/*
 * PCIe host controller driver for Starfive JH7110 Soc.
 *
 * Copyright (C) StarFive Technology Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include "../../pci.h"

#include "pcie-plda.h"

#define DATA_LINK_ACTIVE		BIT(5)
#define PREF_MEM_WIN_64_SUPPORT		BIT(3)
#define PMSG_LTR_SUPPORT		BIT(2)
#define LINK_SPEED_GEN2			BIT(12)
#define PHY_FUNCTION_DIS		BIT(15)
#define PCIE_FUNC_NUM			4
#define PHY_FUNC_SHIFT			9

/* system control */
#define STG_SYSCON_K_RP_NEP			BIT(8)
#define STG_SYSCON_AXI4_SLVL_ARFUNC_MASK	GENMASK(22, 8)
#define STG_SYSCON_AXI4_SLVL_ARFUNC_SHIFT	8
#define STG_SYSCON_AXI4_SLVL_AWFUNC_MASK	GENMASK(14, 0)
#define STG_SYSCON_CLKREQ			BIT(22)
#define STG_SYSCON_CKREF_SRC_SHIFT		18
#define STG_SYSCON_CKREF_SRC_MASK		GENMASK(19, 18)

struct starfive_jh7110_pcie {
	struct plda_pcie	plda;
	struct reset_control *resets;
	struct clk_bulk_data *clks;
	struct regmap *reg_syscon;
	struct gpio_desc *power_gpio;
	struct gpio_desc *reset_gpio;

	u32 stg_arfun;
	u32 stg_awfun;
	u32 stg_rp_nep;
	u32 stg_lnksta;

	int num_clks;
};

/*
 * StarFive PCIe port uses BAR0-BAR1 of RC's configuration space as
 * the translation from PCI bus to native BUS.  Entire DDR region
 * is mapped into PCIe space using these registers, so it can be
 * reached by DMA from EP devices.  The BAR0/1 of bridge should be
 * hidden during enumeration to avoid the sizing and resource allocation
 * by PCIe core.
 */
static bool starfive_pcie_hide_rc_bar(struct pci_bus *bus, unsigned int  devfn,
				      int offset)
{
	if (pci_is_root_bus(bus) && !devfn &&
	    (offset == PCI_BASE_ADDRESS_0 || offset == PCI_BASE_ADDRESS_1))
		return true;

	return false;
}

int starfive_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
			       int where, int size, u32 value)
{
	if (starfive_pcie_hide_rc_bar(bus, devfn, where))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return pci_generic_config_write(bus, devfn, where, size, value);
}

static int starfive_pcie_parse_dt(struct starfive_jh7110_pcie *pcie, struct device *dev)
{
	unsigned int args[4];

	pcie->num_clks = devm_clk_bulk_get_all(dev, &pcie->clks);
	if (pcie->num_clks < 0)
		return dev_err_probe(dev, -ENODEV,
			"Failed to get pcie clocks\n");

	pcie->resets = devm_reset_control_array_get_exclusive(dev);
	if (IS_ERR(pcie->resets))
		return dev_err_probe(dev, PTR_ERR(pcie->resets),
			"Failed to get pcie resets");

	pcie->reg_syscon =
		syscon_regmap_lookup_by_phandle_args(dev->of_node,
						     "starfive,stg-syscon", 4, args);

	if (IS_ERR(pcie->reg_syscon))
		return dev_err_probe(dev, PTR_ERR(pcie->reg_syscon),
			"Failed to parse starfive,stg-syscon\n");

	pcie->stg_arfun = args[0];
	pcie->stg_awfun = args[1];
	pcie->stg_rp_nep = args[2];
	pcie->stg_lnksta = args[3];

	pcie->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(pcie->reset_gpio)) {
		dev_warn(dev, "Failed to get reset-gpio.\n");
		return -EINVAL;
	}

	pcie->power_gpio = devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_LOW);
	if (IS_ERR_OR_NULL(pcie->power_gpio))
		pcie->power_gpio = NULL;

	return 0;
}

static struct pci_ops starfive_pcie_ops = {
	.map_bus	= plda_pcie_map_bus,
	.read           = pci_generic_config_read,
	.write          = starfive_pcie_config_write,
};

static int starfive_pcie_clk_rst_init(struct starfive_jh7110_pcie *pcie)
{
	int ret;
	struct device *dev = pcie->plda.dev;

	ret = clk_bulk_prepare_enable(pcie->num_clks, pcie->clks);
	if (ret) {
		dev_err(dev, "Failed to enable clocks\n");
		return ret;
	}

	ret = reset_control_deassert(pcie->resets);
	if (ret) {
		clk_bulk_disable_unprepare(pcie->num_clks, pcie->clks);
		dev_err(dev, "Failed to resets\n");
	}

	return ret;
}

static void starfive_pcie_clk_rst_deinit(struct starfive_jh7110_pcie *pcie)
{
	reset_control_assert(pcie->resets);
	clk_bulk_disable_unprepare(pcie->num_clks, pcie->clks);
}

static int starfive_pcie_is_link_up(struct starfive_jh7110_pcie *pcie)
{
	struct device *dev = pcie->plda.dev;
	int ret;
	u32 stg_reg_val;

	/* 100ms timeout value should be enough for Gen1/2 training */
	ret = regmap_read_poll_timeout(pcie->reg_syscon,
				       pcie->stg_lnksta,
				       stg_reg_val,
				       stg_reg_val & DATA_LINK_ACTIVE,
				       10 * 1000, 100 * 1000);

	/* If the link is down (no device in slot), then exit. */
	if (ret == -ETIMEDOUT) {
		dev_info(dev, "Port link down, exit.\n");
		return 0;
	} else if (ret == 0) {
		dev_info(dev, "Port link up.\n");
		return 1;
	}

	return 0;
}

static void starfive_pcie_host_deinit(struct plda_pcie *plda)
{
	struct starfive_jh7110_pcie *pcie =
		container_of(plda, struct starfive_jh7110_pcie, plda);

	starfive_pcie_clk_rst_deinit(pcie);
	if (pcie->power_gpio)
		gpiod_set_value_cansleep(pcie->power_gpio, 0);
}

static int starfive_pcie_host_init(struct plda_pcie *plda)
{
	int i;
	struct starfive_jh7110_pcie *pcie =
		container_of(plda, struct starfive_jh7110_pcie, plda);
	int ret;

	regmap_update_bits(pcie->reg_syscon, pcie->stg_rp_nep,
			   STG_SYSCON_K_RP_NEP, STG_SYSCON_K_RP_NEP);

	regmap_update_bits(pcie->reg_syscon, pcie->stg_awfun,
			   STG_SYSCON_CKREF_SRC_MASK,
			   2 << STG_SYSCON_CKREF_SRC_SHIFT);

	regmap_update_bits(pcie->reg_syscon, pcie->stg_awfun,
			   STG_SYSCON_CLKREQ, STG_SYSCON_CLKREQ);

	ret = starfive_pcie_clk_rst_init(pcie);
	if (ret)
		return ret;

	if (pcie->power_gpio)
		gpiod_set_value_cansleep(pcie->power_gpio, 1);

	if (pcie->reset_gpio)
		gpiod_set_value_cansleep(pcie->reset_gpio, 1);

	/* Disable physical functions except #0 */
	for (i = 1; i < PCIE_FUNC_NUM; i++) {
		regmap_update_bits(pcie->reg_syscon,
				   pcie->stg_arfun,
				   STG_SYSCON_AXI4_SLVL_ARFUNC_MASK,
				   (i << PHY_FUNC_SHIFT) <<
				   STG_SYSCON_AXI4_SLVL_ARFUNC_SHIFT);
		regmap_update_bits(pcie->reg_syscon,
				   pcie->stg_awfun,
				   STG_SYSCON_AXI4_SLVL_AWFUNC_MASK,
				   i << PHY_FUNC_SHIFT);

		plda_pcie_disable_func(plda);
	}

	regmap_update_bits(pcie->reg_syscon, pcie->stg_arfun,
			   STG_SYSCON_AXI4_SLVL_ARFUNC_MASK, 0);
	regmap_update_bits(pcie->reg_syscon, pcie->stg_awfun,
			   STG_SYSCON_AXI4_SLVL_AWFUNC_MASK, 0);

	/* Enable root port */
	plda_pcie_enable_root_port(plda);

	/* PCIe PCI Standard Configuration Identification Settings. */
	plda_pcie_set_standard_class(plda);

	/*
	 * The LTR message forwarding of PCIe Message Reception was set by core
	 * as default, but the forward id & addr are also need to be reset.
	 * If we do not disable LTR message forwarding here, or set a legal
	 * forwarding address, the kernel will get stuck after this driver probe.
	 * To workaround, disable the LTR message forwarding support on
	 * PCIe Message Reception.
	 */
	plda_pcie_disable_ltr(plda);

	/* Prefetchable memory window 64-bit addressing support */
	plda_pcie_set_pref_win_64bit(plda);
	/*
	 * As the two host bridges in JH7110 soc have the same default
	 * address translation table, this cause the second root port can't
	 * access it's host bridge config space correctly.
	 * To workaround, config the ATR of host bridge config space by SW.
	 */
	plda_pcie_setup_window(plda->bridge_addr, 0, plda->config_phyaddr, 0,
			       plda->cfg_size);

	plda_pcie_setup_iomems(plda, plda->bridge);

	/* Ensure that PERST has been asserted for at least 100 ms */
	msleep(300);
	if (pcie->reset_gpio)
		gpiod_set_value_cansleep(pcie->reset_gpio, 0);

	if (!starfive_pcie_is_link_up(pcie))
		return -EIO;

	return ret;
}

static const struct plda_pcie_ops pcie_ops = {
	.host_init = starfive_pcie_host_init,
	.host_deinit = starfive_pcie_host_deinit,
};

static int starfive_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct starfive_jh7110_pcie *pcie;
	struct plda_pcie *plda;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	plda = &pcie->plda;
	plda->dev = dev;

	ret = starfive_pcie_parse_dt(pcie, dev);
	if (ret)
		return ret;

	plda->ops = &pcie_ops;
	plda->num_events = NUM_PLDA_EVENTS;
	ret = plda_pcie_host_init(&pcie->plda, &starfive_pcie_ops);
	if (ret)
		return ret;

	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
	platform_set_drvdata(pdev, pcie);

	return 0;
}

static int starfive_pcie_remove(struct platform_device *pdev)
{
	struct starfive_jh7110_pcie *pcie = platform_get_drvdata(pdev);

	plda_pcie_host_deinit(&pcie->plda);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int __maybe_unused starfive_pcie_suspend_noirq(struct device *dev)
{
	struct starfive_jh7110_pcie *pcie = dev_get_drvdata(dev);

	if (!pcie)
		return 0;

	clk_bulk_disable_unprepare(pcie->num_clks, pcie->clks);
	plda_pcie_disable_phy(&pcie->plda);

	return 0;
}

static int __maybe_unused starfive_pcie_resume_noirq(struct device *dev)
{
	struct starfive_jh7110_pcie *pcie = dev_get_drvdata(dev);
	int ret;

	if (!pcie)
		return 0;

	ret = plda_pcie_enable_phy(dev, &pcie->plda);
	if (ret)
		return ret;

	ret = clk_bulk_prepare_enable(pcie->num_clks, pcie->clks);
	if (ret) {
		dev_err(dev, "Failed to enable clocks\n");
		plda_pcie_disable_phy(&pcie->plda);
		return ret;
	}

	return ret;
}

static const struct dev_pm_ops starfive_pcie_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(starfive_pcie_suspend_noirq,
				      starfive_pcie_resume_noirq)
};
#endif

static const struct of_device_id starfive_pcie_of_match[] = {
	{ .compatible = "starfive,jh7110-pcie"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, starfive_pcie_of_match);

static struct platform_driver starfive_pcie_driver = {
	.driver = {
		.name = "pcie-starfive",
		.of_match_table = of_match_ptr(starfive_pcie_of_match),
#ifdef CONFIG_PM_SLEEP
		.pm = &starfive_pcie_pm_ops,
#endif
	},
	.probe = starfive_pcie_probe,
	.remove = starfive_pcie_remove,
};
module_platform_driver(starfive_pcie_driver);

MODULE_DESCRIPTION("StarFive JH7110 PCIe host driver");
MODULE_LICENSE("GPL v2");
