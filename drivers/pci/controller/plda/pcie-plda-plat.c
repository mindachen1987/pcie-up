// SPDX-License-Identifier: GPL-2.0
/*
 * PLDA XpressRich PCIe platform driver
 *
 * Authors: Minda Chen <minda.chen@starfivetech.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>

#include "pcie-plda.h"

struct plda_plat_pcie {
	struct plda_pcie	*pci;
};

static struct pci_ops plda_default_ops = {
	.map_bus	= plda_pcie_map_bus,
	.read		= pci_generic_config_read,
	.write		= pci_generic_config_write,
};

static int plda_plat_add_pcie_port(struct plda_plat_pcie *plda_plat_pcie,
				   struct platform_device *pdev)
{
	struct plda_pcie *pci = plda_plat_pcie->pci;
	struct device *dev = &pdev->dev;
	int ret;

	ret = plda_pcie_host_init(pci, &plda_default_ops);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int plda_plat_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct plda_plat_pcie *plda_plat_pcie;
	struct plda_pcie *pci;
	int ret;

	plda_plat_pcie = devm_kzalloc(dev, sizeof(*plda_plat_pcie), GFP_KERNEL);
	if (!plda_plat_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;

	plda_plat_pcie->pci = pci;

	plda_plat_add_pcie_port(plda_plat_pcie, pdev);

	platform_set_drvdata(pdev, plda_plat_pcie);

	return ret;
}

static const struct of_device_id plda_plat_pcie_of_match[] = {
	{ .compatible = "plda,xpressrich-pcie"},
	{ /* sentinel */ }
};

static struct platform_driver plda_plat_pcie_driver = {
	.driver = {
		.name	= "plda-xpressrich-pcie",
		.of_match_table = plda_plat_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = plda_plat_pcie_probe,
};
builtin_platform_driver(plda_plat_pcie_driver);
